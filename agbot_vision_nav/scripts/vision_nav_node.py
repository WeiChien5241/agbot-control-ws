#!/usr/bin/python3
"""ROS1 node: segmentation mask -> row-centering cmd_vel.

Only this file (and its cv_bridge/ROS message handling) touches rospy/ROS
types. All perception/control logic lives in agbot_vision_nav's rospy-free
modules (segmentation_model, centerline_estimator, controller, debug_viz),
which are independently unit-tested.

Architecture: the camera subscriber only stashes the latest frame
(single-slot, overwrite-not-queue, guarded by a Condition). A separate
inference thread waits for a new frame, runs the model + control law, and
publishes cmd_vel right after each successful inference -- decoupled from
camera framerate, so a slow model never lets the subscriber's callback queue
back up. A 10 Hz keep-alive timer republishes the last computed command
between inferences (the Jackal base brakes if cmd_vel goes silent for a few
hundred ms, so a slow model must not starve it) and publishes zero Twist
instead once no successful inference has completed within max_data_age_sec.
"""

import math
import threading
import time

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage, Image, Joy
from std_msgs.msg import String
from std_srvs.srv import SetBool, SetBoolResponse

from agbot_vision_nav.centerline_estimator import estimate_centerline
from agbot_vision_nav.controller import MPCRowController
from agbot_vision_nav.debug_viz import render_debug_image
from agbot_vision_nav.intervention_detector import (
    DEFAULT_DEADMAN_BUTTONS,
    InterventionDetector,
)
from agbot_vision_nav.metrics_logger import (
    RunMetricsLogger,
    format_summary,
    join_list,
    make_run_path,
    read_csv,
    summarize,
)
from agbot_vision_nav.mission_fsm import (
    STATE_BACKOUT,
    STATE_EXIT_CLEAR,
    STATE_FOLLOW_ROW,
    MissionFSM,
)
from agbot_vision_nav.row_exit_detector import RowExitDetector
from agbot_vision_nav.segmentation_model import SegmentationModel
from agbot_vision_nav.timing_stats import PipelineTimingStats

# How long the inference loop waits for a rear frame before falling back to
# the front camera. Long enough that a working rear camera is never skipped
# (30 Hz in sim, ~10 Hz on the Brio), short enough that a DEAD one degrades
# within one control cycle instead of hanging the loop.
_REAR_FRAME_WAIT_SEC = 0.5

def _quaternion_to_yaw(q):
    """Yaw from a geometry_msgs/Quaternion (avoids a tf dependency)."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class VisionNavNode(object):
    def __init__(self):
        rospy.init_node("vision_nav_node")

        try:
            model_path = rospy.get_param("~model_path")
        except KeyError:
            rospy.logfatal(
                "~model_path is required (no default) -- set it via the launch "
                "file or `rosparam set`."
            )
            raise

        camera_topic = rospy.get_param("~camera_topic", "/usb_cam/image_raw/compressed")
        camera_topic_is_compressed = rospy.get_param("~camera_topic_is_compressed", True)
        cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        debug_image_topic = rospy.get_param("~debug_image_topic", "~debug/image")

        self._scan_row_fractions = rospy.get_param(
            "~scan_row_fractions", [0.65, 0.78, 0.92]
        )
        self._scan_row_weights = rospy.get_param("~scan_row_weights", [0.2, 0.3, 0.5])
        self._min_traversable_fraction = rospy.get_param(
            "~min_traversable_fraction", 0.10
        )
        # Optional dedicated scan rows for the exit detector (empty = share
        # the steering rows). Weights are irrelevant to the detector, which
        # reads per-row corridors, so default them to uniform.
        self._exit_scan_row_fractions = rospy.get_param(
            "~exit_scan_row_fractions", []
        )
        self._exit_scan_row_weights = rospy.get_param(
            "~exit_scan_row_weights", []
        ) or [1.0] * len(self._exit_scan_row_fractions)
        self._flank_margin_frac = rospy.get_param("~exit_flank_edge_margin", 0.05)
        self._max_data_age_sec = rospy.get_param("~max_data_age_sec", 0.5)
        self._publish_debug_image = rospy.get_param("~publish_debug_image", True)

        self._controller = MPCRowController(
            N=rospy.get_param("~mpc_horizon", 8),
            dt=rospy.get_param("~mpc_dt", 0.1),
            alpha=rospy.get_param("~mpc_alpha", 0.10),
            beta=rospy.get_param("~mpc_beta", 0.10),
            q_offset=rospy.get_param("~mpc_q_offset", 10.0),
            q_heading=rospy.get_param("~mpc_q_heading", 1.0),
            r_control=rospy.get_param("~mpc_r_control", 0.1),
            r_delta=rospy.get_param("~mpc_r_delta", 0.5),
            linear_x_cruise=rospy.get_param("~linear_x_cruise", 0.15),
            angular_z_max=rospy.get_param("~angular_z_max", 0.3),
            delta_angular_z_max=rospy.get_param("~delta_angular_z_max", 0.2),
            invalid_frame_stop_count=rospy.get_param("~invalid_frame_stop_count", 5),
        )

        # Optional multi-row mission mode (headland turns). Default off:
        # plain row-following, identical to pre-mission behavior.
        self._mission_enabled = rospy.get_param("~mission_enabled", False)

        # Optional rear camera: only consulted during the blocked-row
        # back-out (STATE_BACKOUT); normal operation runs front-only.
        # Without it the back-out is disabled entirely -- a blocked signal
        # then stops the robot and ends the mission.
        self._rear_camera_enabled = self._mission_enabled and rospy.get_param(
            "~rear_camera_enabled", False
        )
        rear_camera_topic = rospy.get_param("~rear_camera_topic", "/camera_rear/image_raw")
        rear_camera_topic_is_compressed = rospy.get_param(
            "~rear_camera_topic_is_compressed", False
        )

        self._fsm = None
        self._detector = None
        if self._mission_enabled:
            detector = RowExitDetector(
                exit_width_threshold=rospy.get_param("~exit_width_threshold", 0.8),
                exit_confirm_distance=rospy.get_param(
                    "~exit_confirm_distance", 0.4
                ),
                min_in_row_distance=rospy.get_param("~min_in_row_distance", 2.0),
                blocked_min_obstacle_fraction=rospy.get_param(
                    "~blocked_min_obstacle_fraction", 0.2
                ),
                blocked_arming_distance=rospy.get_param(
                    "~blocked_arming_distance", 0.3
                ),
                exit_open_rows_required=rospy.get_param(
                    "~exit_open_rows_required", 1
                ),
                blocked_confirm_seconds=rospy.get_param(
                    "~blocked_confirm_seconds", 4.0
                ),
                exit_flank_edge_margin=rospy.get_param(
                    "~exit_flank_edge_margin", 0.05
                ),
                exit_flank_min_clear_fraction=rospy.get_param(
                    "~exit_flank_min_clear_fraction", 0.8
                ),
                exit_detect_min_frames=rospy.get_param(
                    "~exit_detect_min_frames", 2
                ),
                exit_leak_ratio=rospy.get_param("~exit_leak_ratio", 0.5),
                blocked_leak_ratio=rospy.get_param("~blocked_leak_ratio", 1.0),
            )
            self._detector = detector
            self._fsm = MissionFSM(
                self._controller,
                detector,
                num_rows=rospy.get_param("~num_rows", 3),
                first_turn_direction=rospy.get_param("~first_turn_direction", "left"),
                row_spacing=rospy.get_param("~row_spacing", 0.75),
                traverse_distance=rospy.get_param("~traverse_distance", 0.6),
                headland_clearance=rospy.get_param("~headland_clearance", 1.0),
                turn_rate=rospy.get_param("~turn_rate", 0.4),
                yaw_tolerance_deg=rospy.get_param("~yaw_tolerance_deg", 5.0),
                reacquire_speed=rospy.get_param("~reacquire_speed", 0.08),
                reacquire_confirm_distance=rospy.get_param(
                    "~reacquire_confirm_distance", 0.12
                ),
                reacquire_steering_enabled=rospy.get_param(
                    "~reacquire_steering_enabled", True
                ),
                reacquire_max_distance=rospy.get_param("~reacquire_max_distance", 2.0),
                backout_speed=rospy.get_param("~backout_speed", 0.10),
                backout_enabled=self._rear_camera_enabled,
                exit_clear_speed=rospy.get_param("~exit_clear_speed", 0.10),
                exit_revoke_enabled=rospy.get_param("~exit_revoke_enabled", True),
                exit_revoke_distance=rospy.get_param("~exit_revoke_distance", 0.5),
                exit_revoke_fail_distance=rospy.get_param(
                    "~exit_revoke_fail_distance", 0.25
                ),
                exit_clear_min_distance=rospy.get_param(
                    "~exit_clear_min_distance", 0.2
                ),
                # Rear-steered headland leg. ANDed with the rear camera the
                # same way backout_enabled is: without a rear camera there is
                # nothing to steer from, and the FSM falls back to the
                # open-loop, headland_clearance-terminated leg.
                exit_clear_rear_steering=self._rear_camera_enabled
                and rospy.get_param("~exit_clear_rear_steering", True),
                exit_clear_post_rear_distance=rospy.get_param(
                    "~exit_clear_post_rear_distance", 0.2
                ),
                exit_clear_max_distance=rospy.get_param(
                    "~exit_clear_max_distance", 1.5
                ),
                exit_clear_rear_offset_gain=rospy.get_param(
                    "~exit_clear_rear_offset_gain", 2.0
                ),
                exit_clear_angular_z_max=rospy.get_param(
                    "~exit_clear_angular_z_max", 0.08
                ),
                exit_clear_max_yaw_deg=rospy.get_param(
                    "~exit_clear_max_yaw_deg", 20.0
                ),
            )

        rospy.loginfo("Loading segmentation model from %s ...", model_path)
        self._model = SegmentationModel(
            model_path, device=rospy.get_param("~model_device", "auto")
        )
        rospy.loginfo(
            "Model loaded on device: %s (a GPU robot showing 'cpu' here is "
            "the reason inference is slow)",
            self._model.device_str,
        )

        self._bridge = CvBridge()

        self._frame_condition = threading.Condition()
        # Slots hold (frame, header_stamp_sec or None, recv_time_sec);
        # timestamps feed the pipeline timing stats.
        self._latest_frame = None
        self._latest_frame_seq = 0
        self._latest_rear_frame = None
        self._latest_rear_frame_seq = 0
        # Which camera the next EXIT_CLEAR frame comes from. Flipped once per
        # frame actually dispatched, never per wakeup: _use_rear_camera() is
        # called in a spin loop that may go round many times waiting for a
        # frame, and flipping there would thrash between the two slots.
        self._exit_clear_next_is_rear = True
        self._mission_done_logged = False
        self._revoked_logged = 0
        self._blocked_logged = 0
        self._watchdog_stale = False
        self._timing = PipelineTimingStats()

        # Per-run CSV metrics. ON by default: a run you forgot to instrument
        # is a run you cannot report on, and there is no second chance at a
        # field pass. Set ~metrics_csv_dir to "" to disable.
        self._metrics = None
        # "none" as well as "": the launch file's conditional-<param> pattern
        # skips a <param> whose arg is empty, so an empty string can only ever
        # come from params.yaml. "none" is what makes the launch ARG able to
        # turn this off for a single run.
        metrics_dir = rospy.get_param("~metrics_csv_dir", "~/agbot_logs")
        if metrics_dir and str(metrics_dir).strip().lower() != "none":
            self._metrics = RunMetricsLogger(make_run_path(metrics_dir))
            for message in self._metrics.errors:
                rospy.logwarn("metrics CSV disabled: %s", message)
            if not self._metrics.errors:
                rospy.loginfo("metrics CSV: %s", self._metrics.path)
            rospy.on_shutdown(self._on_shutdown)

        self._last_success_lock = threading.Lock()
        self._last_success_time = None
        self._last_cmd = (0.0, 0.0)

        self._odom_lock = threading.Lock()
        self._odom_pose = None  # (x, y, yaw)
        # Subscribed on EVERY run, not just missions: the mission FSM is one
        # consumer, the run's path length is the other, and distance travelled
        # is the denominator of every autonomy number we report. A plain
        # row-following run with no odometry column can never be quoted in
        # meters afterwards, and field runs do not come back.
        odom_topic = rospy.get_param("~odom_topic", "/odometry/filtered")
        rospy.Subscriber(odom_topic, Odometry, self._odom_cb, queue_size=1)

        # Human-intervention counter. An intervention is a joystick takeover
        # (deadman held on the teleop pad) -- see intervention_detector.py for
        # why that, and why activity is collapsed into windows rather than
        # counted per button edge. Off with intervention_joy_topic:=none.
        self._interventions = None
        joy_topic = rospy.get_param("~intervention_joy_topic", "/bluetooth_teleop/joy")
        if joy_topic and str(joy_topic).strip().lower() != "none":
            self._interventions = InterventionDetector(
                deadman_buttons=rospy.get_param(
                    "~intervention_deadman_buttons", list(DEFAULT_DEADMAN_BUTTONS)
                ),
                axis_deadzone=rospy.get_param("~intervention_axis_deadzone", 0.15),
                gap_seconds=rospy.get_param("~intervention_gap_seconds", 3.0),
                hold_seconds=rospy.get_param("~intervention_hold_seconds", 0.5),
            )
            rospy.Subscriber(joy_topic, Joy, self._joy_cb, queue_size=10)

        # Operator pause. The point is that the mission SURVIVES it: Ctrl-C
        # and relaunch loses rows_driven, the turn direction, the detector's
        # arming distance and the row-entry pose, so a mid-run stop used to
        # mean restarting the mission from scratch. Paused publishes zero and
        # skips the FSM update entirely, so the state machine is frozen rather
        # than reset. See _set_paused() for what resuming has to undo.
        self._paused = False
        self._pause_lock = threading.Lock()
        rospy.Service("~pause", SetBool, self._pause_srv)
        self._status_pub = rospy.Publisher(
            "~status", String, queue_size=1, latch=True
        )
        rospy.Timer(rospy.Duration(0.5), self._publish_status)

        self._cmd_vel_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)
        self._debug_pub = None
        self._debug_rear_pub = None
        if self._publish_debug_image:
            self._debug_pub = rospy.Publisher(debug_image_topic, Image, queue_size=1)
            # Rear frames get their own topic so neither view ever flips (see
            # _publish_debug). Created unconditionally: the rear camera can be
            # enabled by a launch arg, and a topic nobody publishes to is free.
            self._debug_rear_pub = rospy.Publisher(
                debug_image_topic.rstrip("/") + "_rear", Image, queue_size=1
            )

        if camera_topic_is_compressed:
            rospy.Subscriber(
                camera_topic,
                CompressedImage,
                self._compressed_image_cb,
                queue_size=1,
                buff_size=2 ** 24,
            )
        else:
            rospy.Subscriber(
                camera_topic, Image, self._image_cb, queue_size=1, buff_size=2 ** 24
            )

        if self._rear_camera_enabled:
            if rear_camera_topic_is_compressed:
                rospy.Subscriber(
                    rear_camera_topic,
                    CompressedImage,
                    self._rear_compressed_image_cb,
                    queue_size=1,
                    buff_size=2 ** 24,
                )
            else:
                rospy.Subscriber(
                    rear_camera_topic,
                    Image,
                    self._rear_image_cb,
                    queue_size=1,
                    buff_size=2 ** 24,
                )
            rospy.loginfo("Rear camera enabled on %s", rear_camera_topic)

        self._inference_thread = threading.Thread(target=self._inference_loop)
        self._inference_thread.daemon = True
        self._inference_thread.start()

        rospy.Timer(rospy.Duration(1.0 / 10.0), self._watchdog_cb)

        self._log_config()
        rospy.loginfo("vision_nav_node ready, listening on %s", camera_topic)

    @staticmethod
    def _stamp_sec(msg):
        """Header stamp in seconds, or None when the publisher left it unset."""
        stamp = msg.header.stamp.to_sec()
        return stamp if stamp > 0.0 else None

    def _compressed_image_cb(self, msg):
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            rospy.logwarn_throttle(5.0, "Failed to decode CompressedImage frame")
            return
        self._store_frame(frame, self._stamp_sec(msg))

    def _image_cb(self, msg):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "cv_bridge conversion failed: %s", exc)
            return
        self._store_frame(frame, self._stamp_sec(msg))

    def _rear_compressed_image_cb(self, msg):
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            rospy.logwarn_throttle(5.0, "Failed to decode rear CompressedImage frame")
            return
        self._store_rear_frame(frame, self._stamp_sec(msg))

    def _rear_image_cb(self, msg):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "rear cv_bridge conversion failed: %s", exc)
            return
        self._store_rear_frame(frame, self._stamp_sec(msg))

    def _odom_cb(self, msg):
        pose = msg.pose.pose
        yaw = _quaternion_to_yaw(pose.orientation)
        with self._odom_lock:
            self._odom_pose = (pose.position.x, pose.position.y, yaw)

    def _joy_cb(self, msg):
        """Count joystick takeovers. Runs on the joy subscriber's thread."""
        now = rospy.Time.now().to_sec()
        if self._interventions.update(now, msg.buttons, msg.axes):
            rospy.logwarn(
                "human intervention #%d (joystick takeover)",
                self._interventions.count,
            )
            if self._metrics is not None:
                self._metrics.mark_event("INTERVENTION")

    def _store_frame(self, frame, stamp):
        recv_time = rospy.Time.now().to_sec()
        with self._frame_condition:
            self._latest_frame = (frame, stamp, recv_time)
            self._latest_frame_seq += 1
            self._timing.record_camera_frame(recv_time)
            self._frame_condition.notify()

    def _store_rear_frame(self, frame, stamp):
        recv_time = rospy.Time.now().to_sec()
        with self._frame_condition:
            self._latest_rear_frame = (frame, stamp, recv_time)
            self._latest_rear_frame_seq += 1
            self._timing.record_camera_frame(recv_time)
            self._frame_condition.notify()

    def _use_rear_camera(self):
        # Two states run rear inference:
        #   STATE_BACKOUT      -- reversing down the row, every frame (the
        #                         other BACKOUT_* states are odometry-only, so
        #                         the front camera resumes there harmlessly);
        #   STATE_EXIT_CLEAR   -- the rear-steered headland leg, ALTERNATING
        #                         with the front camera. The rear view steers
        #                         the leg and says when the tail is clear; the
        #                         front view runs the revocation backstop,
        #                         which is the only thing that catches a false
        #                         exit and cannot be rebuilt on the rear view
        #                         (just after a genuine exit the rear near row
        #                         still legitimately has corn on both sides).
        #                         Half the frame rate each, for ~10 s.
        # FSM state is only mutated by the inference thread itself, so this
        # read is race-free.
        if not self._rear_camera_enabled or self._fsm is None:
            return False
        if self._fsm.state == STATE_BACKOUT:
            return True
        return (
            self._fsm.state == STATE_EXIT_CLEAR
            and self._fsm.exit_clear_rear_steering
            and self._exit_clear_next_is_rear
        )

    def _pause_srv(self, req):
        changed = self._set_paused(bool(req.data))
        state = "paused" if req.data else "running"
        if changed:
            rospy.logwarn("vision_nav %s by operator request", state)
        return SetBoolResponse(success=True, message=state)

    def _set_paused(self, paused):
        """Returns True if this changed anything."""
        with self._pause_lock:
            if paused == self._paused:
                return False
            self._paused = paused
        if paused:
            self._publish_twist(0.0, 0.0)
        else:
            # ⚠ Resuming has to undo the pause's effect on everything that
            # accumulates against a clock or an odometry reference, or the
            # first frame back banks the whole pause:
            #   - the BLOCKED timer is in SECONDS of ROS time, so a 30 s pause
            #     would otherwise deposit the entire blocked_confirm_seconds at
            #     once and fire a back-out that nothing justified. (_MAX_DT
            #     caps a single step at 2 s, which limits this but does not
            #     remove it.)
            #   - the MPC's rate limiter is relative to the last command it
            #     issued, which is now stale by however long the pause lasted.
            if self._detector is not None:
                self._detector.reset()
            if self._fsm is not None:
                self._fsm.rear_exit_detector.reset()
            self._controller.reset()
        if self._metrics is not None:
            self._metrics.mark_event("PAUSE" if paused else "RESUME")
        self._publish_status(None)
        return True

    def _is_paused(self):
        with self._pause_lock:
            return self._paused

    def _publish_status(self, event):
        """Latched one-line status for the operator panel."""
        if self._status_pub is None:
            return
        parts = ["paused" if self._is_paused() else "running"]
        if self._fsm is not None:
            with self._odom_lock:
                odom_pose = self._odom_pose
            parts.append("state=%s" % self._fsm.state)
            parts.append("rows=%d/%d" % (self._fsm.rows_driven, self._fsm.num_rows))
            d = self._fsm._distance_in_row(odom_pose)
            parts.append("in_row=%s m" % ("?" if d is None else "%.2f" % d))
        else:
            parts.append("state=FOLLOW_ROW (no mission)")
        try:
            self._status_pub.publish(String(data=" | ".join(parts)))
        except rospy.ROSException:
            pass

    def _watchdog_cb(self, event):
        if self._is_paused():
            # The keep-alive below would otherwise re-publish the last
            # non-zero command at 10 Hz and drive straight through the pause.
            self._publish_twist(0.0, 0.0)
            return
        with self._last_success_lock:
            last_success_time = self._last_success_time
            linear_x, angular_z = self._last_cmd
        if last_success_time is None:
            return
        age = (rospy.Time.now() - last_success_time).to_sec()
        if age > self._max_data_age_sec:
            self._publish_twist(0.0, 0.0)
            # Edge-triggered: this fires at 10 Hz for as long as the stall
            # lasts, and one stall must leave one mark in the CSV rather than
            # a flood that buries every other event.
            if not self._watchdog_stale:
                self._watchdog_stale = True
                if self._metrics is not None:
                    self._metrics.mark_event("WATCHDOG_ZERO")
        else:
            self._watchdog_stale = False
            # Keep-alive: the base controller brakes if cmd_vel goes silent,
            # so hold the last command until the next inference lands.
            self._publish_twist(linear_x, angular_z)

    def _inference_loop(self):
        last_front_seq = -1
        last_rear_seq = -1
        while not rospy.is_shutdown():
            with self._frame_condition:
                slot = None
                is_rear = False
                # ⚠ Waiting for the rear camera FOREVER is a hang, not a
                # degraded mode. The rear camera fails silently (unplugged, or
                # a topic-name mismatch), and both states that ask for it are
                # states the robot must still get out of. update() is never
                # called while this loop waits, so the FSM's own
                # "no rear frames arrived" fallback -- the thing that stops a
                # dead rear camera stranding the robot at a row end -- could
                # never run either. After this long, take a front frame and
                # let the FSM see the leg with rear_centerline_result=None.
                deadline = time.monotonic() + _REAR_FRAME_WAIT_SEC
                while not rospy.is_shutdown():
                    # Pick the source per wakeup so a state change mid-wait
                    # (e.g. entering/leaving BACKOUT) switches cameras on the
                    # very next frame.
                    is_rear = self._use_rear_camera()
                    have_front = (
                        self._latest_frame is not None
                        and self._latest_frame_seq != last_front_seq
                    )
                    if is_rear:
                        if (
                            self._latest_rear_frame is not None
                            and self._latest_rear_frame_seq != last_rear_seq
                        ):
                            slot = self._latest_rear_frame
                            last_rear_seq = self._latest_rear_frame_seq
                            break
                        if have_front and time.monotonic() >= deadline:
                            is_rear = False
                            slot = self._latest_frame
                            last_front_seq = self._latest_frame_seq
                            break
                    elif have_front:
                        slot = self._latest_frame
                        last_front_seq = self._latest_frame_seq
                        break
                    self._frame_condition.wait(timeout=0.2)
                if rospy.is_shutdown():
                    return

            # Alternate cameras for the next EXIT_CLEAR frame. Done here, once
            # per dispatched frame, because the loop above re-asks
            # _use_rear_camera() on every wakeup.
            self._exit_clear_next_is_rear = not self._exit_clear_next_is_rear
            frame, stamp, recv_time = slot
            self._process_frame(frame, stamp, recv_time, is_rear)

    def _process_frame(self, frame, stamp, recv_time, is_rear=False):
        pickup_time = rospy.Time.now().to_sec()
        t_inf_start = time.monotonic()
        try:
            mask = self._model.predict(frame)
        except Exception as exc:
            rospy.logerr_throttle(5.0, "Model inference failed: %s", exc)
            if self._metrics is not None:
                self._metrics.mark_event("INFERENCE_FAILED")
            return
        inference_s = time.monotonic() - t_inf_start

        result = estimate_centerline(
            mask,
            scan_row_fractions=self._scan_row_fractions,
            scan_row_weights=self._scan_row_weights,
            min_traversable_fraction=self._min_traversable_fraction,
            flank_margin_frac=self._flank_margin_frac,
        )
        # The exit detector may look at its own scan rows: steering wants
        # lookahead spread, exit detection wants maximum in-row vs open-field
        # separation, and the best rows for one are not the best for the other
        # (they also shift with camera mount height). Empty
        # ~exit_scan_row_fractions keeps both on the same rows, as before.
        exit_result = result
        if self._exit_scan_row_fractions:
            exit_result = estimate_centerline(
                mask,
                scan_row_fractions=self._exit_scan_row_fractions,
                scan_row_weights=self._exit_scan_row_weights,
                min_traversable_fraction=self._min_traversable_fraction,
                flank_margin_frac=self._flank_margin_frac,
            )

        # Snapshot odometry ONCE, before the mode branch. The metrics row
        # wants it on every run -- distance travelled is the denominator of
        # the autonomy number, and that is why odom is subscribed even
        # without a mission. It used to be read inside the mission branch
        # only, so a plain row-following run (mission_enabled:=false, the
        # DEFAULT) with metrics on (also the default) raised UnboundLocalError
        # here and killed the inference thread on its first frame.
        with self._odom_lock:
            odom_pose = self._odom_pose

        state_name = None
        if self._is_paused():
            # Perception keeps running (the HUD stays live and the CSV keeps
            # a record of what the robot was looking at while stopped), but
            # the FSM is NOT advanced and nothing is commanded. Skipping
            # update() is the whole point: the mission state, the row count,
            # the turn direction and the row-entry pose all survive the pause.
            linear_x, angular_z, done = 0.0, 0.0, False
            if self._fsm is not None:
                state_name = self._fsm.state + " (PAUSED)"
        elif self._fsm is not None:
            if is_rear:
                linear_x, angular_z, state_name, done = self._fsm.update(
                    # None, NOT an invalid result: this tick carries no front
                    # perception at all. EXIT_CLEAR's revocation test reads
                    # "no corridor beside me" as evidence the exit was false,
                    # so handing it a blank result would revoke every real
                    # exit within exit_revoke_fail_distance.
                    None,
                    odom_pose,
                    mask.shape[1],
                    # Rear stays on the steering rows: the FSM uses this one
                    # result both to watch for the exit behind AND to steer the
                    # reverse leg, and the reverse steering must keep the
                    # field-proven scan rows.
                    rear_centerline_result=result,
                    now=rospy.get_time(),
                )
                state_name = state_name + " (REAR)"
                if self._fsm.state == STATE_BACKOUT:
                    # Telemetry for the reverse-steering nudge test: off-center
                    # rear offset must produce an angular_z that re-centers it.
                    reversed_m, target_m = self._fsm.backout_progress(odom_pose)
                    rospy.loginfo_throttle(
                        1.0,
                        "BACKOUT: rear offset=%+.3f slope=%+.3f -> angular_z=%+.3f, "
                        "reversed %s of %.2f m",
                        result.offset_norm,
                        result.slope_term,
                        angular_z,
                        "?" if reversed_m is None else "%.2f" % reversed_m,
                        target_m,
                    )
                elif self._fsm.state == STATE_EXIT_CLEAR:
                    # Same shape for the rear-steered headland leg. 'rear
                    # open' flipping from no to a distance is the moment the
                    # tail cleared the row; the turn follows
                    # exit_clear_post_rear_distance later.
                    travelled_m, open_at, _ = self._fsm.exit_clear_progress(
                        odom_pose
                    )
                    rospy.loginfo_throttle(
                        1.0,
                        "EXIT_CLEAR: rear offset=%+.3f slope=%+.3f -> "
                        "angular_z=%+.3f, travelled %s m, rear open %s",
                        result.offset_norm,
                        result.slope_term,
                        angular_z,
                        "?" if travelled_m is None else "%.2f" % travelled_m,
                        "no" if open_at is None else "at %.2f m" % open_at,
                    )
            else:
                linear_x, angular_z, state_name, done = self._fsm.update(
                    result,
                    odom_pose,
                    mask.shape[1],
                    now=rospy.get_time(),
                    exit_centerline_result=exit_result,
                )
            # A revoked exit is the loudest thing the detector can tell you:
            # it means the open signature confirmed and then the ground beside
            # the robot said otherwise. Never throttle it.
            while self._revoked_logged < len(self._fsm.revoked_exits):
                row, dist = self._fsm.revoked_exits[self._revoked_logged]
                self._revoked_logged += 1
                rospy.logwarn(
                    "EXIT REVOKED: row %d at %.2f m -- near scan row still "
                    "corn-flanked; back to FOLLOW_ROW",
                    row,
                    -1.0 if dist is None else dist,
                )
                if self._metrics is not None:
                    self._metrics.mark_event("EXIT_REVOKED")
            # Same watch-the-list-length pattern for blocked rows, so the CSV
            # marks the frame the back-out was committed on -- alongside the
            # obst=/trav= readings that justified it.
            while self._blocked_logged < len(self._fsm.blocked_events):
                self._blocked_logged += 1
                if self._metrics is not None:
                    self._metrics.mark_event("BLOCKED")
            if done and not self._mission_done_logged:
                self._mission_done_logged = True
                blocked = self._fsm.blocked_events
                summary = (
                    ", ".join(
                        "row %d blocked at %.2f m" % (row, dist)
                        for row, dist in blocked
                    )
                    if blocked
                    else "none"
                )
                rospy.loginfo(
                    "Mission DONE: rows_driven=%d, blocked rows: %s, "
                    "revoked exits: %d",
                    self._fsm.rows_driven,
                    summary,
                    len(self._fsm.revoked_exits),
                )
                if self._metrics is not None:
                    self._metrics.mark_event("MISSION_DONE")
        else:
            linear_x, angular_z = self._controller.compute(
                result.offset_norm, result.slope_term, result.valid
            )
        self._publish_twist(linear_x, angular_z)

        now = rospy.Time.now()
        with self._last_success_lock:
            self._last_success_time = now
            self._last_cmd = (linear_x, angular_z)

        publish_time = now.to_sec()
        self._timing.record_inference(
            inference_s,
            wait_age_s=max(0.0, pickup_time - recv_time),
            # End-to-end: camera exposure (header stamp) -> command publish.
            # This is the true control staleness the colleagues ask about.
            e2e_latency_s=(publish_time - stamp) if stamp is not None else None,
            publish_time=publish_time,
        )
        rospy.loginfo_throttle(5.0, "timing: %s", self._timing.format_summary())

        if self._metrics is not None:
            self._log_metrics_row(
                result, linear_x, angular_z, state_name, is_rear,
                stamp, publish_time, inference_s, pickup_time, recv_time,
                odom_pose,
            )

        detector_line = self._detector_line()
        if detector_line is not None:
            # Countdowns (to a back-out, or to the rear seeing the row exit)
            # are worth seeing quickly; everything else shares the 5 s
            # cadence of the timing line so a mission run always leaves a
            # state + detector trace in the console ("why didn't the exit
            # fire" must be answerable from the log alone).
            # A meter that is moving -- filling OR draining -- is the whole
            # story of why an exit did or did not fire. At 5 s throttling on a
            # 2 Hz robot only 1 frame in 10 is printed, which is how a
            # partially-firing exit signature (open 0.13/0.40 m) was misread as
            # never firing at all (sim, 2026-07-28).
            s = self._detector.last_status
            counting_down = is_rear or (
                s is not None
                and (s.blocked_seconds > 0.0 or s.open_distance > 0.0)
            )
            rospy.loginfo_throttle(
                1.0 if counting_down else 5.0, "%s", detector_line
            )

        if self._debug_pub is not None:
            self._publish_debug(
                frame, mask, result, linear_x, angular_z, state_name,
                self._timing.hud_line(), detector_line, is_rear=is_rear,
            )

    def _log_metrics_row(self, result, linear_x, angular_z, state_name,
                         is_rear, stamp, publish_time, inference_s,
                         pickup_time, recv_time, odom_pose):
        """One CSV row for this frame.

        Everything here is already a local in _process_frame -- this method
        exists only to keep the assembly out of the control path's line of
        sight. It must never raise; RunMetricsLogger swallows its own errors,
        and the getattr() defaults below cover a hand-built or rear-path
        result that lacks the optional fields.
        """
        scan_rows = result.scan_rows or ()
        row = {
            "t_ros": publish_time,
            "t_wall": time.time(),
            "frame_stamp": stamp,
            "valid": result.valid,
            "offset_norm": result.offset_norm,
            "slope_term": result.slope_term,
            "traversable_fraction": result.traversable_fraction,
            "obstacle_fraction": getattr(result, "obstacle_fraction", None),
            "scan_offsets": join_list([r.offset_norm for r in scan_rows]),
            "scan_x_left": join_list([r.x_left for r in scan_rows]),
            "scan_x_right": join_list([r.x_right for r in scan_rows]),
            "linear_x": linear_x,
            "angular_z": angular_z,
            "camera": "rear" if is_rear else "front",
            # Marks the frames a human was driving, so the report can subtract
            # them from the autonomous distance rather than crediting the
            # controller with the rescue.
            "teleop": (
                self._interventions.active(publish_time)
                if self._interventions is not None
                else None
            ),
            "state": state_name or "",
            "inference_s": inference_s,
            "e2e_latency_s": (publish_time - stamp) if stamp is not None else None,
            "wait_age_s": max(0.0, pickup_time - recv_time),
            "frames_received": self._timing.frames_received,
            "frames_processed": self._timing.frames_processed,
        }
        if odom_pose is not None:
            row["odom_x"], row["odom_y"], row["odom_yaw"] = odom_pose
        if self._fsm is not None:
            row["rows_driven"] = self._fsm.rows_driven
        # Detector status is what tells you WHY a run behaved as it did, so it
        # rides on every row rather than only on the ones where it changed.
        status = self._detector.last_status if self._detector is not None else None
        if status is not None:
            row.update({
                "distance_in_row": status.distance_in_row,
                "near_row_width": status.near_row_width,
                "open_distance": status.open_distance,
                "blocked_seconds": status.blocked_seconds,
                "corridor_rows": status.corridor_rows,
                "wide_rows": status.wide_rows,
                "open_rows": status.open_rows,
                "open_armed": status.open_armed,
                "blocked_armed": status.blocked_armed,
            })
            if status.near_row_edges is not None:
                row["near_row_edge_l"] = status.near_row_edges[0]
                row["near_row_edge_r"] = status.near_row_edges[1]
        self._metrics.log(**row)

    def _on_shutdown(self):
        """Close the CSV and log the run's headline numbers.

        Read back from the file rather than kept in memory: it both proves the
        file is intact and means the console log of a run always carries its
        own summary, even when nobody opens the CSV.
        """
        if self._metrics is None:
            return
        path = self._metrics.path
        rows_written = self._metrics.rows_written
        self._metrics.close()
        for message in self._metrics.errors:
            rospy.logwarn("metrics CSV: %s", message)
        if rows_written == 0:
            return
        try:
            summary = format_summary(summarize(read_csv(path)))
        except Exception as exc:                      # noqa: BLE001
            rospy.logwarn("could not summarize %s: %s", path, exc)
            return
        rospy.loginfo("---- run summary (%d frames) ----", rows_written)
        for line in summary.splitlines():
            rospy.loginfo("%s", line)
        rospy.loginfo("full per-frame data: %s", path)
        rospy.loginfo("---------------------------------")

    def _detector_line(self):
        """HUD/log line showing the mission state and, where one applies,
        why the exit detector is (not) firing.

        FOLLOW_ROW shows the front detector; BACKOUT and the rear-steered
        EXIT_CLEAR show the rear-view open-exit watcher that ends the leg;
        every other state shows the state alone, so the trace never goes
        silent (a silent log cannot distinguish "blocker never seen" from
        "mission already moved on"). None only outside mission mode.
        """
        if self._detector is None:
            return None
        prefix = "state=%s " % self._fsm.state
        if self._fsm.state == STATE_BACKOUT or (
            self._fsm.state == STATE_EXIT_CLEAR
            and self._fsm.exit_clear_rear_steering
        ):
            rear = self._fsm.rear_exit_detector
            if rear.last_status is None:
                return prefix.rstrip()
            s = rear.last_status
            # openrows / near w= / edges= for the same reason the front line
            # carries them: wide= alone cannot distinguish "corridor too
            # narrow" from "flanks not clear", which is exactly what made the
            # 2026-08-05 back-out log unreadable.
            return prefix + (
                "rear exit: open %.2f/%.2f m wide=%d openrows=%d "
                "near w=%s edges=%s"
            ) % (
                s.open_distance,
                rear.exit_confirm_distance,
                s.wide_rows,
                s.open_rows,
                "-" if s.near_row_width is None else "%.2f" % s.near_row_width,
                "-"
                if s.near_row_edges is None
                else "%.2f/%.2f" % s.near_row_edges,
            )
        if self._detector.last_status is None or self._fsm.state != STATE_FOLLOW_ROW:
            return prefix.rstrip()
        s = self._detector.last_status
        # near w= / edges= say WHICH threshold is holding an exit back. Without
        # them, "width just under exit_width_threshold" and "edges just under
        # exit_flank_min_clear_fraction" both render as openrows=0 and there is
        # no way to tell which knob to turn.
        # trav=/obst= are the same story for BLOCKED: obst= is what the gate
        # tests, and the PAIR is what makes an approach readable -- trav falling
        # to 0.00 while obst climbs is a healthy blocker; both at 0.00 is a
        # garbage mask and the gate is right to hold.
        return prefix + (
            "exit: blk %.1f/%.1f s open %.2f/%.2f m rows=%d wide=%d openrows=%d "
            "near w=%s edges=%s trav=%.2f obst=%s armed o:%s b:%s"
        ) % (
            s.blocked_seconds,
            self._detector.blocked_confirm_seconds,
            s.open_distance,
            self._detector.exit_confirm_distance,
            s.corridor_rows,
            s.wide_rows,
            s.open_rows,
            "-" if s.near_row_width is None else "%.2f" % s.near_row_width,
            "-"
            if s.near_row_edges is None
            else "%.2f/%.2f" % s.near_row_edges,
            s.traversable_fraction,
            "-" if s.obstacle_fraction is None else "%.2f" % s.obstacle_fraction,
            "Y" if s.open_armed else "N",
            "Y" if s.blocked_armed else "N",
        )

    def _log_config(self):
        """One-shot startup dump of the tuning actually in effect.

        Read back from the node's own private namespace rather than from the
        constructor arguments, so this reports what the node RECEIVED -- the
        merge of config/params.yaml, the launch-file defaults, and any
        command-line override -- instead of restating what we think we passed.

        Worth the console space for two reasons. A field log read back a week
        later otherwise has no record of how the robot was tuned for that run.
        And a value that silently did not take effect becomes visible here at
        launch, which is the failure mode this repo keeps hitting: launch-arg
        <param> tags override config/params.yaml for every duplicated knob, so
        editing the yaml alone does nothing (2026-07-30: headland_clearance
        edited to 1.5 in the yaml, robot ran at the launch default 0.75).
        """
        try:
            p = rospy.get_param("~")
        except Exception:            # noqa: BLE001 - diagnostics must never
            rospy.logwarn(           # take the node down
                "could not read the private parameter namespace; "
                "skipping the config dump"
            )
            return

        def g(key, default="?"):
            return p.get(key, default)

        rospy.loginfo("---- vision_nav config in effect ----")
        rospy.loginfo(
            "speed:    cruise=%s ang_max=%s d_ang_max=%s | mpc dt=%s N=%s "
            "| invalid_stop=%s data_age=%s s",
            g("linear_x_cruise"), g("angular_z_max"), g("delta_angular_z_max"),
            g("mpc_dt"), g("mpc_horizon"),
            g("invalid_frame_stop_count"), g("max_data_age_sec"),
        )
        exit_rows = self._exit_scan_row_fractions or "shared with steering"
        rospy.loginfo(
            "scanrows: steering=%s | exit=%s",
            self._scan_row_fractions, exit_rows,
        )
        rospy.loginfo(
            "metrics:  %s",
            self._metrics.path if self._metrics is not None else "DISABLED",
        )

        if not self._mission_enabled:
            rospy.loginfo(
                "mission:  DISABLED (plain row-following; no exit detection, "
                "no headland turns)"
            )
            rospy.loginfo("-------------------------------------")
            return

        rospy.loginfo(
            "mission:  rows=%s first_turn=%s | rear_camera=%s backout=%s",
            g("num_rows"), g("first_turn_direction"),
            self._rear_camera_enabled, self._fsm.backout_enabled,
        )
        rospy.loginfo(
            "exit:     width>=%s flank=%s/%s | confirm=%s m leak=%s "
            "min_frames=%s rows_req=%s | arms after %s m",
            g("exit_width_threshold"), g("exit_flank_edge_margin"),
            g("exit_flank_min_clear_fraction"), g("exit_confirm_distance"),
            g("exit_leak_ratio"), g("exit_detect_min_frames"),
            g("exit_open_rows_required"), g("min_in_row_distance"),
        )
        rospy.loginfo(
            "blocked:  confirm=%s s leak=%s obstacle>=%s | arms after %s m",
            g("blocked_confirm_seconds"), g("blocked_leak_ratio"),
            g("blocked_min_obstacle_fraction"), g("blocked_arming_distance"),
        )
        rospy.loginfo(
            "headland: clearance=%s m (min %s) speed=%s | traverse=%s "
            "row_spacing=%s | turn=%s rad/s tol=%s deg",
            g("headland_clearance"), g("exit_clear_min_distance"),
            g("exit_clear_speed"), g("traverse_distance"), g("row_spacing"),
            g("turn_rate"), g("yaw_tolerance_deg"),
        )
        # Report the RESOLVED value, not the parameter: it is ANDed with
        # rear_camera_enabled, so the yaml can say true while the leg still
        # runs open loop. Which terminator is live changes what
        # headland_clearance above even means, so it is spelled out.
        if self._fsm.exit_clear_rear_steering:
            rospy.loginfo(
                "  exit leg: REAR-STEERED (gain=%s, |w|<=%s rad/s, <=%s deg "
                "total) -- turns on the rear open-exit signature +%s m, NOT on "
                "headland_clearance; turns anyway after %s m; cameras "
                "ALTERNATE so revocation still runs",
                g("exit_clear_rear_offset_gain"),
                g("exit_clear_angular_z_max"), g("exit_clear_max_yaw_deg"),
                g("exit_clear_post_rear_distance"), g("exit_clear_max_distance"),
            )
        else:
            rospy.loginfo(
                "  exit leg: OPEN LOOP (%s) -- turns on headland_clearance, "
                "revocation is the false-exit backstop",
                "no rear camera" if not self._rear_camera_enabled
                else "exit_clear_rear_steering is false",
            )
        rospy.loginfo(
            "recovery: reacquire speed=%s confirm=%s m max=%s m steering=%s "
            "| revoke=%s within %s m after %s m of corn",
            g("reacquire_speed"), g("reacquire_confirm_distance"),
            g("reacquire_max_distance"), g("reacquire_steering_enabled"),
            g("exit_revoke_enabled"), g("exit_revoke_distance"),
            g("exit_revoke_fail_distance"),
        )
        rospy.loginfo("-------------------------------------")

    def _publish_twist(self, linear_x, angular_z):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self._cmd_vel_pub.publish(twist)

    def _publish_debug(self, frame, mask, result, linear_x, angular_z,
                       state_name=None, timing_line=None, detector_line=None,
                       is_rear=False):
        # ⚠ One topic per CAMERA, never one topic carrying both. The rear-
        # terminated EXIT_CLEAR alternates cameras frame by frame, and a single
        # topic then flips between a forward and a backward view several times
        # a second -- unreadable at exactly the moment the operator needs to
        # judge whether the robot is leaving the row straight (reported from
        # the first sim run, 2026-08-07). Open both in rqt_image_view to watch
        # the leg; the front topic simply goes quiet during BACKOUT, which is
        # the one state that is rear-only.
        publisher = self._debug_rear_pub if is_rear else self._debug_pub
        if publisher is None:
            return
        try:
            debug_img = render_debug_image(
                frame, mask, result, linear_x, angular_z, state_name=state_name,
                timing_line=timing_line, detector_line=detector_line,
            )
            msg = self._bridge.cv2_to_imgmsg(debug_img, encoding="bgr8")
            publisher.publish(msg)
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "Failed to publish debug image: %s", exc)

    def spin(self):
        rospy.spin()


def main():
    node = VisionNavNode()
    node.spin()


if __name__ == "__main__":
    main()
