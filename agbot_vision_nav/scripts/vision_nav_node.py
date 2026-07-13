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

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage, Image

from agbot_vision_nav.centerline_estimator import CenterlineResult, estimate_centerline
from agbot_vision_nav.controller import MPCRowController
from agbot_vision_nav.debug_viz import render_debug_image
from agbot_vision_nav.mission_fsm import STATE_BACKOUT, MissionFSM
from agbot_vision_nav.row_exit_detector import RowExitDetector
from agbot_vision_nav.segmentation_model import SegmentationModel

# Placeholder front result passed to the FSM on rear-camera ticks (the FSM
# ignores front perception in STATE_BACKOUT; this only keeps the signature).
_INVALID_RESULT = CenterlineResult(
    offset_norm=0.0,
    slope_term=0.0,
    valid=False,
    traversable_fraction=0.0,
    scan_rows=(),
)


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
        if self._mission_enabled:
            detector = RowExitDetector(
                exit_width_threshold=rospy.get_param("~exit_width_threshold", 0.8),
                exit_detect_frames=rospy.get_param("~exit_detect_frames", 5),
                min_in_row_distance=rospy.get_param("~min_in_row_distance", 2.0),
                blocked_min_traversable_fraction=rospy.get_param(
                    "~blocked_min_traversable_fraction", 0.15
                ),
                blocked_arming_distance=rospy.get_param(
                    "~blocked_arming_distance", 0.3
                ),
                exit_open_rows_required=rospy.get_param(
                    "~exit_open_rows_required", 2
                ),
                blocked_detect_frames=rospy.get_param("~blocked_detect_frames", 8),
            )
            self._fsm = MissionFSM(
                self._controller,
                detector,
                num_rows=rospy.get_param("~num_rows", 3),
                first_turn_direction=rospy.get_param("~first_turn_direction", "left"),
                row_spacing=rospy.get_param("~row_spacing", 0.75),
                headland_clearance=rospy.get_param("~headland_clearance", 1.0),
                turn_rate=rospy.get_param("~turn_rate", 0.4),
                yaw_tolerance_deg=rospy.get_param("~yaw_tolerance_deg", 5.0),
                reacquire_speed=rospy.get_param("~reacquire_speed", 0.08),
                reacquire_max_width=rospy.get_param("~reacquire_max_width", 0.6),
                reacquire_frames=rospy.get_param("~reacquire_frames", 3),
                reacquire_max_distance=rospy.get_param("~reacquire_max_distance", 1.5),
                backout_speed=rospy.get_param("~backout_speed", 0.10),
                backout_enabled=self._rear_camera_enabled,
            )

        rospy.loginfo("Loading segmentation model from %s ...", model_path)
        self._model = SegmentationModel(model_path)
        rospy.loginfo("Model loaded.")

        self._bridge = CvBridge()

        self._frame_condition = threading.Condition()
        self._latest_frame = None
        self._latest_frame_seq = 0
        self._latest_rear_frame = None
        self._latest_rear_frame_seq = 0
        self._mission_done_logged = False

        self._last_success_lock = threading.Lock()
        self._last_success_time = None
        self._last_cmd = (0.0, 0.0)

        self._odom_lock = threading.Lock()
        self._odom_pose = None  # (x, y, yaw)
        if self._mission_enabled:
            odom_topic = rospy.get_param("~odom_topic", "/odometry/filtered")
            rospy.Subscriber(odom_topic, Odometry, self._odom_cb, queue_size=1)

        self._cmd_vel_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)
        self._debug_pub = None
        if self._publish_debug_image:
            self._debug_pub = rospy.Publisher(debug_image_topic, Image, queue_size=1)

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

        rospy.loginfo("vision_nav_node ready, listening on %s", camera_topic)

    def _compressed_image_cb(self, msg):
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            rospy.logwarn_throttle(5.0, "Failed to decode CompressedImage frame")
            return
        self._store_frame(frame)

    def _image_cb(self, msg):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "cv_bridge conversion failed: %s", exc)
            return
        self._store_frame(frame)

    def _rear_compressed_image_cb(self, msg):
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            rospy.logwarn_throttle(5.0, "Failed to decode rear CompressedImage frame")
            return
        self._store_rear_frame(frame)

    def _rear_image_cb(self, msg):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "rear cv_bridge conversion failed: %s", exc)
            return
        self._store_rear_frame(frame)

    def _odom_cb(self, msg):
        pose = msg.pose.pose
        yaw = _quaternion_to_yaw(pose.orientation)
        with self._odom_lock:
            self._odom_pose = (pose.position.x, pose.position.y, yaw)

    def _store_frame(self, frame):
        with self._frame_condition:
            self._latest_frame = frame
            self._latest_frame_seq += 1
            self._frame_condition.notify()

    def _store_rear_frame(self, frame):
        with self._frame_condition:
            self._latest_rear_frame = frame
            self._latest_rear_frame_seq += 1
            self._frame_condition.notify()

    def _use_rear_camera(self):
        # Rear inference only while actively reversing down the row; the
        # other BACKOUT_* states are odometry-only, so the front camera
        # resumes there harmlessly. FSM state is only mutated by the
        # inference thread itself, so this read is race-free.
        return (
            self._rear_camera_enabled
            and self._fsm is not None
            and self._fsm.state == STATE_BACKOUT
        )

    def _watchdog_cb(self, event):
        with self._last_success_lock:
            last_success_time = self._last_success_time
            linear_x, angular_z = self._last_cmd
        if last_success_time is None:
            return
        age = (rospy.Time.now() - last_success_time).to_sec()
        if age > self._max_data_age_sec:
            self._publish_twist(0.0, 0.0)
        else:
            # Keep-alive: the base controller brakes if cmd_vel goes silent,
            # so hold the last command until the next inference lands.
            self._publish_twist(linear_x, angular_z)

    def _inference_loop(self):
        last_front_seq = -1
        last_rear_seq = -1
        while not rospy.is_shutdown():
            with self._frame_condition:
                frame = None
                is_rear = False
                while not rospy.is_shutdown():
                    # Pick the source per wakeup so a state change mid-wait
                    # (e.g. entering/leaving BACKOUT) switches cameras on the
                    # very next frame.
                    is_rear = self._use_rear_camera()
                    if is_rear:
                        if (
                            self._latest_rear_frame is not None
                            and self._latest_rear_frame_seq != last_rear_seq
                        ):
                            frame = self._latest_rear_frame
                            last_rear_seq = self._latest_rear_frame_seq
                            break
                    elif (
                        self._latest_frame is not None
                        and self._latest_frame_seq != last_front_seq
                    ):
                        frame = self._latest_frame
                        last_front_seq = self._latest_frame_seq
                        break
                    self._frame_condition.wait(timeout=0.2)
                if rospy.is_shutdown():
                    return

            self._process_frame(frame, is_rear)

    def _process_frame(self, frame, is_rear=False):
        try:
            mask = self._model.predict(frame)
        except Exception as exc:
            rospy.logerr_throttle(5.0, "Model inference failed: %s", exc)
            return

        result = estimate_centerline(
            mask,
            scan_row_fractions=self._scan_row_fractions,
            scan_row_weights=self._scan_row_weights,
            min_traversable_fraction=self._min_traversable_fraction,
        )

        state_name = None
        if self._fsm is not None:
            with self._odom_lock:
                odom_pose = self._odom_pose
            if is_rear:
                linear_x, angular_z, state_name, done = self._fsm.update(
                    _INVALID_RESULT,
                    odom_pose,
                    mask.shape[1],
                    rear_centerline_result=result,
                )
                state_name = state_name + " (REAR)"
            else:
                linear_x, angular_z, state_name, done = self._fsm.update(
                    result, odom_pose, mask.shape[1]
                )
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
                    "Mission DONE: rows_driven=%d, blocked rows: %s",
                    self._fsm.rows_driven,
                    summary,
                )
        else:
            linear_x, angular_z = self._controller.compute(
                result.offset_norm, result.slope_term, result.valid
            )
        self._publish_twist(linear_x, angular_z)

        with self._last_success_lock:
            self._last_success_time = rospy.Time.now()
            self._last_cmd = (linear_x, angular_z)

        if self._debug_pub is not None:
            self._publish_debug(frame, mask, result, linear_x, angular_z, state_name)

    def _publish_twist(self, linear_x, angular_z):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self._cmd_vel_pub.publish(twist)

    def _publish_debug(self, frame, mask, result, linear_x, angular_z, state_name=None):
        try:
            debug_img = render_debug_image(
                frame, mask, result, linear_x, angular_z, state_name=state_name
            )
            msg = self._bridge.cv2_to_imgmsg(debug_img, encoding="bgr8")
            self._debug_pub.publish(msg)
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "Failed to publish debug image: %s", exc)

    def spin(self):
        rospy.spin()


def main():
    node = VisionNavNode()
    node.spin()


if __name__ == "__main__":
    main()
