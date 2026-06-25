#!/usr/bin/env python3
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
back up. A 5 Hz watchdog publishes zero Twist if no successful inference has
completed within max_data_age_sec.
"""

import threading

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage, Image

from agbot_vision_nav.centerline_estimator import estimate_centerline
from agbot_vision_nav.controller import RowCenteringController
from agbot_vision_nav.debug_viz import render_debug_image
from agbot_vision_nav.segmentation_model import SegmentationModel


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

        self._controller = RowCenteringController(
            k_p=rospy.get_param("~k_p", 1.0),
            k_slope=rospy.get_param("~k_slope", 0.0),
            linear_x_cruise=rospy.get_param("~linear_x_cruise", 0.15),
            angular_z_max=rospy.get_param("~angular_z_max", 0.3),
            invalid_frame_stop_count=rospy.get_param("~invalid_frame_stop_count", 5),
        )

        rospy.loginfo("Loading segmentation model from %s ...", model_path)
        self._model = SegmentationModel(model_path)
        rospy.loginfo("Model loaded.")

        self._bridge = CvBridge()

        self._frame_condition = threading.Condition()
        self._latest_frame = None
        self._latest_frame_seq = 0

        self._last_success_lock = threading.Lock()
        self._last_success_time = None

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

        self._inference_thread = threading.Thread(target=self._inference_loop)
        self._inference_thread.daemon = True
        self._inference_thread.start()

        rospy.Timer(rospy.Duration(1.0 / 5.0), self._watchdog_cb)

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

    def _store_frame(self, frame):
        with self._frame_condition:
            self._latest_frame = frame
            self._latest_frame_seq += 1
            self._frame_condition.notify()

    def _watchdog_cb(self, event):
        with self._last_success_lock:
            last_success_time = self._last_success_time
        if last_success_time is None:
            return
        age = (rospy.Time.now() - last_success_time).to_sec()
        if age > self._max_data_age_sec:
            self._publish_twist(0.0, 0.0)

    def _inference_loop(self):
        last_processed_seq = -1
        while not rospy.is_shutdown():
            with self._frame_condition:
                while (
                    self._latest_frame is None
                    or self._latest_frame_seq == last_processed_seq
                ) and not rospy.is_shutdown():
                    self._frame_condition.wait(timeout=0.2)
                if rospy.is_shutdown():
                    return
                frame = self._latest_frame
                last_processed_seq = self._latest_frame_seq

            self._process_frame(frame)

    def _process_frame(self, frame):
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

        linear_x, angular_z = self._controller.compute(
            result.offset_norm, result.slope_term, result.valid
        )
        self._publish_twist(linear_x, angular_z)

        with self._last_success_lock:
            self._last_success_time = rospy.Time.now()

        if self._debug_pub is not None:
            self._publish_debug(frame, mask, result, linear_x, angular_z)

    def _publish_twist(self, linear_x, angular_z):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self._cmd_vel_pub.publish(twist)

    def _publish_debug(self, frame, mask, result, linear_x, angular_z):
        try:
            debug_img = render_debug_image(frame, mask, result, linear_x, angular_z)
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
