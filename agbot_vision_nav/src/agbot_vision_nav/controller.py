"""Lateral-offset proportional controller: (offset, slope) -> (linear_x, angular_z).

Pure python, no rospy dependency, so it can be unit-tested directly and
constructs plain floats rather than geometry_msgs/Twist -- the ROS node is
responsible for wrapping the returned values into a Twist message.
"""

import numpy as np


class RowCenteringController:
    """Proportional row-centering control law with an invalid-frame safety net.

    angular_z = -(k_p * offset_norm + k_slope * slope_term), clamped.
    linear_x = linear_x_cruise (constant) while tracking is valid.

    Sign convention: image x increases rightward. If the traversable
    centerline is left of image center, offset_norm < 0, and the robot must
    turn left, which is positive angular.z under ROS/REP-103 convention --
    hence the leading minus sign. Locked in by test_controller.py.

    On an invalid (or stale) frame, the last commanded velocity is held for
    up to invalid_frame_stop_count - 1 consecutive invalid frames (tolerating
    brief single-frame noise/flicker), after which zero velocity is
    published instead of continuing to extrapolate blindly.
    """

    def __init__(
        self,
        k_p=1.0,
        k_slope=0.0,
        linear_x_cruise=0.15,
        angular_z_max=0.3,
        invalid_frame_stop_count=5,
    ):
        self.k_p = k_p
        self.k_slope = k_slope
        self.linear_x_cruise = linear_x_cruise
        self.angular_z_max = angular_z_max
        self.invalid_frame_stop_count = invalid_frame_stop_count

        self._consecutive_invalid = 0
        self._last_command = (0.0, 0.0)

    def reset(self):
        """Clear held state, e.g. on node startup or after a manual stop."""
        self._consecutive_invalid = 0
        self._last_command = (0.0, 0.0)

    @property
    def consecutive_invalid(self):
        return self._consecutive_invalid

    def compute(self, offset_norm, slope_term, valid):
        """Returns (linear_x, angular_z) as plain floats."""
        if valid:
            self._consecutive_invalid = 0
            angular_z = -(self.k_p * offset_norm + self.k_slope * slope_term)
            angular_z = float(np.clip(angular_z, -self.angular_z_max, self.angular_z_max))
            linear_x = float(self.linear_x_cruise)
            self._last_command = (linear_x, angular_z)
            return self._last_command

        self._consecutive_invalid += 1
        if self._consecutive_invalid >= self.invalid_frame_stop_count:
            self._last_command = (0.0, 0.0)
            return self._last_command

        return self._last_command
