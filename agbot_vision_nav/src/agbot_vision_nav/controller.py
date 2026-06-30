"""Model Predictive Controller for camera-based row centering.

State: x = [offset_norm, slope_term]
  - offset_norm: normalized lateral error in image space [-1, 1].
    Positive = robot LEFT of corridor center (corridor appears right in image).
    Negative = robot RIGHT of corridor center (corridor appears left in image).
  - slope_term: geometric heading proxy = far_row_midpoint - near_row_midpoint,
    computed for free by centerline_estimator from the same scan used for lateral
    estimation. Positive = corridor tilts rightward in image = robot heading LEFT.

Control: u = angular_z (rad/s, positive = left turn, REP-103 convention).

Dynamics (linear, time-invariant, in normalized image-coordinate space):
  x[k+1] = A x[k] + B u[k]
  A = [[1, alpha],   B = [[0   ],
       [0, 1    ]]        [beta ]]

  alpha: lateral coupling -- how much heading error (slope_term) drives lateral
    drift (offset_norm) per control step. Positive: slope_term > 0 (heading left)
    causes offset_norm to increase (robot drifts left, corridor shifts right).
  beta:  control effectiveness -- how much angular_z changes slope_term per step.
    Positive: turning left (angular_z > 0) increases slope_term (heading becomes
    more leftward). Both alpha and beta are tuned empirically.

Cost over horizon N:
  J = sum_{k=1}^{N} [q_offset * e_d^2 + q_heading * slope^2]
    + sum_{k=0}^{N-1} [r_control * u^2 + r_delta * (u_k - u_{k-1})^2]

  r_delta provides temporal smoothing without needing an EKF or IMU.

Constraints: |u| <= angular_z_max, |delta_u| <= delta_angular_z_max.

How this differs from CropFollow/P-AgNav/Agronav:
  - No regression head needed: state is derived by pure geometry from a
    semantic segmentation mask (scan-row boundary midpoints), not from a
    separately trained heading/distance regression network.
  - No EKF: smoothing comes from r_delta and the receding-horizon structure,
    not from Bayesian sensor fusion with IMU.
  - MPC formulated in normalized image-space, not in metric/physical units,
    making it camera-resolution-agnostic and intrinsic-free.
  - slope_term heading proxy is a free by-product of the same scan used for
    lateral estimation -- no extra model, no extra computation.

Pure Python, no rospy dependency. Returns plain floats; the ROS node wraps
them into geometry_msgs/Twist.
"""

import numpy as np
from scipy.optimize import minimize


class MPCRowController:
    """Receding-horizon MPC for image-space row centering."""

    def __init__(
        self,
        N=8,
        dt=0.1,
        alpha=0.10,
        beta=0.10,
        q_offset=10.0,
        q_heading=1.0,
        r_control=0.1,
        r_delta=0.5,
        linear_x_cruise=0.15,
        angular_z_max=0.3,
        delta_angular_z_max=0.2,
        invalid_frame_stop_count=5,
    ):
        self.N = N
        self.linear_x_cruise = linear_x_cruise
        self.angular_z_max = angular_z_max
        self.delta_angular_z_max = delta_angular_z_max
        self.invalid_frame_stop_count = invalid_frame_stop_count

        self._A = np.array([[1.0, alpha], [0.0, 1.0]])
        self._B = np.array([0.0, beta])
        self._Q = np.diag([q_offset, q_heading])
        self._r = r_control
        self._r_delta = r_delta

        self._u_prev = 0.0
        self._consecutive_invalid = 0
        self._last_command = (0.0, 0.0)

    def reset(self):
        """Clear all held state (call on node startup or after a manual stop)."""
        self._u_prev = 0.0
        self._consecutive_invalid = 0
        self._last_command = (0.0, 0.0)

    @property
    def consecutive_invalid(self):
        return self._consecutive_invalid

    def _objective(self, u_seq, x0):
        x = x0.copy()
        J = 0.0
        u_prev = self._u_prev
        for u in u_seq:
            x = self._A @ x + self._B * u
            J += x @ self._Q @ x + self._r * u ** 2 + self._r_delta * (u - u_prev) ** 2
            u_prev = u
        return J

    def _solve(self, x0):
        u0 = np.zeros(self.N)
        bounds = [(-self.angular_z_max, self.angular_z_max)] * self.N

        # Rate constraint: each consecutive pair within the sequence.
        # The first step is also constrained against u_prev (the last applied command).
        constraints = []
        u_max, du_max = self.angular_z_max, self.delta_angular_z_max
        u_prev_ref = self._u_prev

        def make_lower(k, u_prev_ref=u_prev_ref):
            def c(u):
                u_k = u[k]
                u_km1 = u_prev_ref if k == 0 else u[k - 1]
                return du_max - (u_k - u_km1)
            return c

        def make_upper(k, u_prev_ref=u_prev_ref):
            def c(u):
                u_k = u[k]
                u_km1 = u_prev_ref if k == 0 else u[k - 1]
                return du_max + (u_k - u_km1)
            return c

        for k in range(self.N):
            constraints.append({"type": "ineq", "fun": make_lower(k)})
            constraints.append({"type": "ineq", "fun": make_upper(k)})

        result = minimize(
            self._objective,
            u0,
            args=(x0,),
            method="SLSQP",
            bounds=bounds,
            constraints=constraints,
            options={"ftol": 1e-6, "maxiter": 100, "disp": False},
        )
        return float(result.x[0])

    def compute(self, offset_norm, slope_term, valid):
        """Returns (linear_x, angular_z) as plain floats.

        Sign convention preserved from the P-controller:
          offset_norm < 0  (centerline left of image)  -> angular_z > 0  (left turn)
          offset_norm > 0  (centerline right of image) -> angular_z < 0  (right turn)
          slope_term > 0   (heading left)              -> angular_z < 0  (corrective right)
        """
        if valid:
            self._consecutive_invalid = 0
            x0 = np.array([float(offset_norm), float(slope_term)])
            u_opt = self._solve(x0)
            u_opt = float(np.clip(u_opt, -self.angular_z_max, self.angular_z_max))
            self._u_prev = u_opt
            self._last_command = (float(self.linear_x_cruise), u_opt)
            return self._last_command

        self._consecutive_invalid += 1
        if self._consecutive_invalid >= self.invalid_frame_stop_count:
            self._u_prev = 0.0
            self._last_command = (0.0, 0.0)
        return self._last_command
