Plan: Upgrade agbot_vision_nav from P-controller to MPC

## Context

The current `agbot_vision_nav` package uses a simple proportional controller (RowCenteringController in `controller.py`):

`angular_z = -(k_p * offset_norm + k_slope * slope_term)`

Both the professor and grad students have asked for an MPC (Model Predictive Controller), which matches the approach in all three relevant papers (CropFollow, P-AgNav, P-AgSLAM). The Gazebo simulation is now working, so the goal is to complete the controller upgrade and make the system ready for closed-loop testing in simulation.

Once this MPC is written and the tests pass, the next step is to launch `agbot_bringup` + the new `vision_nav` controller and tune the MPC parameters against the simulated corn-row world.

---

## Paper Review: How Others Did It

### CropFollow (most directly analogous — camera → MPC)

- **State:** (φ = robot heading relative to row, d = ratio of distance from left row to lane width)
- **Perception:** ResNet-18 directly predicts (φ, d) from a single RGB frame
- **Filter:** EKF fuses these noisy predictions with IMU
- **Control:** nonlinear robust MPC minimizes heading and lateral deviation over a receding horizon
- **Key lesson:** the two-signal state (heading + lateral) → MPC is exactly the structure we should use. Our `(slope_term, offset_norm)` is the direct analog of CropFollow’s `(φ, d)`.

### P-AgNav (LiDAR range view → MPC)

- **State:** robot’s position in the range-view image
- **Control:** MPC cost minimizes deviation from the corridor centerline in the range-view
- **Key lesson:** MPC cost is just “stay close to the desired state (centerline = zero error).” Same structure applies to our image-space state.

### Agronav (segmentation → centerline → path planner)

- Extracts centerline from segmentation mask using per-scanline boundary midpoints
- That’s exactly what our `centerline_estimator.py` already does — no change needed.

---

## MPC Design

### State Vector

`x_k = [offset_norm_k, slope_term_k]^T`

Both quantities are already produced by `centerline_estimator.py` with no changes:

- `offset_norm ∈ [-1, 1]`: normalized lateral distance of corridor midpoint from image center.
	- Positive = robot is LEFT of row center (corridor appears right in image).
	- Negative = robot is RIGHT of row center (corridor appears left in image).
- `slope_term = far_row_offset - near_row_offset`: heading proxy, per the sign convention locked in by the existing tests.
	- Positive = corridor tilts rightward in image = robot heading slightly LEFT of row.
	- Negative = corridor tilts leftward in image = robot heading slightly RIGHT of row.

### Control Input

`u_k = angular_z_k` (rad/s, positive = left turn under ROS REP-103)

### Kinematic Model (discrete, LTI)

Derived from unicycle kinematics in normalized image space:

`x[k+1] = A * x[k] + B * u[k]`

`A = [[1, alpha], [0, 1]]`  
`B = [[0], [beta]]`

**Physical meaning:**
- `alpha` (coupling): how much lateral drift (`offset_norm`) accumulates per step given a heading error (`slope_term`). Positive because `slope_term > 0` (heading left) → `offset_norm` increases over time (robot drifts left → corridor shifts right → `offset_norm` increases). Start: `0.10`.
- `beta` (control effectiveness): how much `angular_z` changes the heading (`slope_term`) per step. Positive because `angular_z > 0` (left turn) → `slope_term` increases (heading becomes more leftward). Start: `0.10`.

Both `alpha` and `beta` are exposed as tunable ROS params — they cannot be derived analytically without camera intrinsic + real-world corridor geometry measurements, so they must be tuned empirically once closed-loop simulation is running.

### Cost Function (over horizon N)

`J = sum_{k=1}^{N} [ q_offset * offset_norm[k]^2 + q_heading * slope_term[k]^2 ]`  
`+ sum_{k=0}^{N-1} [ r_control * angular_z[k]^2 + r_delta * (angular_z[k] - angular_z[k-1])^2 ]`

- `q_offset`: penalizes lateral deviation — the primary objective.
- `q_heading`: penalizes heading error — provides anticipatory correction like CropFollow’s φ term.
- `r_control`: penalizes large control inputs (reduces aggressive steering).
- `r_delta`: penalizes rapid changes in `angular_z` (smooth steering, prevents oscillation).
- `angular_z[-1] = u_prev` (the last applied command, stored in the controller).

### Constraints

- `|angular_z[k]| ≤ angular_z_max` (magnitude clamp)
- `|angular_z[k] - angular_z[k-1]| ≤ delta_angular_z_max` (rate clamp, smoothness)

### Solver

Use `scipy.optimize.minimize` with `method="SLSQP"`:

- Handles box bounds and inequality constraints.
- For `N=8` decision variables (`u_0…u_7`) this is < 1 ms even on CPU. No additional dependency.
- `scipy` is already available in ROS1 Noetic Python environments.

Applied in receding-horizon fashion: solve for the full sequence `U*`, apply only `u_0*`, discard the rest, and re-solve on the next frame.

---

## Sign-Convention Verification (before writing code)

---

## Files to Modify

### 1) `agbot_vision_nav/src/agbot_vision_nav/controller.py`

Replace `RowCenteringController` with `MPCRowController`. Same public interface:

- `__init__(N, dt, alpha, beta, q_offset, q_heading, r_control, r_delta, linear_x_cruise, angular_z_max, delta_angular_z_max, invalid_frame_stop_count)`
- `compute(offset_norm, slope_term, valid) -> (linear_x, angular_z)` (unchanged)
- `reset()` (unchanged)
- `consecutive_invalid` property (unchanged)

Internal additions:
- `self._u_prev = 0.0` (last applied `angular_z`, for delta-u constraint and `r_delta` cost)
- SLSQP solve loop over horizon `N`
- Same invalid-frame hold-then-stop state machine as the P-controller

Example helper:
