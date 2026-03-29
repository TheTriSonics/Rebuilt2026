# RobotState Pose Estimator

Custom pose estimator ported from FRC 6328 (Mechanical Advantage) `RobotState.java`.
Drop-in replacement for WPILib's `SwerveDrive4PoseEstimator`.

## Why Not WPILib's Estimator?

WPILib's `SwerveDrive4PoseEstimator` has no `setStateStdDevs()` method. The
odometry standard deviations (`m_q`) are set once in the constructor and cannot
be changed. To adjust odometry trust (e.g., switching from autonomous to teleop),
you must recreate the entire estimator, losing all buffered history.

This estimator uses the same Kalman gain formula but a different latency
compensation architecture that makes `set_state_std_devs()` trivial.

## The Kalman Gain (Identical in Both)

Both use the closed-form solution to the continuous algebraic Riccati equation
(CARE) with A=0, C=I. The gain is a 3x3 diagonal matrix (x, y, theta
independent):

```
K[i] = q[i] / (q[i] + sqrt(q[i] * r[i]))

where  q[i] = state_std_dev[i]^2    (odometry trust)
       r[i] = vision_std_dev[i]^2   (vision trust)
```

| state_std | vision_std | K     | Effect              |
|-----------|-----------|-------|---------------------|
| 0.01      | 0.5       | 0.017 | Nearly ignore vision |
| 0.1       | 0.5       | 0.167 | Slight vision trust  |
| 0.1       | 0.1       | 0.500 | Equal trust          |
| 0.5       | 0.1       | 0.833 | Heavy vision trust   |
| 0.0       | any       | 0.000 | Fully ignore vision  |
| any       | 0.0       | 1.000 | Fully trust vision   |

## Architecture: Dual-Pose + Twist Replay

The estimator maintains two parallel poses:

- `_odometry_pose` -- pure wheel + gyro integration (no vision)
- `_estimated_pose` -- fused pose (odometry + vision corrections)

And a 2-second buffer of timestamped `_odometry_pose` snapshots for latency
compensation.

### Odometry Update (every 20ms)

```
twist = kinematics.toTwist2d(module_position_deltas)
twist.dtheta = gyro_delta           # gyro is more accurate than wheel-derived rotation
odometry_pose = odometry_pose.exp(twist)
estimated_pose = estimated_pose.exp(twist)
buffer.add(timestamp, odometry_pose)
```

Both poses advance by the same twist each cycle. The difference between them
is the accumulated vision correction.

### Vision Correction (latency-compensated)

```
1. sample = buffer.interpolate(vision_timestamp)        # odometry at vision time
2. odom_to_est = odometry_pose.log(estimated_pose)      # accumulated correction twist
3. estimated_at_time = sample.exp(odom_to_est)           # time-travel back
4. innovation = estimated_at_time.log(vision_pose)       # what vision says vs what we think
5. scaled = K * innovation                               # scale by Kalman gain
6. corrected = estimated_at_time.exp(scaled)             # apply correction in the past
7. estimated_pose = corrected.exp(sample.log(odometry_pose))  # replay forward to now
```

The key insight: the "vision correction" is just the twist between the two poses
(`odom_to_est`). This twist can be transported to any point in time by applying
it to the historical odometry pose. No chain of records to maintain or invalidate.

## How This Differs from WPILib

### WPILib's Approach: VisionUpdate Records

WPILib stores a map of `VisionUpdate(visionPose, odometryPose)` records. On each
odometry update, the current pose is computed as:

```
poseEstimate = latestVisionUpdate.visionPose
             + (currentOdometry - latestVisionUpdate.odometryPose)
```

When a vision measurement arrives, it interpolates the estimated pose at that
timestamp, applies the Kalman correction, stores a new VisionUpdate, and
**deletes all VisionUpdate records after that timestamp**. This means
out-of-order vision measurements destroy later corrections.

### Comparison

| Behavior                        | WPILib                          | RobotState (ours)                |
|---------------------------------|---------------------------------|----------------------------------|
| Out-of-order vision             | Deletes later corrections       | Each measurement independent     |
| Multiple cameras same cycle     | Order matters, later wins       | All corrections accumulate       |
| Changing state std devs         | Must recreate (loses history)   | `set_state_std_devs()` immediate |
| Changing vision std devs        | Per-measurement                 | Per-measurement                  |
| Per-update cost                 | O(1)                            | O(1)                             |
| Per-vision cost                 | O(1) amortized, O(n) worst case | O(1) always                      |
| Memory                          | Odometry buffer + VisionUpdate map | Odometry buffer only          |
| Pose buffer duration            | 1.5 seconds                     | 2.0 seconds                      |

### Computational Cost Per Vision Measurement

| Operation          | WPILib                                    | RobotState                          |
|--------------------|-------------------------------------------|-------------------------------------|
| Kalman gain        | 3 muls + 3 sqrts + 3 divs                | 3 muls + 3 sqrts + 3 divs          |
| Latency comp       | 2 interpolations + map insert + cleanup   | 1 interpolation + 2 log + 2 exp    |
| Trig operations    | ~10                                       | ~40 (log/exp use sin/cos/atan2)     |
| Wall time (roboRIO)| ~10-20 us                                 | ~20-40 us                           |

Both are well within the 20ms control loop budget.

## Usage

```python
from utilities.robot_state import RobotState

# Construction (in drivetrain setup)
estimator = RobotState(
    kinematics, gyro_heading, module_positions, initial_pose,
    state_std_devs=(0.01, 0.01, 0.01),
    vision_measurement_std_devs=(0.4, 0.4, 0.2),
)

# Every control loop
estimator.update(gyro_heading, module_positions)

# Vision measurements (from VisionComponent)
estimator.setVisionMeasurementStdDevs(std_devs)
estimator.addVisionMeasurement(pose, timestamp)

# Change odometry trust at runtime (no recreation needed)
estimator.setStateStdDevs((0.1, 0.1, 0.1))

# Reset to known pose
estimator.resetPosition(gyro_heading, module_positions, pose)
```

## Origin

Ported from `RobotState.java` in
[Mechanical-Advantage/RobotCode2026Public](https://github.com/Mechanical-Advantage/RobotCode2026Public).
The original is MIT-licensed. 6328 does not provide support for external use.
