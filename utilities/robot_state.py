"""
Custom pose estimator ported from 6328 Mechanical Advantage's RobotState.

Drop-in replacement for WPILib's SwerveDrive4PoseEstimator with two key
advantages:

1. Odometry standard deviations can be changed at runtime via setStateStdDevs()
   without recreating the estimator object.
2. Closed-form diagonal Kalman gain recomputed per vision measurement, so
   changing trust levels takes effect immediately.

The algorithm maintains two parallel poses:
  - _odometry_pose: pure wheel + gyro integration (no vision)
  - _estimated_pose: fused pose (odometry + vision corrections)

Vision measurements are latency-compensated using a 2-second interpolation
buffer of historical odometry poses.
"""

import bisect
import math

import wpilib
from wpimath.geometry import Pose2d, Rotation2d, Twist2d
from wpimath.kinematics import (
    SwerveDrive4Kinematics,
    SwerveModulePosition,
)

# How many seconds of odometry history to keep for latency compensation.
_BUFFER_DURATION = 2.0


class RobotState:
    """Kalman-filter pose estimator with runtime-adjustable odometry trust."""

    def __init__(
        self,
        kinematics: SwerveDrive4Kinematics,
        gyro_angle: Rotation2d,
        module_positions: tuple[
            SwerveModulePosition,
            SwerveModulePosition,
            SwerveModulePosition,
            SwerveModulePosition,
        ],
        initial_pose: Pose2d,
        state_std_devs: tuple[float, float, float] = (0.01, 0.01, 0.01),
        vision_measurement_std_devs: tuple[float, float, float] = (0.4, 0.4, 0.2),
    ) -> None:
        self._kinematics = kinematics
        self._odometry_pose = initial_pose
        self._estimated_pose = initial_pose

        # Squared odometry std devs — mutable for runtime changes.
        self._q = [s * s for s in state_std_devs]

        # Cached vision std devs (set before each add_vision_measurement call).
        self._vision_std_devs = vision_measurement_std_devs

        # Gyro offset so that heading = gyro + offset matches the pose frame.
        self._gyro_offset = initial_pose.rotation() - gyro_angle

        # Previous module positions for computing twist deltas.
        self._prev_module_positions = module_positions

        # Pose interpolation buffer: sorted list of (timestamp, Pose2d).
        self._buffer: list[tuple[float, Pose2d]] = []

    # ── Odometry update ───────────────────────────────────────────────────

    def update(
        self,
        gyro_angle: Rotation2d,
        module_positions: tuple[
            SwerveModulePosition,
            SwerveModulePosition,
            SwerveModulePosition,
            SwerveModulePosition,
        ],
    ) -> Pose2d:
        """Apply one odometry cycle.  Called every 20 ms control loop."""
        # Compute the twist from wheel position deltas.
        deltas = tuple(
            SwerveModulePosition(
                curr.distance - prev.distance,
                curr.angle,
            )
            for prev, curr in zip(self._prev_module_positions, module_positions, strict=True)
        )
        twist = self._kinematics.toTwist2d(deltas)

        # Override twist rotation with the (more accurate) gyro delta.
        gyro_heading = gyro_angle + self._gyro_offset
        twist = Twist2d(
            twist.dx,
            twist.dy,
            (gyro_heading - self._odometry_pose.rotation()).radians(),
        )

        # Advance both poses by the same twist.
        self._odometry_pose = self._odometry_pose.exp(twist)
        self._estimated_pose = self._estimated_pose.exp(twist)

        # Buffer the odometry pose for future latency compensation.
        ts = wpilib.Timer.getFPGATimestamp()
        self._buffer_add(ts, self._odometry_pose)

        self._prev_module_positions = module_positions
        return self._estimated_pose

    # ── Vision correction ─────────────────────────────────────────────────

    def add_vision_measurement(self, vision_pose: Pose2d, timestamp: float) -> None:
        """Fuse a latency-compensated vision measurement."""
        # Look up odometry pose at the vision measurement's timestamp.
        sample = self._buffer_sample(timestamp)
        if sample is None:
            return  # Too old — outside the buffer window.

        # The accumulated vision correction twist (odom → estimated).
        odom_to_est = self._odometry_pose.log(self._estimated_pose)

        # Time-travel the estimated pose back to the vision timestamp.
        estimated_at_time = sample.exp(odom_to_est)

        # Compute closed-form diagonal Kalman gain.
        r = [s * s for s in self._vision_std_devs]
        k = [0.0, 0.0, 0.0]
        for i in range(3):
            qi = self._q[i]
            ri = r[i]
            if qi == 0.0:
                k[i] = 0.0
            else:
                k[i] = qi / (qi + math.sqrt(qi * ri))

        # Innovation: twist from estimated-at-time to the vision pose.
        innovation = estimated_at_time.log(vision_pose)
        scaled_twist = Twist2d(
            k[0] * innovation.dx,
            k[1] * innovation.dy,
            k[2] * innovation.dtheta,
        )

        # Apply scaled correction, then replay odometry forward to now.
        corrected_at_time = estimated_at_time.exp(scaled_twist)
        vision_to_now = sample.log(self._odometry_pose)
        self._estimated_pose = corrected_at_time.exp(vision_to_now)

    def set_vision_std_devs(self, std_devs: tuple[float, float, float]) -> None:
        """Cache vision std devs for the next addVisionMeasurement call."""
        self._vision_std_devs = std_devs

    # ── State std devs (the key feature) ──────────────────────────────────

    def set_odometry_std_devs(self, std_devs: tuple[float, float, float]) -> None:
        """Change odometry trust at runtime — no estimator recreation needed."""
        self._q = [s * s for s in std_devs]

    # ── Pose accessors ────────────────────────────────────────────────────

    def get_estimated_position(self) -> Pose2d:
        return self._estimated_pose

    def reset_position(
        self,
        gyro_angle: Rotation2d,
        module_positions: tuple[
            SwerveModulePosition,
            SwerveModulePosition,
            SwerveModulePosition,
            SwerveModulePosition,
        ],
        pose: Pose2d,
    ) -> None:
        """Reset the estimator to a known pose and clear history."""
        self._odometry_pose = pose
        self._estimated_pose = pose
        self._gyro_offset = pose.rotation() - gyro_angle
        self._prev_module_positions = module_positions
        self._buffer.clear()

    def get_estimated_position(self) -> Pose2d:  # noqa: N802
        return self.get_estimated_position()

    def reset_position(  # noqa: N802
        self,
        gyro_angle: Rotation2d,
        module_positions: tuple[
            SwerveModulePosition,
            SwerveModulePosition,
            SwerveModulePosition,
            SwerveModulePosition,
        ],
        pose: Pose2d,
    ) -> None:
        self.reset_position(gyro_angle, module_positions, pose)

    # ── Interpolation buffer (simple sorted list) ─────────────────────────
    def _buffer_add(self, timestamp: float, pose: Pose2d) -> None:
        """Add a timestamped odometry pose and prune old entries."""
        self._buffer.append((timestamp, pose))
        cutoff = timestamp - _BUFFER_DURATION
        # Prune entries older than the buffer window.
        while self._buffer and self._buffer[0][0] < cutoff:
            self._buffer.pop(0)

    def _buffer_sample(self, timestamp: float) -> Pose2d | None:
        """Interpolate the odometry pose at a given timestamp."""
        if not self._buffer:
            return None

        # Out of range checks.
        if timestamp < self._buffer[0][0]:
            return None
        if timestamp >= self._buffer[-1][0]:
            return self._buffer[-1][1]

        # Binary search for the surrounding entries.
        timestamps = [entry[0] for entry in self._buffer]
        idx = bisect.bisect_right(timestamps, timestamp)
        if idx == 0:
            return self._buffer[0][1]

        t0, p0 = self._buffer[idx - 1]
        t1, p1 = self._buffer[idx]

        # Twist-based interpolation (correct on the Lie group, and
        # avoids Pose2d.interpolate which may not exist in RobotPy).
        if t1 == t0:
            return p0
        t = (timestamp - t0) / (t1 - t0)
        twist = p0.log(p1)
        return p0.exp(Twist2d(twist.dx * t, twist.dy * t, twist.dtheta * t))
