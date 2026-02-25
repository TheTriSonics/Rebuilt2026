import math
from robotpy_apriltag import AprilTagFieldLayout, AprilTagField
from wpilib import Timer
import ntcore
from wpimath.geometry import Pose2d, Rotation2d, Translation3d, Rotation3d, Transform3d
from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator
from components.drivetrain import DrivetrainComponent
from wpimath import units
from utilities.game import is_sim, is_disabled


class VisionComponent:

    drivetrain: DrivetrainComponent

    def __init__(self) -> None:
        self.timer = Timer()
        self.camera_rr = PhotonCamera("Rear_Right")
        self.camera_rl = PhotonCamera("Rear_Left")
        self.camera_back = PhotonCamera("Rear")

        self.camera_rr_offset = Transform3d(
            Translation3d(
                units.inchesToMeters(-11.0),    # Forward/backward offset
                units.inchesToMeters(11.625),   # Left/right offset, right is negative
                units.inchesToMeters(6.0),      # Up/down offset
            ),
            Rotation3d.fromDegrees(0.0, -22.0, 90.0),  # roll, pitch, yaw
        )
        self.camera_rl_offset = Transform3d(
            Translation3d(
                units.inchesToMeters(-11.0),    # Forward/backward offset
                units.inchesToMeters(-11.625),  # Left/right offset, right is negative
                units.inchesToMeters(6.0),      # Up/down offset
            ),
            Rotation3d.fromDegrees(0.0, -22.0, -90.0),  # roll, pitch, yaw
        )
        self.camera_back_offset = Transform3d(
            Translation3d(
                units.inchesToMeters(-10.0),  # Forward/backward offset
                units.inchesToMeters(0.0),    # Left/right offset, right is negative
                units.inchesToMeters(15.0),   # Up/down offset
            ),
            Rotation3d.fromDegrees(0.0, 10.0, 0.0),  # roll, pitch, yaw
        )
        self.linear_baseline_std = 0.10  # meters
        self.angular_baseline_std = math.radians(10)
        self.angular_baseline_std_sim = math.radians(30)

        field = AprilTagFieldLayout.loadField(AprilTagField.k2026RebuiltWelded)

        self.pose_estimator_rr = PhotonPoseEstimator(field, self.camera_rr_offset)
        self.pose_estimator_rl = PhotonPoseEstimator(field, self.camera_rl_offset)
        self.pose_estimator_back = PhotonPoseEstimator(field, self.camera_back_offset)

        self.publisher_rr = (
            ntcore.NetworkTableInstance.getDefault()
            .getStructTopic("/components/vision/pose_rr", Pose2d)
            .publish()
        )
        self.publisher_rl = (
            ntcore.NetworkTableInstance.getDefault()
            .getStructTopic("/components/vision/pose_rl", Pose2d)
            .publish()
        )
        self.publisher_back = (
            ntcore.NetworkTableInstance.getDefault()
            .getStructTopic("/components/vision/pose_back", Pose2d)
            .publish()
        )

        # Rejection stats published to NT for debugging
        nt = ntcore.NetworkTableInstance.getDefault().getTable("/components/vision")
        self._rejected_z = nt.getIntegerTopic("rejected_z").publish()
        self._rejected_yaw = nt.getIntegerTopic("rejected_yaw").publish()
        self._rejected_origin = nt.getIntegerTopic("rejected_origin").publish()
        self._rejected_stale = nt.getIntegerTopic("rejected_stale").publish()
        self._accepted = nt.getIntegerTopic("accepted").publish()
        self._rejected_z_count = 0
        self._rejected_yaw_count = 0
        self._rejected_origin_count = 0
        self._rejected_stale_count = 0
        self._accepted_count = 0

        # All cameras active
        self.cameras = [self.camera_rr, self.camera_rl, self.camera_back]
        self.pose_estimators = [
            self.pose_estimator_rr,
            self.pose_estimator_rl,
            self.pose_estimator_back,
        ]
        self.publishers = [
            self.publisher_rr,
            self.publisher_rl,
            self.publisher_back,
        ]

        # Stale timestamp tracking per camera
        self._last_timestamps: list[float] = [0.0] * len(self.cameras)

        # Gyro-fused fallback: track recent yaw rates for stability check
        self._yaw_rate_history: list[float] = []
        self._yaw_rate_history_max = 15  # ~0.3s at 50Hz

    def _compute_std_devs(
        self, avg_dist: float, tag_count: int, is_single_tag_gyro_fused: bool = False
    ) -> tuple[float, float, float]:
        """Compute standard deviations for a vision measurement."""
        angular_baseline = self.angular_baseline_std_sim if is_sim() else self.angular_baseline_std
        std_factor = (avg_dist ** 2) / tag_count
        std_xy = self.linear_baseline_std * std_factor
        if is_single_tag_gyro_fused:
            # Gyro heading is far more trustworthy; tell Kalman filter to ignore vision heading
            std_rot = 1e6
        else:
            std_rot = angular_baseline * std_factor
        return (std_xy, std_xy, std_rot)

    def _reject_measurement(
        self, pose3d, twod_pose: Pose2d, ts: float, cam_idx: int,
        res, robot_pose: Pose2d, disabled: bool
    ) -> bool:
        """Return True if the measurement should be rejected."""
        # Z-height check: reject poses that claim robot is far off ground
        if abs(pose3d.Z()) > 0.2:
            self._rejected_z_count += 1
            return True

        # Field origin norm check: reject degenerate PnP solutions near (0,0)
        if twod_pose.translation().norm() < 1.0:
            self._rejected_origin_count += 1
            return True

        # Stale timestamp check
        if ts <= self._last_timestamps[cam_idx]:
            self._rejected_stale_count += 1
            return True

        # Distance from current pose check (existing)
        dist = robot_pose.relativeTo(twod_pose).translation().norm()
        if dist > 1.0 and not disabled:
            return True

        # Yaw consistency check for weak observations
        targets = res.getTargets()
        tag_count = len(targets)
        total_area = sum(t.getArea() for t in targets)
        avg_area = total_area / tag_count if tag_count > 0 else 0
        if tag_count == 1 or avg_area < 2.0:
            # Compare vision heading to gyro heading
            gyro_heading = self.drivetrain.get_rotation().radians()
            vision_heading = twod_pose.rotation().radians()
            heading_diff = abs(math.atan2(
                math.sin(vision_heading - gyro_heading),
                math.cos(vision_heading - gyro_heading),
            ))
            if heading_diff > math.radians(5):
                self._rejected_yaw_count += 1
                return True

        return False

    def _fuse_estimates(
        self, estimates: list[tuple[Pose2d, float, tuple[float, float, float]]]
    ) -> tuple[Pose2d, float, tuple[float, float, float]]:
        """Fuse multiple vision estimates using inverse-variance weighting.

        Each estimate is (pose2d, timestamp, (std_x, std_y, std_rot)).
        Returns fused (pose2d, timestamp, std_devs).
        """
        if len(estimates) == 1:
            return estimates[0]

        # Use the most recent timestamp
        estimates.sort(key=lambda e: e[1])
        newest_ts = estimates[-1][1]

        # Time-align older estimates to newest using odometry delta
        aligned: list[tuple[Pose2d, tuple[float, float, float]]] = []
        for pose, ts, stds in estimates:
            if ts < newest_ts:
                # Approximate alignment: use the pose estimator's odometry
                # The time difference should be small (< 1 frame), so we use
                # the raw pose offset as a simple approximation
                dt = newest_ts - ts
                speeds = self.drivetrain.get_chassis_speeds()
                dx = speeds.vx * dt
                dy = speeds.vy * dt
                dtheta = speeds.omega * dt
                cos_t = math.cos(dtheta)
                sin_t = math.sin(dtheta)
                new_x = pose.x + dx * cos_t - dy * sin_t
                new_y = pose.y + dx * sin_t + dy * cos_t
                new_rot = pose.rotation().radians() + dtheta
                pose = Pose2d(new_x, new_y, Rotation2d(new_rot))
            aligned.append((pose, stds))

        # Inverse-variance weighting for X and Y independently
        sum_inv_var_x = 0.0
        sum_inv_var_y = 0.0
        sum_inv_var_rot = 0.0
        weighted_x = 0.0
        weighted_y = 0.0
        weighted_sin = 0.0
        weighted_cos = 0.0

        for pose, stds in aligned:
            inv_var_x = 1.0 / (stds[0] ** 2) if stds[0] > 0 else 1e6
            inv_var_y = 1.0 / (stds[1] ** 2) if stds[1] > 0 else 1e6
            inv_var_rot = 1.0 / (stds[2] ** 2) if stds[2] > 0 else 1e6

            sum_inv_var_x += inv_var_x
            sum_inv_var_y += inv_var_y
            sum_inv_var_rot += inv_var_rot

            weighted_x += pose.x * inv_var_x
            weighted_y += pose.y * inv_var_y
            rot = pose.rotation().radians()
            weighted_sin += math.sin(rot) * inv_var_rot
            weighted_cos += math.cos(rot) * inv_var_rot

        fused_x = weighted_x / sum_inv_var_x
        fused_y = weighted_y / sum_inv_var_y
        fused_rot = math.atan2(weighted_sin, weighted_cos)

        fused_std_x = math.sqrt(1.0 / sum_inv_var_x)
        fused_std_y = math.sqrt(1.0 / sum_inv_var_y)
        fused_std_rot = math.sqrt(1.0 / sum_inv_var_rot)

        fused_pose = Pose2d(fused_x, fused_y, Rotation2d(fused_rot))
        return (fused_pose, newest_ts, (fused_std_x, fused_std_y, fused_std_rot))

    def _try_gyro_fused_single_tag(self, res, pose_est, pose3d):
        """Attempt gyro-fused single-tag fallback.

        When multi-tag fails and only a single tag is visible:
        - Check yaw rate stability over last 0.3s
        - Replace vision heading with gyro heading
        - Return (pose2d, is_gyro_fused) or None if not usable
        """
        pupdate = pose_est.estimateLowestAmbiguityPose(res)
        if pupdate is None:
            return None

        # Check yaw rate stability: all recent samples must be < 5 rad/s
        if len(self._yaw_rate_history) < 5:
            return pupdate, False  # Not enough history, use normal single-tag

        max_recent_yaw_rate = max(abs(yr) for yr in self._yaw_rate_history)
        if max_recent_yaw_rate > 5.0:
            return pupdate, False  # Too much rotation, use normal single-tag

        # Stable enough — fuse with gyro heading
        vision_pose = pupdate.estimatedPose.toPose2d()
        gyro_heading = self.drivetrain.get_rotation()
        fused_pose = Pose2d(vision_pose.x, vision_pose.y, gyro_heading)
        # Replace the 2D pose but keep the same pupdate structure info
        pupdate_2d_override = fused_pose
        return pupdate, True, pupdate_2d_override

    def execute(self) -> None:
        robot_pose = self.drivetrain.get_pose()
        disabled = is_disabled()
        setDevs = self.drivetrain.estimator.setVisionMeasurementStdDevs

        # Update yaw rate history for gyro-fused fallback
        yaw_rate = self.drivetrain.get_rotational_velocity()
        self._yaw_rate_history.append(yaw_rate)
        if len(self._yaw_rate_history) > self._yaw_rate_history_max:
            self._yaw_rate_history.pop(0)

        # Collect valid estimates from all cameras for fusion
        valid_estimates: list[tuple[Pose2d, float, tuple[float, float, float]]] = []

        for cam_idx, (cam, pose_est, pub) in enumerate(zip(
            self.cameras, self.pose_estimators, self.publishers
        )):
            results = cam.getAllUnreadResults()
            for res in results:
                best_target = res.getBestTarget()
                if best_target and (best_target.poseAmbiguity > 0.2):
                    continue

                # Try multi-tag estimation first (most accurate)
                pupdate = pose_est.estimateCoprocMultiTagPose(res)
                is_gyro_fused = False
                twod_pose_override = None

                if pupdate is None:
                    # Attempt gyro-fused single-tag fallback
                    result = self._try_gyro_fused_single_tag(res, pose_est, None)
                    if result is None:
                        continue
                    if len(result) == 3:
                        pupdate, is_gyro_fused, twod_pose_override = result
                    else:
                        pupdate, is_gyro_fused = result

                if pupdate is None:
                    continue

                pose3d = pupdate.estimatedPose
                twod_pose = twod_pose_override if twod_pose_override else pose3d.toPose2d()
                ts = pupdate.timestampSeconds
                pub.set(twod_pose)

                # Apply rejection pipeline
                if self._reject_measurement(
                    pose3d, twod_pose, ts, cam_idx, res, robot_pose, disabled
                ):
                    continue

                # Compute std devs
                targets = res.getTargets()
                tag_count = len(targets)
                total_dist = sum(
                    t.getBestCameraToTarget().translation().norm() for t in targets
                )
                avg_dist = total_dist / tag_count
                if avg_dist > 2.0 and not disabled:
                    continue

                std_devs = self._compute_std_devs(avg_dist, tag_count, is_gyro_fused)

                # Record timestamp
                self._last_timestamps[cam_idx] = ts

                valid_estimates.append((twod_pose, ts, std_devs))

        # Fuse and apply
        if valid_estimates:
            if len(valid_estimates) == 1:
                pose, ts, stds = valid_estimates[0]
            else:
                pose, ts, stds = self._fuse_estimates(valid_estimates)

            setDevs(stds)
            self.drivetrain.estimator.addVisionMeasurement(pose, ts)
            self._accepted_count += 1

        # Publish rejection stats
        self._rejected_z.set(self._rejected_z_count)
        self._rejected_yaw.set(self._rejected_yaw_count)
        self._rejected_origin.set(self._rejected_origin_count)
        self._rejected_stale.set(self._rejected_stale_count)
        self._accepted.set(self._accepted_count)
