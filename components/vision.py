import math
from collections import deque

from robotpy_apriltag import AprilTagFieldLayout, AprilTagField
from wpilib import Timer
import ntcore
from wpimath.geometry import Pose2d, Pose3d, Translation3d, Rotation3d, Transform3d, Rotation2d
from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator
from components.drivetrain import DrivetrainComponent
from components.gyro import GyroComponent
from wpimath import units
from utilities.game import is_auton, is_sim, is_disabled, is_match

# Max trail length for AdvantageScope trajectory visualization
_TRAIL_MAX = 100


class VisionComponent:
    drivetrain: DrivetrainComponent
    gyro: GyroComponent

    def __init__(self) -> None:
        self.sim = is_sim()
        self.timer = Timer()
        self.angular_baseline_std = math.radians(10)
        self.angular_baseline_std_sim = math.radians(30)
        self.angular_baseline = (
            self.angular_baseline_std_sim if self.sim else self.angular_baseline_std
        )

        self.camera_rr = PhotonCamera("Rear_Right")
        self.camera_rl = PhotonCamera("Rear_Left")
        self.camera_back = PhotonCamera("Rear")

        self.camera_rr_offset = Transform3d(
            Translation3d(
                units.inchesToMeters(-12.0),  # Forward/backward offset
                units.inchesToMeters(-10.5),  # Left/right offset, left is positive
                units.inchesToMeters(8.5),  # Up/down offset
            ),
            # Pitching up is a negative value
            Rotation3d.fromDegrees(0.0, -22.0, -85.0),  # roll, pitch, yaw
        )
        self.camera_rl_offset = Transform3d(
            Translation3d(
                units.inchesToMeters(-12.0),  # Forward/backward offset
                units.inchesToMeters(10.5),  # Left/right offset, left is positive
                units.inchesToMeters(8.5),  # Up/down offset
            ),
            Rotation3d.fromDegrees(0.0, -22.0, 85.0),  # roll, pitch, yaw
        )
        self.camera_back_offset = Transform3d(
            Translation3d(
                units.inchesToMeters(-13.0),  # Forward/backward offset
                units.inchesToMeters(0.0),  # Left/right offset, right is negative
                units.inchesToMeters(7.5),  # Up/down offset
            ),
            Rotation3d.fromDegrees(0.0, -25.0, 180.0),  # roll, pitch, yaw
        )
        self.linear_baseline_std = 0.10  # meters

        self.field = AprilTagFieldLayout.loadField(AprilTagField.k2026RebuiltWelded)

        self.pose_estimator_rr = PhotonPoseEstimator(self.field, self.camera_rr_offset)
        self.pose_estimator_rl = PhotonPoseEstimator(self.field, self.camera_rl_offset)
        self.pose_estimator_back = PhotonPoseEstimator(self.field, self.camera_back_offset)

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

        self.camera_offsets = [
            self.camera_rr_offset,
            self.camera_rl_offset,
            self.camera_back_offset,
        ]

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

        # Pre-computed inverse offsets for single-tag estimation (all 3 cameras)
        self._camera_offsets_inv = [
            self.camera_rr_offset.inverse(),
            self.camera_rl_offset.inverse(),
            self.camera_back_offset.inverse(),
        ]

        # Stale timestamp tracking per camera
        self._last_timestamps: list[float] = [0.0] * len(self.cameras)

        # Vision quality tracking for dashboard
        self.last_vision_update: float = 0.0
        self._last_std_xy: float = float("inf")
        self._consecutive_frames: int = 0

        # Pose stability tracking — ring buffer of (timestamp, x, y)
        self._pose_history: deque[tuple[float, float, float]] = deque(maxlen=20)

        # Only needed for single-tag gyro-fused fallback
        # self._yaw_rate_history: deque[float] = deque(maxlen=15)
        # Debug NT publishers — created lazily on first non-FMS execute
        self._debug_pubs_created = False
        self._single_tag_pubs: list | None = None
        self._single_tag_trail_pub = None
        self._multi_tag_trail_pub = None

        # Trail buffers
        self._single_tag_trail: deque[Pose2d] = deque(maxlen=_TRAIL_MAX)
        self._multi_tag_trail: deque[Pose2d] = deque(maxlen=_TRAIL_MAX)

    def _create_debug_publishers(self) -> None:
        """Lazily create debug NT publishers (only when not on FMS)."""
        if self._debug_pubs_created:
            return
        self._debug_pubs_created = True

        nt = ntcore.NetworkTableInstance.getDefault()
        cam_names = ("rr", "rl", "back")
        self._single_tag_pubs = [
            nt.getStructTopic(f"/components/vision/single_tag_pose_{n}", Pose2d).publish()
            for n in cam_names
        ]
        self._single_tag_trail_pub = nt.getStructArrayTopic(
            "/components/vision/single_tag_trail", Pose2d
        ).publish()
        self._multi_tag_trail_pub = nt.getStructArrayTopic(
            "/components/vision/multi_tag_trail", Pose2d
        ).publish()

    # def _estimate_single_tag(self, targets: list, cam_idx: int) -> Pose2d | None:
    #     """Estimate robot pose from the best single tag using transform math.

    #     Uses pre-computed camera-to-target transforms from PhotonVision.
    #     Returns a Pose2d or None.  This is for debug visualization only —
    #     the result is NOT fed to the pose estimator.
    #     """
    #     if not targets:
    #         return None

    #     # Pick the lowest ambiguity target
    #     target = min(targets, key=lambda t: t.getPoseAmbiguity())
    #     tag_id = target.getFiducialId()
    #     tag_pose = self.field.getTagPose(tag_id)
    #     if tag_pose is None:
    #         return None

    #     # Compute robot pose: field_to_tag * inv(cam_to_tag) * inv(robot_to_cam)
    #     camera_to_target = target.getBestCameraToTarget()
    #     robot_pose_3d = tag_pose.transformBy(camera_to_target.inverse()).transformBy(
    #         self._camera_offsets_inv[cam_idx]
    #     )

    #     twod_pose = robot_pose_3d.toPose2d()

    #     # Correct the heading by 180 degrees
    #     # NOTE: This flip is likely a coordinate-system workaround.
    #     # 254's 2025 code avoids this entirely by re-projecting the
    #     # robot-to-tag vector with the gyro heading.  Phase 2 will
    #     # adopt that approach — for now we keep the flip so the debug
    #     # visualization matches historical behaviour.
    #     vision_heading = twod_pose.rotation().radians()
    #     flipped_heading = (vision_heading + math.pi) % math.tau
    #     twod_pose = Pose2d(twod_pose.x, twod_pose.y, Rotation2d(flipped_heading))

    #     return twod_pose

    def _compute_std_devs(
        self, avg_dist: float, tag_count: int, is_single_tag_gyro_fused: bool
    ) -> tuple[float, float, float]:
        """Compute standard deviations for a vision measurement."""
        std_factor = (avg_dist**2) / tag_count
        std_xy = self.linear_baseline_std * std_factor
        std_rot = self.angular_baseline * std_factor
        return (std_xy, std_xy, std_rot)

    def _reject_measurement(
        self,
        pose3d: Pose3d,
        twod_pose: Pose2d,
        ts: float,
        cam_idx: int,
        targets: list,
        robot_pose: Pose2d,
        disabled: bool,
    ) -> bool:
        """Return True if the measurement should be rejected."""
        # Z-height check: reject poses that claim robot is far off ground
        if abs(pose3d.Z()) > 0.2:
            return True

        # Stale timestamp check
        if ts <= self._last_timestamps[cam_idx]:
            return True

        # Distance from current pose check (existing)
        dist = robot_pose.relativeTo(twod_pose).translation().norm()
        if dist > 4.0 and not disabled:
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
        speeds = self.drivetrain.get_chassis_speeds()
        for pose, ts, stds in estimates:
            if ts < newest_ts:
                # Approximate alignment: use the pose estimator's odometry
                # The time difference should be small (< 1 frame), so we use
                # the raw pose offset as a simple approximation
                dt = newest_ts - ts
                dx = speeds.vx * dt
                dy = speeds.vy * dt
                dtheta = speeds.omega * dt
                heading = pose.rotation().radians()
                cos_t = math.cos(heading)
                sin_t = math.sin(heading)
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

    def has_good_vision(self) -> bool:
        """True when vision data is fresh, low-uncertainty, and stable.
        Requires a recent estimate (< 2s), std_xy < 0.5m, and 5+ consecutive frames."""
        age = Timer.getFPGATimestamp() - self.last_vision_update
        return age < 2.0 and self._last_std_xy < 0.5 and self._consecutive_frames >= 5

    def has_stable_pose(
        self, min_count: int = 6, tolerance: float = 0.10, max_age: float = 1.0
    ) -> bool:
        return False
        """True when enough recent vision poses are spatially consistent.

        Checks that at least *min_count* poses received in the last *max_age*
        seconds all fall within *tolerance* metres of their centroid.
        """
        now = Timer.getFPGATimestamp()
        recent = [(x, y) for ts, x, y in self._pose_history if now - ts <= max_age]
        if len(recent) < min_count:
            return False
        cx = sum(x for x, y in recent) / len(recent)
        cy = sum(y for x, y in recent) / len(recent)
        return all(math.hypot(x - cx, y - cy) <= tolerance for x, y in recent)

    def _is_off_field(self, pose: Pose2d) -> bool:
        return pose.x < -0.5 or pose.x > 17.0 or pose.y < -0.5 or pose.y > 8.5

    def execute(self) -> None:
        self.linear_baseline_std = 0.10 if is_auton() else 0.02
        disabled = is_disabled()
        current_pose = self.drivetrain.estimator.get_estimated_position()
        off_field = self._is_off_field(current_pose)

        # Debug publishing is suppressed during FMS matches to save bandwidth
        debug = not is_match()
        if debug:
            self._create_debug_publishers()

        valid_estimates: list[tuple[Pose2d, float, tuple[float, float, float]]] = []

        for cam_idx, (cam, pose_est, pub) in enumerate(
            zip(self.cameras, self.pose_estimators, self.publishers, strict=True)
        ):
            try:
                res = cam.getLatestResult()
            except Exception:
                continue

            if not res:
                continue

            targets = res.getTargets()
            if not targets:
                continue

            # Single-tag debug estimation — always attempted, never used for
            # pose estimation.  Published to NT only when not on FMS.
            # if debug and self._single_tag_pubs is not None:
            #     st_pose = self._estimate_single_tag(targets, cam_idx)
            #     if st_pose is not None:
            #         self._single_tag_pubs[cam_idx].set(st_pose)
            #         self._single_tag_trail.append(st_pose)

            best_target = res.getBestTarget()
            if best_target and best_target.poseAmbiguity > 0.2:
                continue

            # Try multi-tag estimation first (most accurate)
            pupdate = pose_est.estimateCoprocMultiTagPose(res)
            if pupdate is None:
                # No updated multi-tag pose? We skip this loop
                continue

            ts = pupdate.timestampSeconds
            if ts <= self._last_timestamps[cam_idx]:
                # We've already processed this pose estimate, skip it
                continue
            self._last_timestamps[cam_idx] = ts
            pose3d = pupdate.estimatedPose
            twod_pose = pose3d.toPose2d()
            pub.set(twod_pose)

            # Compute std devs
            tag_count = len(targets)
            total_dist = sum(t.getBestCameraToTarget().translation().norm() for t in targets)
            avg_dist = total_dist / tag_count

            # JJB: We might need to bump this up. We can pick up a pose further
            # away than this with our cameras running in high resolution mode.
            # Orig value: 2.0
            if avg_dist > 7.0 and not disabled:
                continue

            # Reject poses wildly divergent from current estimate, unless
            # disabled or off-field where we want vision to pull us back.
            dist = current_pose.relativeTo(twod_pose).translation().norm()
            accept_dist = 1.0 if is_auton() else 4.0
            if dist > accept_dist and not disabled and not off_field:
                continue

            # std_devs = self._compute_std_devs(avg_dist, tag_count, is_gyro_fused)

            std_factor = (avg_dist**2) / tag_count
            std_xy = self.linear_baseline_std * std_factor
            std_rot = self.angular_baseline * std_factor
            if off_field:
                # Robot is off-field: snap fully to vision, ignore odometry drift
                std_xy = 0.001
                std_rot = 0.001
            elif not disabled:
                # Vision should never be more trusted than wheel odometry.
                # Without this floor, close-range tags produce std devs near
                # or below the state stds (0.01), letting vision noise
                # dominate the pose estimate and cause drift.
                std_xy = max(std_xy, 0.05)
                std_rot = max(std_rot, 0.5)
            std_devs = (std_xy, std_xy, std_rot)

            valid_estimates.append((twod_pose, ts, std_devs))

        # Fuse and apply
        if valid_estimates:
            if len(valid_estimates) == 1:
                pose, ts, stds = valid_estimates[0]
            else:
                pose, ts, stds = self._fuse_estimates(valid_estimates)

            self.drivetrain.estimator.set_vision_std_devs(stds)
            self.drivetrain.estimator.add_vision_measurement(pose, ts)
            now = Timer.getFPGATimestamp()
            self.last_vision_update = now
            self._last_std_xy = stds[0]
            self._consecutive_frames = min(self._consecutive_frames + 1, 100)
            self._pose_history.append((now, pose.x, pose.y))

            # if debug:
            #     self._multi_tag_trail.append(pose)
        else:
            self._consecutive_frames = 0

        # Publish trails for AdvantageScope visualization
        # if debug and self._single_tag_trail_pub is not None:
        #     self._single_tag_trail_pub.set(list(self._single_tag_trail))
        #     self._multi_tag_trail_pub.set(list(self._multi_tag_trail))
