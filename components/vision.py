import math
from robotpy_apriltag import AprilTagFieldLayout, AprilTagField
import ntcore
from wpimath.geometry import Pose2d, Translation3d, Rotation3d, Transform3d
from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator
from components.drivetrain import DrivetrainComponent
from wpimath import units
from utilities.game import is_sim, is_disabled


class VisionComponent:

    drivetrain: DrivetrainComponent

    def __init__(self) -> None:
        # Front cameras are backwards in left/right orientation!
        self.camera_fr = PhotonCamera("fr")
        self.camera_fl = PhotonCamera("fl")
        self.camera_back = PhotonCamera("back")

        self.camera_fr_offset = Transform3d(
            Translation3d(
                units.inchesToMeters(10.0), # Forward/backward offset
                units.inchesToMeters(-11.0), # Left/right offset, right is negative
                units.inchesToMeters(15.0), # Up/down offset
            ),
            Rotation3d.fromDegrees(0.0, 10.0, -15.0),  # roll, pitch, yaw
        )
        self.camera_fl_offset = Transform3d(
            Translation3d(
                units.inchesToMeters(10.0), # Forward/backward offset
                units.inchesToMeters(11.0), # Left/right offset, right is negative
                units.inchesToMeters(15.0), # Up/down offset
            ),
            Rotation3d.fromDegrees(0.0, 10.0, 15.0),  # roll, pitch, yaw
        )
        self.camera_back_offset = Transform3d(
            Translation3d(
                units.inchesToMeters(-10.0), # Forward/backward offset
                units.inchesToMeters(0.0), # Left/right offset, right is negative
                units.inchesToMeters(15.0), # Up/down offset
            ),
            Rotation3d.fromDegrees(0.0, 10.0, 0.0),  # roll, pitch, yaw
        )
        self.linear_baseline_std = 0.04  # meters
        self.angular_baseline_std = math.radians(5)
        self.angular_baseline_std_sim = math.radians(15)

        field = AprilTagFieldLayout.loadField(AprilTagField.k2026RebuiltWelded)

        self.pose_estimator_fr = PhotonPoseEstimator(field, self.camera_fr_offset)
        self.pose_estimator_fl = PhotonPoseEstimator(field, self.camera_fl_offset)
        self.pose_estimator_back = PhotonPoseEstimator(field, self.camera_back_offset)

        self.publisher_fr = (
            ntcore.NetworkTableInstance.getDefault()
            .getStructTopic("/components/vision/pose_fr", Pose2d)
            .publish()
        )
        self.publisher_fl = (
            ntcore.NetworkTableInstance.getDefault()
            .getStructTopic("/components/vision/pose_fl", Pose2d)
            .publish()
        )
        self.publisher_back = (
            ntcore.NetworkTableInstance.getDefault()
            .getStructTopic("/components/vision/pose_back", Pose2d)
            .publish()
        )

        self.cameras = [self.camera_fr, self.camera_fl, self.camera_back]  # Add back camera when we have one working
        self.pose_estimators = [
            self.pose_estimator_fr,
            self.pose_estimator_fl,
            self.pose_estimator_back,
        ]
        self.publishers = [
            self.publisher_fr,
            self.publisher_fl,
            self.publisher_back,
        ]

    def execute(self) -> None:
        robot_pose = self.drivetrain.get_pose()
        disabled = is_disabled()
        linear_baseline_std = self.linear_baseline_std
        angular_baseline_std = self.angular_baseline_std_sim if is_sim() else self.angular_baseline_std
        setDevs = self.drivetrain.estimator.setVisionMeasurementStdDevs

        # Collect candidate measurements from all cameras, then only apply
        # the best two (lowest std_xy) to avoid spending too long in the
        # Kalman filter update when all three cameras report at once.
        candidates = []

        for cam, pose_est, pub in zip(
            self.cameras, self.pose_estimators, self.publishers
        ):
            results = cam.getAllUnreadResults()
            for res in results:
                # Try multi-tag estimation first (most accurate)
                pupdate = pose_est.estimateCoprocMultiTagPose(res)
                is_multi_tag = pupdate is not None
                if not is_multi_tag:
                    # For single-tag, reject high-ambiguity results
                    best_target = res.getBestTarget()
                    if not best_target or best_target.poseAmbiguity > 0.4:
                        continue
                    pupdate = pose_est.estimateLowestAmbiguityPose(res)

                if pupdate:
                    twod_pose = pupdate.estimatedPose.toPose2d()
                    ts = pupdate.timestampSeconds
                    pub.set(twod_pose)
                    dist = robot_pose.relativeTo(twod_pose).translation().norm()
                    if dist < 1.0 or disabled:
                        targets = res.getTargets()
                        tag_count = len(targets)
                        total_dist = sum(t.getBestCameraToTarget().translation().norm() for t in targets)
                        avg_dist = total_dist / tag_count
                        if avg_dist > 2.0 and not disabled:
                            continue
                        std_factor = (avg_dist ** 1.2) / (tag_count ** 2)
                        std_xy = linear_baseline_std * std_factor
                        if is_multi_tag:
                            std_rot = angular_baseline_std * std_factor
                        else:
                            # Scale XY by ambiguity for single-tag
                            ambiguity = best_target.poseAmbiguity
                            std_xy *= 5.0 * (ambiguity + 0.1)
                            # Never trust single-tag rotation over gyro
                            std_rot = float('inf')
                        candidates.append((std_xy, std_rot, twod_pose, ts))

        # Apply only the two best measurements (lowest std_xy)
        candidates.sort(key=lambda c: c[0])
        for std_xy, std_rot, pose, ts in candidates[:2]:
            setDevs((std_xy, std_xy, std_rot))
            self.drivetrain.estimator.addVisionMeasurement(pose, ts)
