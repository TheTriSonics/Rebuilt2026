import math
from robotpy_apriltag import AprilTagFieldLayout, AprilTagField
from wpilib import Timer
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
                units.inchesToMeters(-10.0), # Forward/backward offset
                units.inchesToMeters(0.0), # Left/right offset, right is negative
                units.inchesToMeters(15.0), # Up/down offset
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
            .getStructTopic("/components/vision/pose_bl", Pose2d)
            .publish()
        )
        self.publisher_br = (
            ntcore.NetworkTableInstance.getDefault()
            .getStructTopic("/components/vision/pose_br", Pose2d)
            .publish()
        )
        self.publisher_back = (
            ntcore.NetworkTableInstance.getDefault()
            .getStructTopic("/components/vision/pose_back", Pose2d)
            .publish()
        )

        # self.cameras = [self.camera_rr, self.camera_rl, self.camera_back]  # Add back camera when we have one working
        self.cameras = [self.camera_rr]  # Add back camera when we have one working
        # self.pose_estimators = [
        #     self.pose_estimator_fr,
        #     self.pose_estimator_fl,
        #     self.pose_estimator_back,
        # ]
        # self.publishers = [
        #     self.publisher_rr,
        #     self.publisher_rl,
        #     self.publisher_back,
        # ]
        self.pose_estimators = [
            self.pose_estimator_rr,
        ]
        self.publishers = [
            self.publisher_rr,
        ]

    def execute(self) -> None:
        robot_pose = self.drivetrain.get_pose()
        disabled = is_disabled()
        linear_baseline_std = self.linear_baseline_std
        angular_baseline_std = self.angular_baseline_std_sim if is_sim() else self.angular_baseline_std
        setDevs = self.drivetrain.estimator.setVisionMeasurementStdDevs

        for cam, pose_est, pub in zip(
            self.cameras, self.pose_estimators, self.publishers
        ):
            results = cam.getAllUnreadResults()
            for res in results:
                best_target = res.getBestTarget()
                if best_target and (best_target.poseAmbiguity > 0.2):
                    continue

                # Try multi-tag estimation first (most accurate)
                pupdate = pose_est.estimateCoprocMultiTagPose(res)
                if pupdate is None:
                    # Fallback to single-tag estimation
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
                        std_factor = (avg_dist**2) / tag_count
                        std_xy = linear_baseline_std * std_factor
                        std_rot = angular_baseline_std * std_factor
                        setDevs((std_xy, std_xy, std_rot))
                        self.drivetrain.estimator.addVisionMeasurement(twod_pose, ts)
