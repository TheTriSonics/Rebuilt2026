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
        linear_baseline_std = 0.10  # meters
        angular_baseline_std = math.radians(10)  # degrees to radians
        robot_pose = self.drivetrain.get_pose()
        if is_sim():
            angular_baseline_std = math.radians(30)  # degrees to radians
        setDevs = self.drivetrain.estimator.setVisionMeasurementStdDevs

        # Use all cameras
        z = zip(
            self.cameras, self.pose_estimators, self.publishers, strict=False
        )

        for cam, pose_est, pub in z:
            results = cam.getAllUnreadResults()
            for res in results:
                best_target = res.getBestTarget()
                if best_target and (best_target.poseAmbiguity > 0.2):
                    # Skip using this pose in a vision update; it is too ambiguous
                    continue

                # Try multi-tag estimation first (most accurate)
                pupdate = pose_est.estimateCoprocMultiTagPose(res)
                if pupdate is None:
                    # Fallback to single-tag estimation
                    pupdate = pose_est.estimateLowestAmbiguityPose(res)

                if pupdate:
                    twod_pose = pupdate.estimatedPose.toPose2d()
                    ts = pupdate.timestampSeconds
                    # Check if we're too far off for this to be valid
                    pub.set(twod_pose)
                    dist = robot_pose.relativeTo(twod_pose).translation().norm()
                    # Reject poses that are more than 1 meter from current
                    if dist < 1.0 or is_disabled():
                        # Ok let's figure out a stddev for this
                        total_dist = 0.0
                        tag_count = len(res.getTargets())
                        total_dist = sum(t.getBestCameraToTarget().translation().norm() for t in res.getTargets())
                        avg_dist = total_dist / tag_count
                        if avg_dist > 2.0 and not is_disabled():
                            continue  # Skip anything where the average tag is too far away
                        std_factor = (avg_dist**2) / tag_count
                        std_xy = linear_baseline_std * std_factor
                        std_rot = angular_baseline_std * std_factor
                        setDevs((std_xy, std_xy, std_rot))
                        self.drivetrain.estimator.addVisionMeasurement(twod_pose, ts)
