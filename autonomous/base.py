# File for all of the choreo related paths
import math
from wpimath.geometry import Pose2d
from magicbot import AutonomousStateMachine, state, tunable

from utilities.game import is_red, is_sim, is_left
from choreo.trajectory import SwerveTrajectory, SwerveSample, EventMarker

from components.drivetrain import DrivetrainComponent
from components.gyro import GyroComponent


# Auton routine that scores two algae in the processor, like setting up for
# the coopertition bonus.
class AutonBase(AutonomousStateMachine):
    drivetrain: DrivetrainComponent
    # intake: IntakeComponent
    gyro: GyroComponent

    pose_set = False
    selected_alliance = None
    cached_initial_pose = None

    initial_pose = None
    traj: SwerveTrajectory | None = None

    pose_check = tunable(False)
    pose_error = tunable(0.0)

    def get_initial_pose(self) -> Pose2d:
        return Pose2d(0, 0, 0)

    def get_event_pose(self, event_name: str) -> Pose2d:
        if self.traj is None:
            return Pose2d()
        else:
            event_matches = [e for e in self.traj.events if e.event == event_name]
            if len(event_matches) == 0:
                return Pose2d()
            event = event_matches[0]
            sample = self.traj.sample_at(event.timestamp, is_red())
            assert sample
            return sample.get_pose()

    def set_initial_pose(self) -> None:
        # No need to set the pose twice!
        config_key = ('red' if is_red() else 'blue') + ('_left' if is_left() else '_right')
        if config_key != self.selected_alliance:
            self.pose_set = False

        if self.pose_set is True:
            return

        initial_pose = self.get_initial_pose()
        assert initial_pose
        self.drivetrain.set_pose(initial_pose)
        self.cached_initial_pose = initial_pose
        self.selected_alliance = config_key
        self.pose_set = True

    def at_pose(self, pose: Pose2d, tolerance: float | None = None) -> bool:
        if tolerance is None:
            tolerance = 0.15 if is_sim() else 0.040
        robot_pose = self.drivetrain.get_pose()
        dist = robot_pose.relativeTo(pose).translation().norm()
        self.pose_error = dist
        self.pose_check = dist < tolerance
        return self.pose_check


    def execute(self):
        super().execute()
