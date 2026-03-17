# File for all of the choreo related paths
import math
from wpilib import SmartDashboard
from wpimath.geometry import Pose2d
from magicbot import AutonomousStateMachine, state, feedback, tunable

from utilities.game import is_red, is_sim

from components.drivetrain import DrivetrainComponent
from components.intake import IntakeComponent
from components.gyro import GyroComponent

pb = SmartDashboard.putBoolean
pn = SmartDashboard.putNumber
ps = SmartDashboard.putString


# Auton routine that scores two algae in the processor, like setting up for
# the coopertition bonus.
class AutonBase(AutonomousStateMachine):
    drivetrain: DrivetrainComponent
    # intake: IntakeComponent
    gyro: GyroComponent

    pose_set = False
    selected_alliance = None

    initial_pose = None
    failure_pose = None

    pose_check = tunable(False)
    pose_error = tunable(0.0)

    def __init__(self):
        pass

    def get_initial_pose(self) -> Pose2d:
        return Pose2d(0, 0, 0)

    def set_initial_pose(self) -> None:
        # No need to set the pose twice!
        alliance = 'red' if is_red() else 'blue'
        if alliance is not self.selected_alliance:
            self.pose_set = False

        if self.pose_set is True:
            return

        initial_pose = self.get_initial_pose()
        assert initial_pose
        self.drivetrain.set_pose(initial_pose)
        self.selected_alliance = alliance
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
