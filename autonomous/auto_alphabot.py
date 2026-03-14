import math
import ntcore
from wpilib import SmartDashboard
from wpimath.geometry import Pose2d, Transform2d, Rotation2d
from magicbot import AutonomousStateMachine, state, feedback, tunable

from utilities.game import is_red
from components.drivetrain import DrivetrainComponent
from components.gyro import GyroComponent
from controllers.tanker import Tanker
from controllers.gaspump import GasPump

from autonomous.base import AutonBase
from choreo import load_swerve_trajectory

pb = SmartDashboard.putBoolean
pn = SmartDashboard.putNumber
ps = SmartDashboard.putString


# Base class for 'big one' autons that try and fill a face on the reef
class Right2Hopper(AutonBase):
    MODE_NAME = "Right 2 Hopper"
    TRAJ_NAME = "Right_2_hopper"
    traj = load_swerve_trajectory(TRAJ_NAME)

    tanker: Tanker
    gaspump: GasPump

    drivetrain: DrivetrainComponent
    gyro: GyroComponent

    def __init__(self):
        super().__init__()

    # The base class uses this in set_initial_pose() to set where the robot
    # thinks it should be at the beginning of an auton.  If you dont' provide
    # this method in an auton that inherits from AutonBase, it will default to
    # the origin, positon 0, 0, which is behind a player station on the blue
    # side.
    def get_initial_pose(self) -> Pose2d:
        intake_on_time = self.traj.splits[1] * 0.02
        intake_on_sample = self.traj.sample_at(intake_on_time, is_red())
        assert intake_on_sample
        self.intake_on_pose = intake_on_sample.get_pose()

        sample = self.traj.sample_at(0.0, is_red())
        assert sample
        return sample.get_pose()

    @state(first=True, must_finish=True)
    def begin_path(self, initial_call: bool):
        if initial_call:
            self.tanker.go_follow_traj(self.traj)

        if self.at_pose(self.intake_on_pose, tolerance=0.15):
            self.gaspump.go_intake()



