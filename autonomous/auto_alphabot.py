import math
import ntcore
from wpilib import SmartDashboard
from wpimath.geometry import Pose2d, Transform2d, Rotation2d
from magicbot import AutonomousStateMachine, state, feedback, tunable

from components.intake import IntakeComponent
from utilities.game import is_red
from components.drivetrain import DrivetrainComponent
from components.gyro import GyroComponent
from controllers.tanker import Tanker
from controllers.gaspump import GasPump
from components.shooter import ShooterComponent

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



class Left_blue_simple(AutonBase):
    MODE_NAME = "Left_blue_simple"
    TRAJ_NAME = "Left_blue_simple"
    traj = load_swerve_trajectory(TRAJ_NAME)

    tanker: Tanker
    gaspump: GasPump

    drivetrain: DrivetrainComponent
    gyro: GyroComponent
    intake: IntakeComponent
    shooter: ShooterComponent

    def __init__(self):
        super().__init__()

    # The base class uses this in set_initial_pose() to set where the robot
    # thinks it should be at the beginning of an auton.  If you dont' provide
    # this method in an auton that inherits from AutonBase, it will default to
    # the origin, positon 0, 0, which is behind a player station on the blue
    # side.
    def get_initial_pose(self) -> Pose2d:

        #rotate the intake down first for real robot
        intake_on_time = self.traj.splits[1] * 0.02
        intake_on_sample = self.traj.sample_at(intake_on_time, is_red())
        assert intake_on_sample
        self.intake_on_pose = intake_on_sample.get_pose()

        sample = self.traj.sample_at(0.0, is_red())
        assert sample

        return sample.get_pose()
    
    # def intake_pose(self) -> Pose2d:
    #     intake_off_time = self.traj.splits[2] * 0.02
    #     intake_off_sample = self.traj.sample_at(intake_off_time, is_red()) 
    #     assert intake_off_sample
    #     self.intake_off_pose = intake_off_sample.get_pose()

    #     return self.intake_off_pose

    def shooter_pose(self) -> Pose2d:
        shooter_on_time = self.traj.splits[3] * 0.02
        shooter_on_sample = self.traj.sample_at(shooter_on_time, is_red())
        assert shooter_on_sample
        self.shooter_on_pose = shooter_on_sample.get_pose()

        return self.shooter_on_pose

        


    @state(first=True, must_finish=True)
    def begin_path(self, initial_call: bool):
        if initial_call:
            self.tanker.go_follow_traj(self.traj)

        if self.at_pose(self.intake_on_pose, tolerance=0.15):
            self.gaspump.go_intake()

        # if self.at_pose(self.intake_off_pose, tolerance=0.15):
        #     self.gaspump.waiting()

        if self.at_pose(self.shooter_on_pose, tolerance=0.15):
            self.gaspump.go_shoot()