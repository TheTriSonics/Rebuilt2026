import math
from wpilib import SmartDashboard
from wpimath.geometry import Pose2d
from magicbot import AutonomousStateMachine, state, tunable

from components.intake import IntakeComponent
from utilities.game import is_red, is_left
from utilities.choreo_utils import mirrored
from components.drivetrain import DrivetrainComponent
from components.gyro import GyroComponent
from controllers.tanker import Tanker
from controllers.gaspump import GasPump
from components.shot_calculator import ShotCalculatorComponent

from autonomous.base import AutonBase
from choreo import load_swerve_trajectory
from choreo.trajectory import SwerveTrajectory, SwerveSample, EventMarker

pn = SmartDashboard.putNumber
ps = SmartDashboard.putString


# 
class HopperShoot(AutonBase):
    MODE_NAME = "HopperShoot"
    TRAJ_NAME = "HopperShoot"
    raw_traj = load_swerve_trajectory(TRAJ_NAME)

    tanker: Tanker
    gaspump: GasPump

    drivetrain: DrivetrainComponent
    gyro: GyroComponent
    intake: IntakeComponent

    def get_initial_pose(self) -> Pose2d:
        self.traj = mirrored(self.raw_traj) if is_left() else self.raw_traj
        # Get the markers from the trajectory now
        for e in self.traj.events:
            print(e.event)

        self.intake_on_pose = self.get_event_pose("IntakeOn")
        self.intake_off_pose = self.get_event_pose("IntakeOff")
        self.shoot_on_pose = self.get_event_pose("ShooterOn")
        self.shoot_off_pose = self.get_event_pose("ShooterOff")

        sample = self.traj.sample_at(0.0, is_red())
        assert sample
        return sample.get_pose()

    @state(first=True, must_finish=True)
    def begin_path(self, initial_call: bool):
        if initial_call:
            assert self.traj
            self.tanker.go_follow_traj(self.traj)

        if self.at_pose(self.intake_on_pose, tolerance=0.15):
            self.intake.on()

        if self.at_pose(self.intake_off_pose, tolerance=0.15):
            self.intake.off()

        if self.at_pose(self.shoot_on_pose, tolerance=0.15):
            self.gaspump.go_shoot()

        if self.at_pose(self.shoot_off_pose, tolerance=0.15):
            self.gaspump.go_shoot_off()
