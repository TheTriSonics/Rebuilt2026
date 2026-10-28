# Experimental autons using the point-to-point follower (utilities/path_follower.py)
from wpimath.geometry import Pose2d, Translation2d
from magicbot import state

from components.intake import IntakeComponent
from components.shot_calculator import ShotCalculatorComponent
from utilities.game import is_red, is_left
from utilities.drive_path import DrivePath
from components.drivetrain import DrivetrainComponent
from components.gyro import GyroComponent
from controllers.tanker import Tanker
from controllers.gaspump import GasPump
from choreo import load_swerve_trajectory

from autonomous.base import AutonBase


class P2PNewPath(AutonBase):
    """Drive a PathPlanner path, then build a curve back to the start on the fly."""

    MODE_NAME = "P2P: New Path + return"
    raw_path = DrivePath.from_pathplanner("New Path")

    tanker: Tanker
    gaspump: GasPump

    drivetrain: DrivetrainComponent
    gyro: GyroComponent
    intake: IntakeComponent
    shot_calc: ShotCalculatorComponent

    def get_initial_pose(self) -> Pose2d:
        self.path = self.raw_path.for_side(is_red(), is_left())
        return self.path.start_pose()

    @state(first=True, must_finish=True)
    def drive_path(self, initial_call: bool):
        if initial_call:
            self.tanker.go_follow_drive_path(self.path)
        if self.tanker.is_path_finished():
            self.next_state(self.drive_back)

    @state(must_finish=True)
    def drive_back(self, initial_call: bool):
        if initial_call:
            # Built from wherever we actually ended up, swinging out through
            # the middle of our half of the field
            here = self.drivetrain.get_pose()
            home = self.path.start_pose()
            toward_center = -1.5 if is_red() else 1.5
            middle = Translation2d((here.x + home.x) / 2 + toward_center, (here.y + home.y) / 2)
            self.tanker.go_follow_drive_path(DrivePath.through(here, home, via=[middle]))
        if self.tanker.is_path_finished():
            self.next_state(self.end)

    @state(must_finish=True)
    def end(self, initial_call: bool):
        # Tanker stays in follow_drive_path, holding the final pose
        ...


class P2PHopperShoot(AutonBase):
    """HopperShoot's Choreo path, driven by the point-to-point follower."""

    MODE_NAME = "P2P: HopperShoot (Choreo)"
    raw_path = DrivePath.from_choreo(load_swerve_trajectory("HopperShoot"))

    tanker: Tanker
    gaspump: GasPump

    drivetrain: DrivetrainComponent
    gyro: GyroComponent
    intake: IntakeComponent
    shot_calc: ShotCalculatorComponent

    def get_initial_pose(self) -> Pose2d:
        self.path = self.raw_path.for_side(is_red(), is_left())
        return self.path.start_pose()

    @state(first=True, must_finish=True)
    def begin_path(self, initial_call: bool):
        if initial_call:
            self.intake.on()
            self.intake.extend_out()
            self.tanker.go_follow_drive_path(self.path)

        if self.tanker.path_passed("ShooterWarmUp"):
            self.gaspump.go_pre_speed()

        if self.tanker.is_path_finished():
            self.next_state(self.shoot)

    @state(must_finish=True)
    def shoot(self, initial_call: bool, state_tm: float):
        self.shot_calc.set_target("hub")
        self.tanker.go_drive_auto_target()
        if state_tm > 0.01:
            self.gaspump.go_shoot()
        if state_tm > 3.0:
            self.intake.pull_in()
            self.intake.on()
        if state_tm > 6.0:
            self.next_state(self.end)

    @state(must_finish=True)
    def end(self, initial_call: bool):
        self.gaspump.go_shoot_off()
        self.intake.off()
        self.intake.extend_out()
