# Experimental PathPlanner autons. Paths live in deploy/pathplanner/paths/
from wpimath.geometry import Pose2d
from magicbot import state

from components.intake import IntakeComponent
from components.shot_calculator import ShotCalculatorComponent
from utilities.game import is_red, is_left
from utilities.pathplanner_utils import load_pp_trajectory, pp_event_pose
from components.drivetrain import DrivetrainComponent
from components.gyro import GyroComponent
from controllers.tanker import Tanker
from controllers.gaspump import GasPump

from autonomous.base import AutonBase


class PPNewPath(AutonBase):
    """Drive New Path.path with PathPlanner's time-based follower, then hold the end pose."""

    MODE_NAME = "PP: New Path"
    PATH_NAME = "New Path"

    tanker: Tanker
    gaspump: GasPump

    drivetrain: DrivetrainComponent
    gyro: GyroComponent
    intake: IntakeComponent
    shot_calc: ShotCalculatorComponent

    def get_initial_pose(self) -> Pose2d:
        """Generate the trajectory for the current alliance/side; runs while disabled."""
        # Generated here (while disabled) because it's too slow to do at auton start
        self.pp_traj = load_pp_trajectory(self.PATH_NAME, is_red(), is_left())
        self.last_pose = self.pp_traj.getEndState().pose
        return self.pp_traj.getInitialPose()

    @state(first=True, must_finish=True)
    def begin_path(self, initial_call: bool, state_tm: float):
        """Follow the trajectory until its time is up and we're at the end pose."""
        if initial_call:
            self.tanker.go_follow_pp_traj(self.pp_traj)

        if state_tm > self.pp_traj.getTotalTimeSeconds():
            if self.at_pose(self.last_pose, tolerance=0.15):
                self.next_state(self.end)

    @state(must_finish=True)
    def end(self, initial_call: bool):
        """Done; Tanker keeps holding the final pose."""
        # Tanker stays in follow_pp_path, holding the final pose
        ...


class PPBumpyShoot(AutonBase):
    """BumpyShoot (auto_beta.py), but following deploy/pathplanner/paths/BumpyShoot.path
    with PathPlanner instead of the Choreo trajectory."""

    MODE_NAME = "PP: BumpyShoot"
    PATH_NAME = "BumpyShoot"

    tanker: Tanker
    gaspump: GasPump

    drivetrain: DrivetrainComponent
    gyro: GyroComponent
    intake: IntakeComponent
    shot_calc: ShotCalculatorComponent

    def get_initial_pose(self) -> Pose2d:
        """Generate the trajectory and find the marker poses; runs while disabled."""
        self.pp_traj = load_pp_trajectory(self.PATH_NAME, is_red(), is_left())
        self.intake_on_pose = pp_event_pose(self.PATH_NAME, self.pp_traj, "IntakeOn")
        self.shooter_warm_on_pose = pp_event_pose(self.PATH_NAME, self.pp_traj, "ShooterWarmUp")
        self.last_pose = self.pp_traj.getEndState().pose
        return self.pp_traj.getInitialPose()

    @state(first=True, must_finish=True)
    def begin_path(self, initial_call: bool, state_tm: float):
        """Follow the path, turning on the intake and shooter at their markers."""
        if initial_call:
            self.tanker.go_follow_pp_traj(self.pp_traj)

        if self.at_pose(self.intake_on_pose, tolerance=0.15):
            self.intake.on()
            self.intake.extend_out()

        if self.at_pose(self.shooter_warm_on_pose, tolerance=0.15) and state_tm > 4.0:
            self.gaspump.go_pre_speed()

        if self.at_pose(self.last_pose, tolerance=0.15):
            self.next_state(self.shoot)

    @state(must_finish=True)
    def shoot(self, initial_call: bool, state_tm: float):
        """Aim at the hub and shoot for six seconds, pulling the intake in halfway."""
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
        """Shut the shooter and intake off."""
        self.gaspump.go_shoot_off()
        self.intake.off()
        self.intake.extend_out()
