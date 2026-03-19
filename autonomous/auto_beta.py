import math
from wpilib import SmartDashboard
from wpimath.geometry import Pose2d
from magicbot import AutonomousStateMachine, state, tunable

from components.intake import IntakeComponent
from components.shot_calculator import ShotCalculatorComponent
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
    raw_traj = load_swerve_trajectory(MODE_NAME)
    raw_second_traj = load_swerve_trajectory("HopperShoot2")

    tanker: Tanker
    gaspump: GasPump

    drivetrain: DrivetrainComponent
    gyro: GyroComponent
    intake: IntakeComponent
    shot_calc: ShotCalculatorComponent

    def get_initial_pose(self) -> Pose2d:
        self.traj = mirrored(self.raw_traj) if is_left() else self.raw_traj
        self.second_traj = mirrored(self.raw_second_traj) if is_left() else self.raw_second_traj
        # Get the markers from the trajectory now
        for e in self.traj.events:
            print(e.event)

        self.intake_on_pose = self.get_event_pose("IntakeOn")
        self.last_pose = self.traj.get_final_pose(is_red())

        pose = self.traj.get_initial_pose(is_red())
        assert pose
        return pose

    @state(first=True, must_finish=True)
    def intake_down(self, initial_call: bool):
        if initial_call:
            self.intake.rotate_down()

        if self.intake.get_rotate_position() < 0.05:
            self.next_state(self.begin_path)


    @state(must_finish=True)
    def begin_path(self, initial_call: bool):
        if initial_call:
            assert self.traj
            self.tanker.go_follow_traj(self.traj)

        if self.at_pose(self.intake_on_pose, tolerance=0.15):
            self.intake.on()

        assert self.last_pose
        if self.at_pose(self.last_pose, tolerance=0.15):
            self.next_state(self.shoot)

    @state(must_finish=True)
    def shoot(self, initial_call: bool, state_tm: float):
        self.shot_calc.set_target('hub')
        self.tanker.go_drive_auto_target()
        if state_tm > 0.5:
            self.gaspump.go_shoot()
        if state_tm > 3.0:
            self.intake.rotate_tilt()
            self.intake.on()
        if state_tm > 6.0:
            self.next_state(self.hopper2)

    @state(must_finish=True)
    def hopper2(self, initial_call: bool):
        if initial_call:
            self.intake.rotate_down()
            self.gaspump.go_shoot_off()
        if self.intake.get_rotate_position() < 0.05:
            self.next_state(self.begin_second_path)

    @state(must_finish=True)
    def begin_second_path(self, initial_call: bool, state_tm: float):
        if initial_call:
            assert self.second_traj
            self.tanker.go_follow_traj(self.second_traj, set_pose=False)

        if state_tm > 3.0:
            assert self.last_pose
            if self.at_pose(self.last_pose, tolerance=0.15):
                self.next_state(self.shoot2)

    @state(must_finish=True)
    def shoot2(self, initial_call: bool, state_tm: float):
        self.shot_calc.set_target('hub')
        self.tanker.go_drive_auto_target()
        if state_tm > 0.5:
            self.gaspump.go_shoot()
        if state_tm > 3.0:
            self.intake.rotate_tilt()
            self.intake.on()
        if state_tm > 6.0:
            self.next_state(self.end)

    @state(must_finish=True)
    def end(self, initial_call: bool):
        self.gaspump.go_shoot_off()
        self.intake.off()
        self.intake.rotate_up()

