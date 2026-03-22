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

    tanker: Tanker
    gaspump: GasPump

    drivetrain: DrivetrainComponent
    gyro: GyroComponent
    intake: IntakeComponent
    shot_calc: ShotCalculatorComponent

    def get_initial_pose(self) -> Pose2d:
        self.traj = mirrored(self.raw_traj) if is_left() else self.raw_traj
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
            self.next_state(self.end)

    @state(must_finish=True)
    def end(self, initial_call: bool):
        self.gaspump.go_shoot_off()
        self.intake.off()
        # TODO: remove this before competition, or make it not do it with an FMS connected
        self.intake.rotate_up()


class HopperShootTwo(AutonBase):
    MODE_NAME = "HopperShootTwo"
    raw_traj = load_swerve_trajectory(MODE_NAME)
    raw_second_traj = load_swerve_trajectory("HopperShootTwo")

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


class PlayerStationBump(AutonBase):
    MODE_NAME = "PlayerStationBump"
    raw_traj = load_swerve_trajectory(MODE_NAME)

    tanker: Tanker
    gaspump: GasPump

    drivetrain: DrivetrainComponent
    gyro: GyroComponent
    intake: IntakeComponent
    shot_calc: ShotCalculatorComponent
    wait_delay = 3.0

    def get_initial_pose(self) -> Pose2d:
        self.full_traj = mirrored(self.raw_traj) if is_left() else self.raw_traj

        # splits is a list of sample indices, e.g. [0, 83] means:
        #   segment 1 = samples[0:83]    (drive to the split waypoint)
        #   segment 2 = samples[83:]     (drive from split waypoint onward)
        split_idx = self.full_traj.splits[1]
        all_samples = self.full_traj.samples

        # Build segment 1: everything up to the split point
        seg1_samples = all_samples[:split_idx]
        self.traj_seg1 = SwerveTrajectory(
            self.full_traj.name + "_seg1", seg1_samples, [0], []
        )

        # Build segment 2: everything from the split onward, with
        # timestamps shifted to start at 0 so the follower works correctly
        seg2_samples = all_samples[split_idx:]
        t_offset = seg2_samples[0].timestamp
        seg2_shifted = [
            SwerveSample(
                s.timestamp - t_offset,
                s.x, s.y, s.heading, s.vx, s.vy, s.omega,
                s.ax, s.ay, s.alpha, s.fx, s.fy,
            )
            for s in seg2_samples
        ]
        self.traj_seg2 = SwerveTrajectory(
            self.full_traj.name + "_seg2", seg2_shifted, [0], []
        )

        # Poses for transition checks
        self.pause_pose = seg1_samples[-1].flipped().get_pose() if is_red() else seg1_samples[-1].get_pose()
        self.last_pose = self.full_traj.get_final_pose(is_red())

        pose = self.full_traj.get_initial_pose(is_red())
        assert pose
        return pose

    @state(first=True, must_finish=True)
    def intake_down(self, initial_call: bool):
        if initial_call:
            self.intake.rotate_down()

        if self.intake.get_rotate_position() < 0.05:
            self.next_state(self.follow_seg1)

    @state(must_finish=True)
    def follow_seg1(self, initial_call: bool):
        """Follow the trajectory up to the split point."""
        if initial_call:
            self.tanker.go_follow_traj(self.traj_seg1)

        if self.at_pose(self.pause_pose, tolerance=0.15):
            self.next_state(self.wait_at_split)

    @state(must_finish=True)
    def wait_at_split(self, initial_call: bool, state_tm: float):
        """Sit still at the split point for 5 seconds."""
        if state_tm > self.wait_delay:
            self.next_state(self.follow_seg2)

    @state(must_finish=True)
    def follow_seg2(self, initial_call: bool, state_tm: float):
        """Resume following from the split point onward."""
        if initial_call:
            self.tanker.go_follow_traj(self.traj_seg2, set_pose=False)

        assert self.last_pose
        if self.at_pose(self.last_pose, tolerance=0.15):
            self.next_state(self.shoot)

    @state(must_finish=True)
    def shoot(self, initial_call: bool, state_tm: float):
        self.shot_calc.set_target('hub')
        self.tanker.go_drive_auto_target()
        if state_tm > 0.5:
            self.gaspump.go_shoot()
        if state_tm > 2.0:
            self.intake.rotate_tilt()
            self.intake.on()
        if state_tm > 6.0:
            self.next_state(self.end)

    @state(must_finish=True)
    def end(self, initial_call: bool):
        self.gaspump.go_shoot_off()
        self.intake.off()
        self.intake.rotate_up()


class PlayerStationBumpNoHang(AutonBase):
    MODE_NAME = "PlayerStationBumpNoHang"
    raw_traj = load_swerve_trajectory(MODE_NAME)

    tanker: Tanker
    gaspump: GasPump

    drivetrain: DrivetrainComponent
    gyro: GyroComponent
    intake: IntakeComponent
    shot_calc: ShotCalculatorComponent
    wait_delay = 3.0

    def get_initial_pose(self) -> Pose2d:
        self.full_traj = mirrored(self.raw_traj) if is_left() else self.raw_traj

        # splits is a list of sample indices, e.g. [0, 83] means:
        #   segment 1 = samples[0:83]    (drive to the split waypoint)
        #   segment 2 = samples[83:]     (drive from split waypoint onward)
        split_idx = self.full_traj.splits[1]
        all_samples = self.full_traj.samples

        # Build segment 1: everything up to the split point
        seg1_samples = all_samples[:split_idx]
        self.traj_seg1 = SwerveTrajectory(
            self.full_traj.name + "_seg1", seg1_samples, [0], []
        )

        # Build segment 2: everything from the split onward, with
        # timestamps shifted to start at 0 so the follower works correctly
        seg2_samples = all_samples[split_idx:]
        t_offset = seg2_samples[0].timestamp
        seg2_shifted = [
            SwerveSample(
                s.timestamp - t_offset,
                s.x, s.y, s.heading, s.vx, s.vy, s.omega,
                s.ax, s.ay, s.alpha, s.fx, s.fy,
            )
            for s in seg2_samples
        ]
        self.traj_seg2 = SwerveTrajectory(
            self.full_traj.name + "_seg2", seg2_shifted, [0], []
        )

        # Poses for transition checks
        self.pause_pose = seg1_samples[-1].flipped().get_pose() if is_red() else seg1_samples[-1].get_pose()
        self.last_pose = self.full_traj.get_final_pose(is_red())

        pose = self.full_traj.get_initial_pose(is_red())
        assert pose
        return pose

    @state(first=True, must_finish=True)
    def intake_down(self, initial_call: bool):
        if initial_call:
            self.intake.rotate_down()

        if self.intake.get_rotate_position() < 0.05:
            self.next_state(self.follow_seg1)

    @state(must_finish=True)
    def follow_seg1(self, initial_call: bool):
        """Follow the trajectory up to the split point."""
        if initial_call:
            self.tanker.go_follow_traj(self.traj_seg1)

        if self.at_pose(self.pause_pose, tolerance=0.15):
            self.next_state(self.wait_at_split)

    @state(must_finish=True)
    def wait_at_split(self, initial_call: bool, state_tm: float):
        """Sit still at the split point for 5 seconds."""
        if state_tm > self.wait_delay:
            self.next_state(self.follow_seg2)

    @state(must_finish=True)
    def follow_seg2(self, initial_call: bool, state_tm: float):
        """Resume following from the split point onward."""
        if initial_call:
            self.tanker.go_follow_traj(self.traj_seg2, set_pose=False)

        assert self.last_pose
        if self.at_pose(self.last_pose, tolerance=0.15):
            self.next_state(self.shoot)

    @state(must_finish=True)
    def shoot(self, initial_call: bool, state_tm: float):
        self.shot_calc.set_target('hub')
        self.tanker.go_drive_auto_target()
        if state_tm > 0.5:
            self.gaspump.go_shoot()
        if state_tm > 2.0:
            self.intake.rotate_tilt()
            self.intake.on()
        if state_tm > 6.0:
            self.next_state(self.end)

    @state(must_finish=True)
    def end(self, initial_call: bool):
        self.gaspump.go_shoot_off()
        self.intake.off()
        self.intake.rotate_up()





class HopperShoot_Move(AutonBase):
    MODE_NAME = "HopperShoot_Move"
    raw_traj = load_swerve_trajectory("HopperShoot")
    raw_second_traj = load_swerve_trajectory("HopperShoot_Move")

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
        if state_tm > 2.0:
            self.intake.rotate_tilt()
            self.intake.on()
        if state_tm > 4.0:
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

        if state_tm > 30.0:
            assert self.last_pose
            if self.at_pose(self.last_pose, tolerance=0.15):
                self.next_state(self.shoot2)

    @state(must_finish=True)
    def shoot2(self, initial_call: bool, state_tm: float):
        self.shot_calc.set_target('hub')
        self.tanker.go_drive_auto_target()
        if state_tm > 0.5:
            self.gaspump.go_shoot()
        if state_tm > 2.0:
            self.intake.rotate_tilt()
            self.intake.on()
        if state_tm > 4.0:
            self.next_state(self.end)