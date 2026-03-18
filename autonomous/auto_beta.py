import math
import ntcore
from wpilib import SmartDashboard
from wpimath.geometry import Pose2d, Transform2d, Rotation2d
from magicbot import AutonomousStateMachine, state, feedback, tunable

from components.intake import IntakeComponent
from utilities.game import is_red, is_left
from utilities.choreo_utils import mirrored
from components.drivetrain import DrivetrainComponent
from components.gyro import GyroComponent
from controllers.tanker import Tanker
from controllers.gaspump import GasPump
from components.shooter import ShooterComponent
from components.shot_calculator import ShotCalculatorComponent

from autonomous.base import AutonBase
from choreo import load_swerve_trajectory
from choreo.trajectory import SwerveTrajectory, SwerveSample, EventMarker

pb = SmartDashboard.putBoolean
pn = SmartDashboard.putNumber
ps = SmartDashboard.putString


# 
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
        _base = mirrored(self.traj) if is_left() else self.traj
        intake_on_time = _base.splits[1] * 0.02
        intake_on_sample = _base.sample_at(intake_on_time, is_red())
        assert intake_on_sample
        self.intake_on_pose = intake_on_sample.get_pose()

        sample = _base.sample_at(0.0, is_red())
        assert sample
        self._active_traj = _base
        return sample.get_pose()

    @state(first=True, must_finish=True)
    def begin_path(self, initial_call: bool):
        if initial_call:
            self.tanker.go_follow_traj(self._active_traj)

        if self.at_pose(self.intake_on_pose, tolerance=0.15):
            ...



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
    
    def intake_pose(self) -> Pose2d:
        intake_off_time = self.traj.splits[2] * 0.02
        intake_off_sample = self.traj.sample_at(intake_off_time, is_red()) 
        assert intake_off_sample
        self.intake_off_pose = intake_off_sample.get_pose()

        sample = self.traj.sample_at(2.67, is_red())
        assert sample

        return sample.get_pose()


    def shooter_pose(self) -> Pose2d:
        shooter_on_time = self.traj.splits[3] * 0.02
        shooter_on_sample = self.traj.sample_at(shooter_on_time, is_red())
        assert shooter_on_sample
        self.shooter_on_pose = shooter_on_sample.get_pose()

        sample = self.traj.sample_at(3.58, is_red())
        assert sample

        return sample.get_pose()


    @state(first=True, must_finish=True)
    def begin_path(self, initial_call: bool):
        if initial_call:
            self.tanker.go_follow_traj(self.traj)

        if self.at_pose(self.intake_on_pose, tolerance=0.15):
            self.gaspump.go_intake()

        if self.at_pose(self.intake_off_pose, tolerance=0.15):
            self.gaspump.waiting()

        if self.at_pose(self.shooter_on_pose, tolerance=0.15):
            self.gaspump.go_shoot()


class RightBump(AutonBase):
    MODE_NAME = "RightBump"
    DEFAULT = True
    TRAJ_NAME = "RightBump"
    traj = load_swerve_trajectory(TRAJ_NAME)

    SHOOT_TIMEOUT = tunable(4.0)

    tanker: Tanker
    gaspump: GasPump
    drivetrain: DrivetrainComponent
    gyro: GyroComponent
    intake: IntakeComponent
    shot_calc: ShotCalculatorComponent

    def __init__(self):
        super().__init__()

    def get_initial_pose(self) -> Pose2d:
        _base = mirrored(self.traj) if is_left() else self.traj

        # splits[0] == 0 (always prepended by loader), splits[1] == first real split
        split_idx = _base.splits[1]
        t_split = _base.samples[split_idx].timestamp

        # Segment 1: start through split point (inclusive)
        seg1_samples = _base.samples[:split_idx + 1]
        seg1_events = [e for e in _base.events if e.timestamp <= t_split]
        self._traj1 = SwerveTrajectory(
            _base.name + "_1", seg1_samples, [0], seg1_events
        )

        # Segment 2: split point onward, timestamps re-zeroed so state_tm aligns
        seg2_raw = _base.samples[split_idx:]
        seg2_samples = [
            SwerveSample(
                s.timestamp - t_split, s.x, s.y, s.heading,
                s.vx, s.vy, s.omega, s.ax, s.ay, s.alpha, s.fx, s.fy
            )
            for s in seg2_raw
        ]
        seg2_events = [
            e.offset_by(-t_split)
            for e in _base.events if e.timestamp > t_split
        ]
        self._traj2 = SwerveTrajectory(
            _base.name + "_2", seg2_samples, [0], seg2_events
        )

        # Precompute alliance-flipped poses for use in state checks
        split_sample = _base.sample_at(t_split, is_red())
        assert split_sample
        self._split_pose = split_sample.get_pose()

        # Diagnostics: confirm events loaded correctly
        print(f"[RightBump] traj.events raw count: {len(_base.events)}")
        print(f"[RightBump] seg1 events ({len(seg1_events)}): "
              + (", ".join(f"{e.event}@{e.timestamp:.2f}s" for e in seg1_events) or "NONE"))
        print(f"[RightBump] seg2 events ({len(seg2_events)}): "
              + (", ".join(f"{e.event}@{e.timestamp:.2f}s" for e in seg2_events) or "NONE"))
        pn("RightBump/TrajEventCount", len(_base.events))
        pn("RightBump/Seg1EventCount", len(seg1_events))
        pn("RightBump/Seg2EventCount", len(seg2_events))

        sample = _base.sample_at(0.0, is_red())
        assert sample
        return sample.get_pose()

    def _handle_events(self, events: list[EventMarker], prev_t: float, curr_t: float,
                       shoot_next_state: str) -> None:
        for e in events:
            if prev_t < e.timestamp <= curr_t:
                print(f"[RightBump] event fired: {e.event} at t={e.timestamp:.2f}s")
                pn("RightBump/LastEventTime", e.timestamp)
                ps("RightBump/LastEvent", e.event)
                if e.event == "IntakeOn":
                    self.intake.on()
                elif e.event == "IntakeOff":
                    self.intake.set_speed(0)
                elif e.event == "Shoot":
                    self.next_state(shoot_next_state)

    @state(first=True, must_finish=True)
    def follow_segment_1(self, initial_call: bool, state_tm: float):
        if initial_call:
            self.tanker.go_follow_traj(self._traj1)
            self._prev_t1 = 0.0
            print(f"[RightBump] follow_segment_1 started, {len(self._traj1.events)} events to watch")
        pn("RightBump/Seg1Time", state_tm)
        self._handle_events(self._traj1.events, self._prev_t1, state_tm,
                            shoot_next_state="targeting")
        self._prev_t1 = state_tm
        # Fallback: if Shoot event not yet in traj, transition when robot reaches split pose
        if self.at_pose(self._split_pose, tolerance=0.25):
            self.next_state(self.targeting)

    @state(must_finish=True)
    def targeting(self, initial_call: bool):
        if initial_call:
            self.shot_calc.set_target("hub")
        self.tanker.go_drive_auto_target()
        if self.drivetrain.is_heading_aligned():
            self.next_state(self.shooting)

    @state(must_finish=True)
    def shooting(self, initial_call: bool, state_tm: float):
        self.gaspump.go_shoot()
        if state_tm >= self.SHOOT_TIMEOUT:
            self.gaspump.go_shoot_off()
            self.next_state(self.follow_segment_2)

    @state(must_finish=True)
    def follow_segment_2(self, initial_call: bool, state_tm: float):
        if initial_call:
            # No pose reset — vision has been correcting pose during the shooting pause
            self.tanker.go_follow_traj_no_reset(self._traj2)
            self._prev_t2 = 0.0
        # Shoot event in seg2 goes to targeting_end → shooting_end → done()
        self._handle_events(self._traj2.events, self._prev_t2, state_tm,
                            shoot_next_state="targeting_end")
        self._prev_t2 = state_tm

    @state(must_finish=True)
    def targeting_end(self, initial_call: bool):
        if initial_call:
            self.shot_calc.set_target("hub")
        self.tanker.go_drive_auto_target()
        if self.drivetrain.is_heading_aligned():
            self.next_state(self.shooting_end)

    @state(must_finish=True)
    def shooting_end(self, state_tm: float):
        self.gaspump.go_shoot()
        if state_tm >= self.SHOOT_TIMEOUT:
            self.gaspump.go_shoot_off()
            self.done()