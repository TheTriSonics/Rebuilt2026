import math
from typing import Callable
from choreo.trajectory import SwerveTrajectory
import ntcore
from wpimath.geometry import Pose2d
from magicbot import StateMachine, state

from components.drivetrain import DrivetrainComponent
from components.intake import IntakeComponent
from components.shot_calculator import ShotCalculatorComponent
from controllers.gaspump import GasPump
from utilities.game import is_red
from choreo import load_swerve_trajectory

class Tanker(StateMachine):
    # Position tolerance thresholds
    POSITION_TOLERANCE_M = 0.02
    ROTATION_TOLERANCE_DEG = 1.0

    drivetrain: DrivetrainComponent
    intake: IntakeComponent
    shot_calc: ShotCalculatorComponent
    gaspump: GasPump

    stick_x, stick_y, stick_o = 0, 0, 0


    def __init__(self):
        super().__init__()
        self.target_pose_pub = (
            ntcore.NetworkTableInstance
            .getDefault()
            .getStructTopic("/components/tanker/target_pose", Pose2d)
            .publish()
        )
        self._last_drive_mode = self.go_drive_local
        self._fired_events: set[int] = set()

    def _get_event_actions(self) -> dict[str, Callable]:
        return {
            "intake_on": self.intake.on,
            "intake_off": self.intake.off,
            "go_shoot": self.gaspump.go_shoot,
            "go_eject": self.gaspump.go_eject,
            "go_stop": self.gaspump.go_stop,
            "go_intake": self.gaspump.go_intake,
        }

    def _process_events(self, state_tm: float) -> None:
        actions = self._get_event_actions()
        for i, marker in enumerate(self.traj.events):
            if i in self._fired_events:
                continue
            if marker.timestamp <= state_tm:
                self._fired_events.add(i)
                action = actions.get(marker.event)
                if action is not None:
                    print(f"[Tanker] Event '{marker.event}' fired at t={state_tm:.2f}s")
                    action()
                else:
                    print(f"[Tanker] Unknown event '{marker.event}' at t={marker.timestamp:.2f}s")

    def set_stick_values(self, x: float, y: float, rot: float):
        self.stick_x = x
        self.stick_y = y
        self.stick_o = rot

    def toggle_mode(self) -> None:
        # Toggle between field relative and robot relative driving
        if self.current_state == self.drive_field.name:
            self.go_drive_local()
        else:
            self.go_drive_field()

    def go_drive_field(self):
        self._last_drive_mode = self.go_drive_field
        if self.current_state != self.drive_field.name:
            self.drivetrain.stop_snapping()
            self.next_state_now(self.drive_field)

    def go_drive_auto_target(self):
        if self.current_state != self.drive_auto_target.name:
            self.next_state_now(self.drive_auto_target)

    @state(first=True, must_finish=True)
    def drive_field(self, initial_call: bool):
        x = -self.stick_x if is_red() else self.stick_x
        y = -self.stick_y if is_red() else self.stick_y
        self.drivetrain.drive_field(x, y, self.stick_o)

    @state(first=False, must_finish=True)
    def drive_auto_target(self, initial_call: bool):
        x = -self.stick_x if is_red() else self.stick_x
        y = -self.stick_y if is_red() else self.stick_y
        self.drivetrain.snap_to_heading(self.shot_calc.field_angle + math.pi)
        self.drivetrain.drive_field(x, y, 0)

    def go_drive_local(self):
        self._last_drive_mode = self.go_drive_local
        if self.current_state != self.drive_local.name:
            self.drivetrain.stop_snapping()
            self.next_state_now(self.drive_local)

    def go_drive_last_mode(self):
        self._last_drive_mode()

    @state(must_finish=True)
    def drive_local(self, initial_call: bool):
        x = self.stick_x
        y = self.stick_y
        self.drivetrain.drive_local(x, y, self.stick_o)

    def go_drive_pose(self, target_pose:Pose2d) -> None:
        self.target_pose = target_pose
        self.target_pose_pub.set(target_pose)

        if self.current_state != self.drive_to_pose.name:
            self.next_state_now(self.drive_to_pose)

    @state(must_finish=True)
    def drive_to_pose(self, initial_call: bool):
        self.drivetrain.drive_to_pose(self.target_pose)
        pose = self.drivetrain.get_pose()
        tdiff = self.target_pose.relativeTo(pose)
        dist = math.sqrt(tdiff.x**2 + tdiff.y**2)
        at_pos = dist < self.POSITION_TOLERANCE_M and abs(tdiff.rotation().degrees()) < self.ROTATION_TOLERANCE_DEG
        if at_pos:
            self._last_drive_mode()

    # JJB: We may remove this
    def go_follow_path(self, path_name:str) -> None:
        #load choreo path
        self.traj = load_swerve_trajectory(path_name)
        # JJB: Not sure about this bit, we should be letting vision and odometry
        # tell us where we are not, the trajectory.
        sample = self.traj.sample_at(0.0, is_red())
        assert sample
        sp = sample.get_pose()
        self.drivetrain.set_pose(sp)

        if self.current_state != self.follow_path.name:
            self.next_state_now(self.follow_path)

    def go_follow_traj(self, traj: SwerveTrajectory) -> None:
        self.traj = traj
        sample = self.traj.sample_at(0.0, is_red())
        assert sample
        sp = sample.get_pose()
        self.drivetrain.set_pose(sp)

        if self.current_state != self.follow_path.name:
            self.next_state_now(self.follow_path)

    @state(must_finish=True)
    def follow_path(self, initial_call: bool, state_tm: float):
        if initial_call:
            self._fired_events.clear()

        self._process_events(state_tm)

        pose = self.drivetrain.get_pose()
        end_pose = self.traj.get_final_pose(is_red())
        assert end_pose
        dist = pose.relativeTo(end_pose).translation().norm()
        angle_diff = pose.relativeTo(end_pose).rotation().degrees()
        if dist < self.POSITION_TOLERANCE_M and abs(angle_diff) < self.ROTATION_TOLERANCE_DEG:
            self._last_drive_mode()
            return
        sample = self.traj.sample_at(state_tm, is_red())
        assert sample
        self.drivetrain.follow_path(sample)
