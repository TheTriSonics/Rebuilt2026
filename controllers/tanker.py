import math
import ntcore
from wpimath.geometry import Pose2d
from magicbot import StateMachine, state

from components.drivetrain import DrivetrainComponent
from utilities.game import is_red
from choreo import load_swerve_trajectory
from choreo.trajectory import SwerveTrajectory

from components.turret import clamp_angle

class Tanker(StateMachine):
    # Position tolerance thresholds
    POSITION_TOLERANCE_M = 0.02
    ROTATION_TOLERANCE_DEG = 1.0

    drivetrain: DrivetrainComponent

    stick_x, stick_y, stick_o = 0, 0, 0
    alliance_loaded = None


    def __init__(self):
        super().__init__()
        self.target_pose: Pose2d = Pose2d()
        self.target_pose_pub = (
            ntcore.NetworkTableInstance
            .getDefault()
            .getStructTopic("/components/tanker/target_pose", Pose2d)
            .publish()
        )
        self.trajectory: SwerveTrajectory | None = None

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
        if self.current_state != self.drive_field.name:
            self.next_state_now(self.drive_field)

    @state(first=True, must_finish=True)
    def drive_field(self, initial_call: bool):
        if initial_call:
            # Clear out any trajectory we're runnning
            ...
        x = -self.stick_x if is_red() else self.stick_x
        y = -self.stick_y if is_red() else self.stick_y
        self.drivetrain.drive_field(x, y, self.stick_o)

    def go_drive_local(self):
        if self.current_state != self.drive_local.name:
            self.next_state_now(self.drive_local)

    @state(must_finish=True)
    def drive_local(self, initial_call: bool):
        if initial_call:
            # Clear out any trajectory we're runnning
            ...
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
            self.go_drive_field()


    def go_follow_trajectory(self, name: str) -> None:
        self.trajectory = load_swerve_trajectory(name)
        first_sample = self.trajectory.sample_at(0.0)
        if first_sample:
            first_pose = first_sample.get_pose()
            self.drivetrain.set_pose(first_pose)
        if self.current_state != self.follow_trajectory.name:
            self.next_state_now(self.follow_trajectory)

    @state(must_finish=True)
    def follow_trajectory(self, initial_call: bool, state_tm: float):
        assert self.trajectory
        sample = self.trajectory.sample_at(state_tm, is_red())
        assert sample
        pose = self.drivetrain.get_pose()
        final_pose = self.trajectory.get_final_pose(is_red())
        assert final_pose
        pdiff = final_pose.relativeTo(pose).translation().norm()
        angle_diff = abs(clamp_angle(final_pose.rotation().radians() - pose.rotation().radians()))
        at_pos = pdiff < self.POSITION_TOLERANCE_M and angle_diff < math.radians(self.ROTATION_TOLERANCE_DEG)
        if at_pos:
            self.go_drive_field()
        if sample:
            self.drivetrain.follow_trajectory(sample)
        else:
            self.go_drive_field()

