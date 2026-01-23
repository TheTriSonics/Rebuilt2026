from collections import deque
from dataclasses import dataclass
from math import atan2, cos, pi, sin, sqrt, tau

import ntcore
from magicbot import feedback, tunable
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from wpimath.geometry import Pose3d, Rotation3d, Translation3d

from components.drivetrain import DrivetrainComponent
from components.gyro import GyroComponent

_G = 9.81
_shooter_height = 0.15  # meters
_goal_height = 2.00  # meters
_margin_factor = 2.00

def clamp_angle(rads: float) -> float:
    rads = rads % tau
    if rads > pi:
        rads -= tau
    return rads

def traj_calc(d: float) -> tuple[float, float, float]:
    delta_h = _goal_height - _shooter_height
    t = sqrt(2 * delta_h / _G) * _margin_factor
    vhoriz = d / t
    vvert = (delta_h + 0.5 * _G * t**2) / t
    return vhoriz, vvert, t


@dataclass
class BallProperties:
    xpos: float
    ypos: float
    zpos: float
    xvel: float
    yvel: float
    zvel: float


class TurretComponent:
    gyro: GyroComponent
    drivetrain: DrivetrainComponent

    distance_to_goal = tunable(0.0)
    desired_angle = tunable(0.0)
    measured_angle = tunable(0.0)

    apriltags = AprilTagFieldLayout.loadField(AprilTagField.k2026RebuiltWelded)

    def __init__(self):
        self.balls: deque[BallProperties] = deque(maxlen=20)
        self.targets = (
            ntcore.NetworkTableInstance.getDefault()
            .getStructArrayTopic('/components/turret/targets', Pose3d)
            .publish()
        )
        tag20: Pose3d = self.apriltags.getTagPose(20) or Pose3d()
        tag26: Pose3d = self.apriltags.getTagPose(26) or Pose3d()
        self.static_goal_center = Pose3d(
            Translation3d((tag20.x + tag26.x)/2, tag20.y, tag20.z),
            Rotation3d(0, 0, 0),
        )
        t = [self.static_goal_center ]
        self.targets.set(t)

    @feedback
    def at_position(self) -> bool:
        # If we're within half a degree say we're at position
        return abs(self.desired_angle - self.measured_angle) < 0.5

    def shoot_fuel(self) -> None:
        current_pose = self.drivetrain.get_pose()
        field_shot_angle = self.measured_angle - self.gyro.get_Rotation2d().radians()
        distance_to_goal = self.distance_to_goal * 0.90
        vhoriz, vz, _ = traj_calc(distance_to_goal)
        xvel = vhoriz * cos(field_shot_angle)
        yvel = vhoriz * sin(field_shot_angle)
        self.fuel_launch_vel = vhoriz
        self.fuel_launch_zvel = vz
        xvel += self.drivetrain.vx
        yvel += self.drivetrain.vy
        self.balls.append(
            BallProperties(
                xpos=current_pose.x,
                ypos=current_pose.y,
                zpos=_shooter_height,
                xvel=xvel,
                yvel=yvel,
                zvel=vz)
        )

    def execute(self) -> None:
        curr_pose = self.drivetrain.get_pose()
        robotvx = self.drivetrain.vx
        robotvy = self.drivetrain.vy

        t = sqrt(2 * (_goal_height - _shooter_height) / _G) * _margin_factor
        futurex = self.static_goal_center.translation().x - robotvx * t
        futurey = self.static_goal_center.translation().y - robotvy * t
        # Calculate the angle to the goal from the current pose
        dx = futurex - curr_pose.translation().x
        dy = futurey - curr_pose.translation().y
        self.desired_angle = clamp_angle(
            atan2(dy, dx) + self.gyro.get_Rotation2d().radians()
        )
        self.distance_to_goal = sqrt(dx**2 + dy**2)
        goal_viz = Pose3d(
            Translation3d(futurex, futurey, self.static_goal_center.translation().z),
            Rotation3d(0, 0, 0),
        )
        self.targets.set([goal_viz])

        # Just hack this for now, later this is where we'd want to drive the
        # motor to the proper setpoint.
        self.measured_angle = self.desired_angle
