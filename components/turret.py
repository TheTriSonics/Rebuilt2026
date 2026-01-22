from math import atan2, pi, tau, sqrt, sin, cos
import wpilib
import ntcore

from wpimath.geometry import Pose3d, Translation3d, Rotation3d

from magicbot import feedback, tunable
from components.drivetrain import DrivetrainComponent
from components.gyro import GyroComponent

from robotpy_apriltag import AprilTagFieldLayout, AprilTagField


def clamp_angle(rads: float) -> float:
    rads = rads % tau
    if rads > pi:
        rads -= tau
    return rads

def traj_calc(d: float, margin_factor: float = 2.0, 
              launch_height: float = 0.15, goal_height: float = 2.0,
              g: float = 9.81) -> dict:
    delta_h = goal_height - launch_height
    
    t_min = sqrt(2 * delta_h / g)
    t = t_min * margin_factor  # slightly longer flight
    
    # Now solve for v and θ with this fixed t:
    # d = v*cos(θ)*t
    # delta_h = v*sin(θ)*t - 0.5*g*t²
    #
    # Let vx = v*cos(θ) = d/t
    # Let vy = v*sin(θ) = (delta_h + 0.5*g*t²) / t
    
    vx = d / t
    vy = (delta_h + 0.5 * g * t**2) / t
    
    v = sqrt(vx**2 + vy**2)
    theta = atan2(vy, vx)
    vx = cos(theta) * v
    vz = sin(theta) * v
    return vx, vz, t


class TurretComponent:
    gyro: GyroComponent
    drivetrain: DrivetrainComponent

    distance_to_goal = tunable(0.0)
    desired_angle = tunable(0.0)
    measured_angle = tunable(0.0)

    apriltags = AprilTagFieldLayout.loadField(AprilTagField.k2026RebuiltWelded)

    def __init__(self):
        self.angle = 0.0
        self.targets = (
            ntcore.NetworkTableInstance.getDefault()
            .getStructArrayTopic('/components/turret/targets', Pose3d)
            .publish()
        )
        tag20 = self.apriltags.getTagPose(20)
        tag26 = self.apriltags.getTagPose(26)
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

    def set_angle(self, angle: float):
        self.desired_angle = angle
    
    def execute(self) -> None:
        curr_pose = self.drivetrain.get_pose()
        robotvx = self.drivetrain.vx
        robotvy = self.drivetrain.vy

        t = sqrt(2 * (2.0 - 0.15) / 9.81) * 2.0
        futurex = self.static_goal_center.translation().x - robotvx * t
        futurey = self.static_goal_center.translation().y - robotvy * t
        # Calculate the angle to the goal from the current pose
        dx = futurex - curr_pose.translation().x
        dy = futurey - curr_pose.translation().y
        self.desired_angle = clamp_angle(
            atan2(dy, dx) + self.gyro.get_Rotation2d().radians()
        )
        self.distance_to_goal = sqrt(dx**2 + dy**2)
        # Just hack this for now
        self.measured_angle = self.desired_angle
        goal_viz = Pose3d(
            Translation3d(futurex, futurey, self.static_goal_center.translation().z),
            Rotation3d(0, 0, 0),
        )
        self.targets.set([goal_viz])