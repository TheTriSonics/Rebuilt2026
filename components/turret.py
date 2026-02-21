from collections import deque
from dataclasses import dataclass
from math import atan2, cos, pi, sin, sqrt, tau

import ntcore
from magicbot import tunable
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from wpimath.geometry import Pose3d, Rotation3d, Translation3d

from components.drivetrain import DrivetrainComponent
from components.gyro import GyroComponent
from utilities.game import is_red

_G = 9.81
_shooter_height = 0.15  # meters
_goal_height = 2.00  # meters
_margin_factor = 2.00

def clamp_angle(rads: float) -> float:
    rads = rads % tau
    if rads > pi:
        rads -= tau
    return rads

# We'll gloss over the derivation of the physics here and just use the formulas
# you might find in a high school physics textbook.
def traj_calc(d: float) -> tuple[float, float, float]:
    # Determine the height difference of our target spot and where fuel exits
    # the shooter
    delta_h = _goal_height - _shooter_height
    # Now calculate the time it will take for the ball to reach the target
    # height if launched on a trajectory where that target height was it's max.
    # This is also the time it would take to drop from the target height to the
    # shooter height
    t = sqrt(2 * delta_h / _G)
    # Now multiply the minimum flight time by something larger than 1 so that we
    # know the fuel cell will be coming down in its flight path. The larger the
    # value the more of a "lob" shot you'll get.
    t *= _margin_factor
    # Horizontal velocity to reach the goal is simply distance / time
    vhoriz = d / t
    # Calculate how hard we have to launch the fuel vertically so that after t
    # seconds it's still risen up delta_h. That means we'll have to launch it
    # above the target height and it'll be falling because t is longer than the
    # time it would take to reach the target height.
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

    # Shooter PIDs 
    # Shooter front: kP 4.0, kS 2.15, kV 0.14 
    # Shooter Rear: kP 4.5, kS 2, kV 0.11

    # These are set to tunables just so they show up on the dashboard for now
    distance_to_goal = tunable(0.0)
    desired_angle = tunable(0.0)
    measured_angle = tunable(0.0)

    apriltags = AprilTagFieldLayout.loadField(AprilTagField.k2026RebuiltWelded)

    def __init__(self):
        # We'll keep track of 20 ball objects and let the physics sim handle
        # their motion. This doesn't matter to a real robot at all.
        self.balls: deque[BallProperties] = deque(maxlen=20)
        # We'll use this to debug targetting. Right now it shows where the
        # robot's turret should be aiming at if it were to launch a fuel cell
        self.targets = (
            ntcore.NetworkTableInstance.getDefault()
            .getStructArrayTopic('/components/turret/targets', Pose3d)
            .publish()
        )

        self.position = (
            ntcore.NetworkTableInstance.getDefault()
            .getStructTopic('/components/turret/position', Pose3d)
            .publish()
        )
        self.set_hub_target()

    def set_hub_target(self) -> None:
        # We calculate the center of the goal based on the positions of
        # AprilTags 20 and 26, then get a point right between them. That's
        # basically dead center of the goal
        tag1 = 10 if is_red() else 20
        tag2 = 4 if is_red() else 26
        tag20: Pose3d = self.apriltags.getTagPose(tag1) or Pose3d()
        tag26: Pose3d = self.apriltags.getTagPose(tag2) or Pose3d()
        self.static_goal_center = Pose3d(
            Translation3d((tag20.x + tag26.x)/2, tag20.y, tag20.z),
            Rotation3d(0, 0, 0),
        )
        t = [self.static_goal_center ]
        self.targets.set(t)

    def shoot_fuel(self) -> None:
        # This method might turn on some motors on a real robot but for now
        # we're just going to calculate what we would want the fuel to do upon
        # ejection and then let the physics sim handle it.
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

        # Calculate flight time of fuel cell to goal
        t = sqrt(2 * (_goal_height - _shooter_height) / _G) * _margin_factor
        # Figure out where to move the goal relative to its actual position so
        # that the chassis' velocity is accounted for when aiming at the goal
        futurex: float = self.static_goal_center.translation().x - robotvx * t
        futurey: float = self.static_goal_center.translation().y - robotvy * t

        # Calculate the angle to the goal's center from the current pose
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

        # Now publishing a Pose3D so AdvanrtageScope can draw a cone in the
        # direction we're aiming.
        field_shot_angle = self.measured_angle - self.gyro.get_Rotation2d().radians()
        turret_viz = Pose3d(
            Translation3d(
                curr_pose.translation().x,
                curr_pose.translation().y,
                _shooter_height + 0.5
            ),
            Rotation3d(0, 0, field_shot_angle),
        )
        self.position.set(turret_viz)
