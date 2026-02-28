from collections import deque
from dataclasses import dataclass
from math import atan2, cos, pi, sin, sqrt, tau

import math
import wpilib
import ntcore
from magicbot import tunable
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from wpimath.geometry import Pose2d, Pose3d, Rotation3d, Translation3d

from phoenix6.hardware import TalonFX, CANcoder
from phoenix6.controls import DutyCycleOut, PositionTorqueCurrentFOC
from phoenix6.configs import (
    CANcoderConfiguration,
    ClosedLoopGeneralConfigs,
    CurrentLimitsConfigs,
    FeedbackConfigs,
    MotorOutputConfigs,
    Slot0Configs,
)
from phoenix6.signals import (
    FeedbackSensorSourceValue,
    InvertedValue,
    NeutralModeValue,
    StaticFeedforwardSignValue,
)

from components.drivetrain import DrivetrainComponent
from components.gyro import GyroComponent
from utilities.game import is_red
import ids

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
    measured_angle = 0.0
    manual_speed = tunable(0.0)
    target_position = tunable(0.0)

    config_limits = tunable(False)
    stator_current_limit = tunable(20.0)
    supply_current_limit = tunable(15.0)
    supply_current_lower_limit = tunable(10.0)
    supply_current_lower_time = tunable(1.0)

    apriltags = AprilTagFieldLayout.loadField(AprilTagField.k2026RebuiltWelded)

    # Turret motor — always created
    turret_motor = TalonFX(ids.TalonId.TURRET_TURN.id, ids.TalonId.TURRET_TURN.bus)

    def __init__(self):
        # We'll use this to debug targetting. Right now it shows where the
        # robot's turret should be aiming at if it were to launch a fuel cell
        self.targets = (
            ntcore.NetworkTableInstance.getDefault()
            .getStructTopic('/components/turret/fuel_target', Pose3d)
            .publish()
        )

        self.position = (
            ntcore.NetworkTableInstance.getDefault()
            .getStructTopic('/components/turret/position', Pose3d)
            .publish()
        )

        # Configure motor output
        motor_config = MotorOutputConfigs()
        motor_config.neutral_mode = NeutralModeValue.BRAKE
        motor_config.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        self.turret_motor.configurator.apply(motor_config)

        # CANCoder + position control setup
        self.turret_encoder = CANcoder(ids.CancoderId.TURRET.id, ids.CancoderId.TURRET.bus)

        enc_config = CANcoderConfiguration()
        self.turret_encoder.configurator.apply(enc_config)

        feedback_config = FeedbackConfigs()
        feedback_config.feedback_remote_sensor_id = ids.CancoderId.TURRET.id
        feedback_config.feedback_sensor_source = FeedbackSensorSourceValue.FUSED_CANCODER
        feedback_config.sensor_to_mechanism_ratio = 1.0
        feedback_config.rotor_to_sensor_ratio = 90.0 
        
        turret_pid = (
            Slot0Configs()
            .with_k_p(15.0)
            .with_k_i(0.0)
            .with_k_d(0.0)
            .with_k_s(1.0)
            .with_k_v(0.0)
            .with_k_a(0.0)
            .with_static_feedforward_sign(
                StaticFeedforwardSignValue.USE_CLOSED_LOOP_SIGN
            )
        )

        closed_loop_config = ClosedLoopGeneralConfigs()
        closed_loop_config.continuous_wrap = True

        self.turret_motor.configurator.apply(turret_pid, 0.01)
        self.turret_motor.configurator.apply(feedback_config)
        self.turret_motor.configurator.apply(closed_loop_config)

        self.position_request = PositionTorqueCurrentFOC(0).with_slot(0)

        self.set_hub_target()


    def setup(self):
        self._apply_current_limits()

    def _apply_current_limits(self):
        current_limits_config = (
            CurrentLimitsConfigs()
            .with_stator_current_limit(self.stator_current_limit)
            .with_stator_current_limit_enable(True)
            .with_supply_current_limit(self.supply_current_limit)
            .with_supply_current_limit_enable(True)
            .with_supply_current_lower_limit(self.supply_current_lower_limit)
            .with_supply_current_lower_time(self.supply_current_lower_time)
        )
        self.turret_motor.configurator.apply(current_limits_config)

    def set_manual_speed(self, speed: float) -> None:
        self.manual_speed = speed

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
        self.targets.set(self.static_goal_center)
    
    def set_lob_target(self, robot_pose: Pose2d) -> None:
        targetx = self.apriltags.getFieldLength() - 2.0 if is_red() else 2.0
        roboty = robot_pose.translation().y
        half_width = self.apriltags.getFieldWidth() / 2.0
        if roboty < half_width:
            targety = 2.0
        else:
            targety = self.apriltags.getFieldWidth() - 2.0
        self.static_goal_center = Pose3d(
            Translation3d(targetx, targety, _goal_height),
            Rotation3d(0, 0, 0),
        )


    def execute(self) -> None:
        if self.config_limits:
            self._apply_current_limits()
            self.config_limits = False

        # self.turret_motor.set_control(self.position_request.with_position(self.target_position))

        # Auto-tracking
        curr_pose = self.drivetrain.get_pose()
        wall_tag = 16 if is_red() else 32
        wall_pose: Pose3d = self.apriltags.getTagPose(wall_tag) or Pose3d()
        curr_pose = self.drivetrain.get_pose()
        if abs(curr_pose.translation().x - wall_pose.translation().x) > 4.0:
            self.set_lob_target(curr_pose)
        else:
            self.set_hub_target()
        

        robotvx = self.drivetrain.vx
        robotvy = self.drivetrain.vy
        
        t = sqrt(2 * (_goal_height - _shooter_height) / _G) * _margin_factor
        futurex: float = self.static_goal_center.translation().x - robotvx * t
        futurey: float = self.static_goal_center.translation().y - robotvy * t
        
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
        self.targets.set(goal_viz)

        # Publish visualization
        field_shot_angle = self.measured_angle - self.gyro.get_Rotation2d().radians()
        turret_viz = Pose3d(
            Translation3d(
                self.drivetrain.get_pose().translation().x,
                self.drivetrain.get_pose().translation().y,
                _shooter_height + 0.5
            ),
            Rotation3d(0, 0, field_shot_angle),
        )

        self.turret_motor.set_control(self.position_request.with_position(field_shot_angle))

        self.position.set(turret_viz)
