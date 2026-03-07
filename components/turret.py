from collections import deque
from dataclasses import dataclass
from math import atan2, cos, pi, sin, sqrt, tau

import math
import wpilib
import ntcore
from magicbot import tunable, feedback
from wpimath.geometry import Pose3d, Rotation3d, Translation3d

from phoenix6.hardware import TalonFX, CANcoder
from phoenix6.controls import DutyCycleOut, PositionTorqueCurrentFOC, MotionMagicDutyCycle
from phoenix6.configs import (
    CANcoderConfiguration,
    ClosedLoopGeneralConfigs,
    CurrentLimitsConfigs,
    FeedbackConfigs,
    MotorOutputConfigs,
    Slot0Configs,
    TalonFXConfiguration,
)
from phoenix6.signals import (
    FeedbackSensorSourceValue,
    InvertedValue,
    NeutralModeValue,
    StaticFeedforwardSignValue,
    SensorDirectionValue,
)

from components.drivetrain import DrivetrainComponent
from components.gyro import GyroComponent
from utilities.game import is_red
import ids

_G = 9.81
_shooter_height = 0.15  # meters
_goal_height = 2.00  # meters
_margin_factor = 2.00

# Fixed field targets derived from AprilTag positions (k2026RebuiltWelded, meters)
_BLUE_HUB    = Translation3d(4.626, 4.035, _goal_height)
_RED_HUB     = Translation3d(11.916, 4.035, _goal_height)
_FIELD_LENGTH = 16.541
_FIELD_WIDTH  = 8.069

def clamp_angle(rads: float) -> float:
    rads = rads % tau
    if rads > pi:
        rads -= tau
    return rads

# JRD This section is deprecated given the flight calculation inside execute()
# # We'll gloss over the derivation of the physics here and just use the formulas
# # you might find in a high school physics textbook.
# def traj_calc(d: float) -> tuple[float, float, float]:
#     # Determine the height difference of our target spot and where fuel exits
#     # the shooter
#     delta_h = _goal_height - _shooter_height
#     # Now calculate the time it will take for the ball to reach the target
#     # height if launched on a trajectory where that target height was it's max.
#     # This is also the time it would take to drop from the target height to the
#     # shooter height
#     t = sqrt(2 * delta_h / _G)
#     # Now multiply the minimum flight time by something larger than 1 so that we
#     # know the fuel cell will be coming down in its flight path. The larger the
#     # value the more of a "lob" shot you'll get.
#     t *= _margin_factor
#     # Horizontal velocity to reach the goal is simply distance / time
#     vhoriz = d / t
#     # Calculate how hard we have to launch the fuel vertically so that after t
#     # seconds it's still risen up delta_h. That means we'll have to launch it
#     # above the target height and it'll be falling because t is longer than the
#     # time it would take to reach the target height.
#     vvert = (delta_h + 0.5 * _G * t**2) / t
#     return vhoriz, vvert, t


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
    # testpos = tunable(0.0)
    distance_to_goal = tunable(0.0)
    desired_angle = tunable(0.0)
    manual_speed = tunable(0.0)
    target_position = tunable(0.0)

    # Lob target offsets from corner of own alliance zone (meters, ~4 ft default)
    lob_alliance_wall_offset = tunable(1.219)  # distance inward from end wall (X axis)
    lob_side_wall_offset     = tunable(1.219)  # distance inward from side wall (Y axis)

    config_limits = tunable(False)
    stator_current_limit = tunable(20.0)
    supply_current_limit = tunable(15.0)
    supply_current_lower_limit = tunable(10.0)
    supply_current_lower_time = tunable(1.0)
    mag_offset = 0.498046875

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
        motor_config.inverted = InvertedValue.CLOCKWISE_POSITIVE
        self.turret_motor.configurator.apply(motor_config)

        closed_loop_config = ClosedLoopGeneralConfigs()
        closed_loop_config.continuous_wrap = True
        self.turret_motor.configurator.apply(closed_loop_config)

        # CANCoder + position control setup
        self.turret_encoder = CANcoder(ids.CancoderId.TURRET.id, ids.CancoderId.TURRET.bus)

        enc_config = CANcoderConfiguration()
        enc_config.magnet_sensor.with_magnet_offset(self.mag_offset)
        enc_config.magnet_sensor.with_sensor_direction(SensorDirectionValue.CLOCKWISE_POSITIVE)
        self.turret_encoder.configurator.apply(enc_config)

        feedback_config = FeedbackConfigs()
        feedback_config.feedback_remote_sensor_id = ids.CancoderId.TURRET.id
        feedback_config.feedback_sensor_source = FeedbackSensorSourceValue.FUSED_CANCODER
        feedback_config.sensor_to_mechanism_ratio = 1.0
        feedback_config.rotor_to_sensor_ratio = 90.0 
        
        turret_pid = (
            Slot0Configs()
            .with_k_p(5.0)
            .with_k_i(0.0)
            .with_k_d(0.0)
            .with_k_s(0.34)
            .with_k_v(0.122)
            .with_k_a(0.0)
            .with_static_feedforward_sign(
                StaticFeedforwardSignValue.USE_VELOCITY_SIGN  # This is important. Remember it in 2027.
            )
        )
        config = TalonFXConfiguration()
        config.motion_magic.motion_magic_cruise_velocity = 1.3  # rps
        config.motion_magic.motion_magic_acceleration = 13
        # config.motion_magic.motion_magic_jerk = math.tau

        self.turret_motor.configurator.apply(turret_pid, 0.01)
        self.turret_motor.configurator.apply(feedback_config)

        # self.position_request = PositionTorqueCurrentFOC(0).with_slot(0)
        self.motor_request = MotionMagicDutyCycle(0, override_brake_dur_neutral=True)

        self.active_target = Pose3d()


    def setup(self):
        self._apply_current_limits()
        self.set_target("hub")

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

    def set_target(self, name: str) -> None:
        """Set the active aim target. name: 'hub', 'left', or 'right'.
        Left/right are from the driver's perspective (blue faces high-X:
        left=low-Y corner, right=high-Y corner of own alliance zone).
        """
        ax = self.lob_alliance_wall_offset
        sy = self.lob_side_wall_offset
        if is_red():
            lookup = {
                "hub":   _RED_HUB,
                "left":  Translation3d(_FIELD_LENGTH - ax, sy, 0),
                "right": Translation3d(_FIELD_LENGTH - ax, _FIELD_WIDTH - sy, 0),
            }
        else:
            lookup = {
                "hub":   _BLUE_HUB,
                "left":  Translation3d(ax, _FIELD_WIDTH - sy, 0),
                "right": Translation3d(ax, sy, 0),
            }
        if name not in lookup:
            raise ValueError(f"Unknown turret target {name!r}. Valid: {list(lookup)}")
        self.active_target = Pose3d(lookup[name], Rotation3d(0, 0, 0))
        self.targets.set(self.active_target)

    @feedback
    def get_turret_position(self) -> float:
        return self.turret_encoder.get_position().value


    def execute(self) -> None:
        if self.config_limits:
            self._apply_current_limits()
            self.config_limits = False

        # Auto-tracking
        curr_pose = self.drivetrain.get_pose()

        robotvx = self.drivetrain.vx
        robotvy = self.drivetrain.vy
        
        flight_time = sqrt(2 * (_goal_height - _shooter_height) / _G) * _margin_factor
        futurex: float = self.active_target.translation().x - robotvx * flight_time
        futurey: float = self.active_target.translation().y - robotvy * flight_time

        pn = wpilib.SmartDashboard.putNumber

        
        dx = futurex - curr_pose.translation().x
        dy = futurey - curr_pose.translation().y

        field_angle = atan2(dy, dx)
        field_angle_degrees = math.degrees(field_angle)

        self.desired_angle = atan2(dy, dx) - self.gyro.get_Rotation2d().radians()

        pn('field angle', field_angle_degrees)
        pn('active target x', self.active_target.translation().x)
        pn('active target y', self.active_target.translation().y)
        pn('Turret Future X', futurex)
        pn('Turret Future Y', futurey)
        pn('Robot X', curr_pose.translation().x)
        pn('Robot Y', curr_pose.translation().y)
        pn('dx', dx)
        pn('dy', dy)
        pn('Desired Angle', math.degrees(self.desired_angle))

        self.distance_to_goal = sqrt(dx**2 + dy**2)
        goal_viz = Pose3d(
            Translation3d(futurex, futurey, self.active_target.translation().z),
            Rotation3d(0, 0, 0),
        )
        self.targets.set(goal_viz)

        # Publish visualization
        field_shot_pos = (self.desired_angle / math.tau)
        turret_viz = Pose3d(
            Translation3d(
                curr_pose.translation().x,
                curr_pose.translation().y,
                _shooter_height + 0.5
            ),
            Rotation3d(0, 0, self.desired_angle),
        )

        pn("field_shot_pos", field_shot_pos)
        self.target_position = field_shot_pos
        self.turret_motor.set_control(self.motor_request.with_position(field_shot_pos))

        # self.turret_motor.set_control(self.motor_request.with_position(self.target_position + 0.5))

        self.position.set(turret_viz)
