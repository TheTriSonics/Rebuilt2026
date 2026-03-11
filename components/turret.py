from math import atan2, sqrt

import math
import wpilib
import ntcore
from magicbot import tunable, feedback
from wpimath.geometry import Pose3d, Rotation3d, Translation3d

from phoenix6.hardware import TalonFX, CANcoder
from phoenix6.controls import MotionMagicDutyCycle
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

_shooter_height = 0.15  # meters
_goal_height = 2.00  # meters

# Fixed field targets derived from AprilTag positions (k2026RebuiltWelded, meters)
_BLUE_HUB    = Translation3d(4.626, 4.035, _goal_height)
_RED_HUB     = Translation3d(11.916, 4.035, _goal_height)
_FIELD_LENGTH = 16.541
_FIELD_WIDTH  = 8.069

class TurretComponent:
    gyro: GyroComponent
    drivetrain: DrivetrainComponent

    distance_to_goal = 0.0
    desired_angle = 0.0
    flight_time = tunable(1.0)
    turret_position = tunable(0.0)

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
        self.goal_pose = Pose3d()
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
        feedback_config.rotor_to_sensor_ratio = 18.0
        
        turret_pid = (
            Slot0Configs()
            .with_k_p(3.0)
            .with_k_i(0.0)
            .with_k_d(0.2)
            .with_k_s(0.05)
            .with_k_v(0.0)
            .with_k_a(0.0)
            .with_static_feedforward_sign(
                StaticFeedforwardSignValue.USE_VELOCITY_SIGN  # This is important. Remember it in 2027.
            )
        )
        config = TalonFXConfiguration()
        config.motion_magic.motion_magic_cruise_velocity = 6.5  # rps
        config.motion_magic.motion_magic_acceleration = 65

        self.turret_motor.configurator.apply(turret_pid, 0.01)
        self.turret_motor.configurator.apply(feedback_config)

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
        
        self.flight_time = 0.4 / 2.0
        robotvx = self.drivetrain.vx
        robotvy = self.drivetrain.vy
        self.futurex: float = self.active_target.translation().x - robotvx * self.flight_time
        self.futurey: float = self.active_target.translation().y - robotvy * self.flight_time

        dx = self.futurex - curr_pose.translation().x
        dy = self.futurey - curr_pose.translation().y

        self.desired_angle = atan2(dy, dx) - math.radians(self.gyro.get_heading())

        self.distance_to_goal = sqrt(dx**2 + dy**2)
        self.goal_pose = Pose3d(
            Translation3d(self.futurex, self.futurey, self.active_target.translation().z),
            Rotation3d(0, 0, 0),
        )
        self.targets.set(self.goal_pose)

        field_shot_pos = (self.desired_angle / math.tau)
        turret_viz = Pose3d(
            Translation3d(
                curr_pose.translation().x,
                curr_pose.translation().y,
                _shooter_height + 0.5
            ),
            Rotation3d(0, 0, self.desired_angle),
        )

        wpilib.SmartDashboard.putNumber("field_shot_pos", field_shot_pos)
        self.turret_motor.set_control(self.motor_request.with_position(0.5))
        # self.turret_motor.set_control(self.motor_request.with_position(self.turret_position))

        self.position.set(turret_viz)
