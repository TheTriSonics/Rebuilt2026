import math
import ntcore
from magicbot import tunable, feedback
from wpimath.geometry import Pose3d, Rotation3d, Translation3d, Pose2d, Rotation2d, Translation2d, Transform2d

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

from components.gyro import GyroComponent
from components.shot_calculator import ShotCalculatorComponent
import ids

_shooter_height = 0.15  # meters


class TurretComponent:
    gyro: GyroComponent
    shot_calc: ShotCalculatorComponent

    config_limits = tunable(False)
    stator_current_limit = tunable(20.0)
    supply_current_limit = tunable(15.0)
    supply_current_lower_limit = tunable(10.0)
    supply_current_lower_time = tunable(1.0)
    mag_offset = 0.498046875

    # Turret motor — always created
    turret_motor = TalonFX(ids.TalonId.TURRET_TURN.id, ids.TalonId.TURRET_TURN.bus)

    def __init__(self):
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

        self.turret_motor.configurator.apply(config)
        self.turret_motor.configurator.apply(turret_pid, 0.01)
        self.turret_motor.configurator.apply(feedback_config)

        self.motor_request = MotionMagicDutyCycle(0, override_brake_dur_neutral=True)

        self.position = (
            ntcore.NetworkTableInstance.getDefault()
            .getStructTopic('/components/turret/position', Pose3d)
            .publish()
        )
        self.desired_turret_pos = 0.0


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


    @feedback
    def get_turret_pos_actual(self) -> float:
        return self.turret_encoder.get_position().value

    @feedback
    def get_turret_pos_desired(self) -> float:
        return self.desired_turret_pos

    def execute(self) -> None:
        if self.config_limits:
            self._apply_current_limits()
            self.config_limits = False

        self.field_angle = self.shot_calc.field_angle
        self.desired_turret_angle_relative_to_robot = (
            self.field_angle - math.radians(self.gyro.get_heading())
        )

        self.desired_turret_pos = (self.desired_turret_angle_relative_to_robot / math.tau)
        # TODO: Turn this off if an FMS is connected. Debug only.
        turret_viz = Pose3d(
            Translation3d(
                self.shot_calc.shooter_pose.translation().x,
                self.shot_calc.shooter_pose.translation().y,
                _shooter_height + 0.5
            ),
            Rotation3d(0, 0, self.field_angle),
        )
        self.position.set(turret_viz)

        # self.turret_motor.set_control(self.motor_request.with_position(0.5))
        self.turret_motor.set_control(self.motor_request.with_position(self.desired_turret_pos))
