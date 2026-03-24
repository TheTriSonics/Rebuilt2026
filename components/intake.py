from magicbot import tunable, feedback
from phoenix6.hardware import CANcoder, TalonFX
from phoenix6.controls import DutyCycleOut, PositionVoltage
import ids

from phoenix6.signals import (
    FeedbackSensorSourceValue,
    InvertedValue,
    NeutralModeValue,
    StaticFeedforwardSignValue,
    SensorDirectionValue,
)
from phoenix6.configs import (
    CANcoderConfiguration,
    CurrentLimitsConfigs,
    FeedbackConfigs,
    MotorOutputConfigs,
    Slot0Configs,
)


class IntakeComponent:

    upper_position = 0.30
    tilt_position = 0.20
    lower_position = -0.015
    target_position = tunable(0.0)

    intake_speed = tunable(-0.45)
    outtake_speed = tunable(0.40)
    target_speed = tunable(0.0)

    config_limits = tunable(False)
    stator_current_limit = tunable(60)
    supply_current_limit = tunable(120)
    supply_current_lower_limit = tunable(0)
    supply_current_lower_time = tunable(0.0)

    rotate = TalonFX(ids.TalonId.ROTATE.id, ids.TalonId.ROTATE.bus)
    roller = TalonFX(ids.TalonId.ROLLER.id, ids.TalonId.ROLLER.bus)
    rotate_encoder = CANcoder(ids.CancoderId.INTAKE.id, ids.CancoderId.INTAKE.bus)
    rotate_request = PositionVoltage(0).with_slot(0)

    def __init__(self):
        motor_config = MotorOutputConfigs()
        motor_config.neutral_mode = NeutralModeValue.BRAKE
        motor_config.inverted = InvertedValue.CLOCKWISE_POSITIVE

        self.mag_offset = -0.42578125 
        enc_config = CANcoderConfiguration()
        enc_config.magnet_sensor.with_magnet_offset(self.mag_offset)
        enc_config.magnet_sensor.with_sensor_direction(SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE)
        self.rotate_encoder.configurator.apply(enc_config)

        feedback_config = FeedbackConfigs()
        feedback_config.feedback_remote_sensor_id = ids.CancoderId.INTAKE.id
        feedback_config.feedback_sensor_source = FeedbackSensorSourceValue.FUSED_CANCODER
        feedback_config.sensor_to_mechanism_ratio = 1.0
        feedback_config.rotor_to_sensor_ratio = 49.846

        pid = (
            Slot0Configs()
            .with_k_p(60.0)
            .with_k_i(0.0)
            .with_k_d(15.0)
            .with_k_s(0.5)
            .with_k_v(2.0)
            .with_k_a(0)
            .with_static_feedforward_sign(
                StaticFeedforwardSignValue.USE_CLOSED_LOOP_SIGN
            )
        )
        self.rotate.configurator.apply(motor_config)
        self.rotate.configurator.apply(pid, 2.0)
        self.rotate.configurator.apply(feedback_config)

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
        self.rotate.configurator.apply(current_limits_config)

    def rotate_down(self) -> None:
        self.target_position = self.lower_position

    def rotate_tilt(self) -> None:
        self.target_position = self.tilt_position

    def rotate_up(self) -> None:
        self.target_position = self.upper_position

    def set_speed(self, speed: float) -> None:
        self.target_speed = speed

    def on(self) -> None:
        self.set_speed(self.intake_speed)

    def off(self) -> None:
        self.set_speed(0)

    def reverse(self) -> None:
        self.set_speed(self.outtake_speed)

    @feedback
    def get_rotate_position(self) -> float:
        return self.rotate_encoder.get_position().value

    @feedback
    def rotate_motor_temp(self) -> float:
        return self.rotate.get_device_temp().value

    @feedback
    def roller_motor_temp(self) -> float:
        return self.roller.get_device_temp().value

    def execute(self) -> None:
        if self.config_limits:
            self._apply_current_limits()
            self.config_limits = False
        if self.target_position > self.upper_position:
            self.target_position = self.upper_position
        elif self.target_position < self.lower_position:
            self.target_position = self.lower_position

        self.rotate.set_control(self.rotate_request.with_position(self.target_position))
        # print(f'INtake running at {self.target_speed}')
        self.roller.set_control(DutyCycleOut(self.target_speed))
