from magicbot import tunable, feedback
from phoenix6.hardware import TalonFX
from phoenix6.controls import DutyCycleOut
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
    ClosedLoopGeneralConfigs,
    CurrentLimitsConfigs,
    FeedbackConfigs,
    MotorOutputConfigs,
    Slot0Configs,
)



class IntakeComponent:

    # These are set to tunables just so they show up on the dashboard for now
    upper_position = tunable(0.70)
    lower_position = tunable(0.10)
    target_position = tunable(0.3)

    intake_speed = tunable(0.7)
    outtake_speed = tunable(-0.7)
    target_speed = tunable(0.5)

    config_limits = tunable(False)
    stator_current_limit = tunable(1.0)
    supply_current_limit = tunable(2.0)
    supply_current_lower_limit = tunable(2.0)
    supply_current_lower_time = tunable(1.0)

    rotate = TalonFX(ids.TalonId.ROTATE.id, ids.TalonId.ROTATE.bus)
    roller = TalonFX(ids.TalonId.ROLLER.id, ids.TalonId.ROLLER.bus)
    rotate_encoder = CANcoder(ids.CancoderId.INTAKE.id, ids.CancoderId.INTAKE.bus)
    rotate_request = PositionVoltage(0).with_slot(0)

    def __init__(self):
        motor_config = MotorOutputConfigs()
        motor_config.neutral_mode = NeutralModeValue.BRAKE
        motor_config.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE

        self.mag_offset = -0.21728515625
        enc_config = CANcoderConfiguration()
        enc_config.magnet_sensor.with_magnet_offset(self.mag_offset)
        enc_config.magnet_sensor.with_sensor_direction(SensorDirectionValue.CLOCKWISE_POSITIVE)
        self.rotate_encoder.configurator.apply(enc_config)


        feedback_config = FeedbackConfigs()
        feedback_config.feedback_remote_sensor_id = ids.CancoderId.INTAKE.id
        feedback_config.feedback_sensor_source = FeedbackSensorSourceValue.FUSED_CANCODER
        feedback_config.sensor_to_mechanism_ratio = 1.0
        feedback_config.rotor_to_sensor_ratio = 135.0

        pid = (
            Slot0Configs()
            .with_k_p(18.0)
            .with_k_i(2.0)
            .with_k_d(0.0)
            .with_k_s(0.3)
            .with_k_v(1.8)
            .with_k_a(0)
            .with_static_feedforward_sign(
                StaticFeedforwardSignValue.USE_CLOSED_LOOP_SIGN
            )
        )
        closed_loop_config = ClosedLoopGeneralConfigs()
        self.rotate.configurator.apply(motor_config)
        self.rotate.configurator.apply(pid, 2.0)
        self.rotate.configurator.apply(feedback_config)
        self.rotate.configurator.apply(closed_loop_config)

    def setup(self):
        self._apply_current_limits()

    def _apply_current_limits(self):
        current_limits_config = (
            CurrentLimitsConfigs()
            .with_stator_current_limit(self.stator_current_limit)
            .with_stator_current_limit_enable(False)
            .with_supply_current_limit(self.supply_current_limit)
            .with_supply_current_limit_enable(False)
            .with_supply_current_lower_limit(self.supply_current_lower_limit)
            .with_supply_current_lower_time(self.supply_current_lower_time)
        )
        self.roller.configurator.apply(current_limits_config)


    def rotate_down(self) -> None:
        # Move intake to the configured lowered setpoint.
        self.target_position = self.lower_position
        
    def rotate_up(self) -> None:
        # Move intake to the configured raised setpoint.
        self.target_position = self.upper_position

    def set_speed(self, speed: float) -> None:
        self.target_speed = speed

    def intake_on(self) -> None:
        self.target_position = self.lower_position
        self.set_speed(self.intake_speed)
    
    def tilt(self) -> None:
        self.target_position = self.lower_position + 0.4

    def intake_off(self) -> None:
        self.target_position = self.upper_position
        self.set_speed(0)

    def intake_reverse(self) -> None:
        self.set_speed(self.outtake_speed)

    @feedback
    def get_rotate_position(self) -> float:
        return self.rotate_encoder.get_position().value

    def execute(self) -> None:
        if self.config_limits:
            self._apply_current_limits()
            self.config_limits = False
        if self.target_position > self.upper_position:
            self.target_position = self.upper_position
        elif self.target_position < self.lower_position:
            self.target_position = self.lower_position

        self.rotate.set_control(self.rotate_request.with_position(self.target_position))
        
        self.roller.set_control(DutyCycleOut(self.target_speed))
