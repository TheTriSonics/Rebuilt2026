import ids

from phoenix6.hardware import TalonFX
from phoenix6.controls import VelocityVoltage
from phoenix6.configs import CurrentLimitsConfigs, MotorOutputConfigs, Slot0Configs, FeedbackConfigs
from phoenix6.signals import InvertedValue, NeutralModeValue, StaticFeedforwardSignValue
from components.shooter import ShooterComponent
from magicbot import tunable


class KickerComponent:
    shooter: ShooterComponent

    kicker = TalonFX(ids.TalonId.KICKER.id, ids.TalonId.KICKER.bus)

    kicker_speed = tunable(32.0)

    config_limits = tunable(False)
    stator_current_limit = tunable(120.0)
    supply_current_limit = tunable(120.0)
    supply_current_lower_limit = tunable(120.0)
    supply_current_lower_time = tunable(0.0)

    def __init__(self):
        motor_pid = (
            Slot0Configs()
            .with_k_p(0.5)
            .with_k_i(0.0)
            .with_k_d(0.0)
            .with_k_s(0.25)
            .with_k_v(0.12)
            .with_k_a(0.0)
            .with_static_feedforward_sign(
                StaticFeedforwardSignValue.USE_CLOSED_LOOP_SIGN
            )
        )
        feedback_config = FeedbackConfigs()
        feedback_config.sensor_to_mechanism_ratio = 1.0

        motor_config = MotorOutputConfigs()
        motor_config.neutral_mode = NeutralModeValue.COAST
        motor_config.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE

        self.kicker.configurator.apply(feedback_config)
        self.kicker.configurator.apply(motor_pid, 0.01)
        self.kicker.configurator.apply(motor_config)

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
        self.kicker.configurator.apply(current_limits_config)

    def is_active(self) -> bool:
        return self.target_speed != 0.0

    def is_reverse(self) -> bool:
        return self.target_speed < 0.0

    def is_off(self) -> bool:
        return self.target_speed == 0.0

    def execute(self) -> None:
        if self.config_limits:
            self._apply_current_limits()
            self.config_limits = False
        if self.shooter.is_active():
            self.target_speed = self.kicker_speed
        else:
            self.target_speed = 0.0
        self.kicker.set_control(VelocityVoltage(-self.target_speed))
