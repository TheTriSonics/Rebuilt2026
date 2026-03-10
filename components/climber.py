from magicbot import tunable
from phoenix6.hardware import TalonFX
from phoenix6.controls import PositionVoltage
import ids

from phoenix6.signals import (
    InvertedValue,
    NeutralModeValue,
    StaticFeedforwardSignValue,
)
from phoenix6.configs import (
    CurrentLimitsConfigs,
    FeedbackConfigs,
    MotorOutputConfigs,
    Slot0Configs,
)


class ClimberComponent:

    upper_limit = tunable(100.0)
    lower_limit = tunable(0.0)
    upper_position = tunable(95.0)
    home_position = 0.0
    target_position = tunable(95.0)

    config_limits = tunable(False)
    stator_current_limit = tunable(1.0)
    supply_current_limit = tunable(10.0)
    supply_current_lower_limit = tunable(5.0)
    supply_current_lower_time = tunable(1.0)

    climber = TalonFX(ids.TalonId.CLIMBER.id, ids.TalonId.CLIMBER.bus)
    position_request = PositionVoltage(0).with_slot(0)

    def __init__(self):
        motor_config = MotorOutputConfigs()
        motor_config.neutral_mode = NeutralModeValue.BRAKE
        motor_config.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE

        feedback_config = FeedbackConfigs()
        feedback_config.sensor_to_mechanism_ratio = 1.0
        feedback_config.rotor_to_sensor_ratio = 1.0

        pid = (
            Slot0Configs()
            .with_k_p(0.1)
            .with_k_i(0)
            .with_k_d(0.0)
            .with_k_s(4.3)
            .with_k_v(2.0)
            .with_k_a(0)
            .with_static_feedforward_sign(
                StaticFeedforwardSignValue.USE_CLOSED_LOOP_SIGN
            )
        )

        self.climber.configurator.apply(motor_config)
        self.climber.configurator.apply(pid, 0.01)
        self.climber.configurator.apply(feedback_config)

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
        self.climber.configurator.apply(current_limits_config)

    def home(self) -> None:
        self.target_position = self.home_position

    def raise_up(self) -> None:
        self.target_position = self.upper_position

    def execute(self) -> None:
        if self.config_limits:
            self._apply_current_limits()
            self.config_limits = False
        if self.target_position > self.upper_limit:
            self.target_position = self.upper_limit
        elif self.target_position < self.lower_limit:
            self.target_position = self.lower_limit

        self.climber.set_control(self.position_request.with_position(self.target_position))
