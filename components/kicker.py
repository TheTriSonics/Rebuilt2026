from phoenix6.hardware import TalonFX
from phoenix6.controls import DutyCycleOut
from phoenix6.configs import CurrentLimitsConfigs
import ids
from magicbot import tunable


class KickerComponent:
    kicker = TalonFX(ids.TalonId.KICKER.id, ids.TalonId.KICKER.bus)

    target_speed = 0.0
    forward_speed = tunable(0.20)
    reverse_speed = tunable(-0.20)

    config_limits = tunable(False)
    stator_current_limit = tunable(80.0)
    supply_current_limit = tunable(120.0)
    supply_current_lower_limit = tunable(120.0)
    supply_current_lower_time = tunable(0.0)

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

    def set_speed(self, speed: float) -> None:
        self.target_speed = speed

    def kicker_forward(self) -> None:
        self.set_speed(self.forward_speed)

    def kicker_off(self) -> None:
        self.set_speed(0.0)

    def kicker_reverse(self) -> None:
        self.set_speed(self.reverse_speed)

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
        self.kicker.set_control(DutyCycleOut(self.target_speed))
