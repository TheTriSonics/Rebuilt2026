#needs to spin both directions, speed control,

from magicbot import tunable
from phoenix6.hardware import TalonFX
from phoenix6.controls import VelocityVoltage, DutyCycleOut
from phoenix6.configs import CurrentLimitsConfigs, Slot0Configs
from phoenix6.signals import StaticFeedforwardSignValue
import ids

class SingulatorComponent:

    singulator = TalonFX(ids.TalonId.SINGULATOR.id, ids.TalonId.SINGULATOR.bus)

    target_speed = tunable(0.0)
    forward_speed = tunable(0.3)
    reverse_speed = tunable(-0.3)

    config_limits = tunable(False)
    stator_current_limit = tunable(20.0)
    supply_current_limit = tunable(40.0)
    supply_current_lower_limit = tunable(40.0)
    supply_current_lower_time = tunable(1.0)


    def __init__(self):
        singulator_pid = (
            Slot0Configs()
            .with_k_p(500.0)
            .with_k_i(0.0)
            .with_k_d(0.0)
            .with_k_s(0.1)
            .with_k_v(0.12)
            .with_static_feedforward_sign(
                StaticFeedforwardSignValue.USE_VELOCITY_SIGN
            )
        )
        self.singulator.configurator.apply(singulator_pid)
        self.output = VelocityVoltage(0).with_slot(0)

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
        self.singulator.configurator.apply(current_limits_config)

    def singulator_off(self):
         self.set_speed(0)

    def singulator_forward(self):
        self.set_speed(self.forward_speed)

    def singulator_reverse(self):
        self.set_speed(self.reverse_speed)

    def set_speed(self, speed: float) -> None:
        self.target_speed = speed

    def execute(self) -> None:
        if self.config_limits:
            self._apply_current_limits()
            self.config_limits = False
        # self.singulator.set_control(self.output.with_velocity(self.target_speed))
        self.singulator.set_control(DutyCycleOut(self.target_speed))
    
