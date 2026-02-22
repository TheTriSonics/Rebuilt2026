#needs to spin both directions, speed control,

from magicbot import tunable
from phoenix6.hardware import TalonFX
from phoenix6.controls import VelocityVoltage, DutyCycleOut
from phoenix6.configs import CurrentLimitsConfigs, FeedbackConfigs, Slot0Configs, MotorOutputConfigs
from phoenix6.signals import FeedbackSensorSourceValue, StaticFeedforwardSignValue, NeutralModeValue
import ids

class SingulatorComponent:

    singulator = TalonFX(ids.TalonId.SINGULATOR.id, ids.TalonId.SINGULATOR.bus)

    target_speed = 0.0  # rotations per second
    forward_speed = tunable(60.0)
    reverse_speed = tunable(-30.0)

    config_limits = tunable(False)
    stator_current_limit = tunable(120.0)
    supply_current_limit = tunable(120.0)
    supply_current_lower_limit = tunable(120.0)
    supply_current_lower_time = tunable(0.0)


    def __init__(self):
        motor_config = MotorOutputConfigs()
        motor_config.neutral_mode = NeutralModeValue.COAST
        self.singulator.configurator.apply(motor_config)

        feedback_config = FeedbackConfigs()
        feedback_config.feedback_sensor_source = FeedbackSensorSourceValue.ROTOR_SENSOR
        self.singulator.configurator.apply(feedback_config)

        singulator_pid = (
            Slot0Configs()
            .with_k_p(1.0)
            .with_k_i(0.0)
            .with_k_d(0.0)
            .with_k_s(0.25)
            .with_k_v(0.12)
            .with_static_feedforward_sign(
                StaticFeedforwardSignValue.USE_VELOCITY_SIGN
            )
        )
        self.singulator.configurator.apply(singulator_pid)
        self.velocity_request = VelocityVoltage(0).with_slot(0)

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
         self.set_speed(0.0)

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
        if abs(self.target_speed) > 0.02:
            self.singulator.set_control(self.velocity_request.with_velocity(self.target_speed))
        else:
            self.singulator.set_control(DutyCycleOut(0.0))
    
