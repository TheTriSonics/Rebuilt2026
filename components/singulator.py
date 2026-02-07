#needs to spin both directions, speed control,

from magicbot import tunable
from phoenix6.hardware import TalonFX
from phoenix6.controls import VelocityVoltage, DutyCycleOut
import ids as ids

from phoenix6.signals import (
    InvertedValue,
    NeutralModeValue,
    StaticFeedforwardSignValue,
)
from phoenix6.configs import (
    ClosedLoopGeneralConfigs,
    FeedbackConfigs,
    MotorOutputConfigs,
    Slot0Configs,
)

class SingulatorComponent:

    singulator = TalonFX(ids.TalonId.SINGULATOR.id, ids.TalonId.SINGULATOR.bus)

    motor_config = MotorOutputConfigs()
    motor_config.neutral_mode = NeutralModeValue.BRAKE
    # The SDS Mk4i rotation has one pair of gears.
    motor_config.inverted = (
        InvertedValue.CLOCKWISE_POSITIVE
        if False
        else InvertedValue.COUNTER_CLOCKWISE_POSITIVE
    )
    feedback_config = FeedbackConfigs()
    feedback_config.sensor_to_mechanism_ratio = 1.0
    feedback_config.rotor_to_sensor_ratio = 1.0

    closed_loop_config = ClosedLoopGeneralConfigs()

    singulator.configurator.apply(motor_config)
    singulator.configurator.apply(feedback_config)
    singulator.configurator.apply(closed_loop_config)

    target_speed = tunable(0.0)
    forward_speed = tunable(0.5)
    reverse_speed = tunable(-0.5)


    def __init__(self):
        ...

    def singulator_off(self):
         self.set_speed(0)

    def singulator_forward(self):
        self.set_speed(self.forward_speed)

    def singulator_reverse(self):
        self.set_speed(self.reverse_speed)

    def set_speed(self, speed: float) -> None:
        self.target_speed = speed

    def execute(self) -> None:
        self.singulator.set_control(DutyCycleOut(self.target_speed))
    
