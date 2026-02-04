from magicbot import tunable
from phoenix6.hardware import TalonFX
from phoenix6.controls import VelocityVoltage
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



class ClimberComponent:


    # These are set to tunables just so they show up on the dashboard for now
    upper_limit = tunable(100.0)
    lower_limit = tunable(0.0)
    upper_position = tunable(95.0)
    home_position = 0.0
    target_position = upper_position


    target_speed = tunable(0.0)



    climber = TalonFX(ids.TalonId.CLIMBER.id, ids.TalonId.CLIMBER.bus)
    power = VelocityVoltage(0)

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

    # configuration for motor pid

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
    closed_loop_config = ClosedLoopGeneralConfigs()

    climber.configurator.apply(motor_config)
    climber.configurator.apply(pid, 0.01)
    climber.configurator.apply(feedback_config)
    climber.configurator.apply(closed_loop_config)


    def __init__(self):
        ...


    def home_climber(self) -> None:
        # This method would set the climber back to home.
        self.target_position = self.home_position

    def raise_climber(self) -> None:
        # This method would raise the climber up.
        self.target_position = self.upper_position

    def set_speed(self, speed: float) -> None:
        self.target_speed = speed


    def execute(self) -> None:
        if self.target_position > self.upper_limit:
            self.target_position = self.upper_limit
        elif self.target_position < self.lower_limit:
            self.target_position = self.lower_limit

        self.climber.set_control(self.power.with_velocity(self.target_speed))




