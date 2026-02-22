from magicbot import tunable
from phoenix6.hardware import TalonFX
from phoenix6.controls import DutyCycleOut
import ids

from phoenix6.signals import (
    InvertedValue,
    NeutralModeValue,
    StaticFeedforwardSignValue,
)
from phoenix6.configs import (
    ClosedLoopGeneralConfigs,
    CurrentLimitsConfigs,
    FeedbackConfigs,
    MotorOutputConfigs,
    Slot0Configs,
)



class IntakeComponent:

    # These are set to tunables just so they show up on the dashboard for now
    upper_limit = tunable(100.0)
    lower_limit = tunable(0.0)
    upper_position = tunable(95.0)
    lower_position = tunable(5.0)
    target_position = upper_position

    target_speed = tunable(0.0)

    intake_speed = tunable(-0.7)
    outtake_speed = tunable(0.7)

    config_limits = tunable(False)
    stator_current_limit = tunable(1.0)
    supply_current_limit = tunable(10.0)
    supply_current_lower_limit = tunable(5.0)
    supply_current_lower_time = tunable(1.0)

    rotate = TalonFX(ids.TalonId.ROTATE.id, ids.TalonId.ROTATE.bus)
    roller = TalonFX(ids.TalonId.ROLLER.id, ids.TalonId.ROLLER.bus)

    def __init__(self):
        motor_config = MotorOutputConfigs()
        motor_config.neutral_mode = NeutralModeValue.BRAKE
        motor_config.inverted = InvertedValue.CLOCKWISE_POSITIVE

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
        closed_loop_config = ClosedLoopGeneralConfigs()

        self.rotate.configurator.apply(motor_config)
        self.rotate.configurator.apply(pid, 0.01)
        self.rotate.configurator.apply(feedback_config)
        self.rotate.configurator.apply(closed_loop_config)

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
        self.roller.configurator.apply(current_limits_config)


    def lower_intake(self) -> None:
        # This method would lower the intake out.
        self.target_position = self.lower_position

    def raise_intake(self) -> None:
        # This method would raise the intake up.
        self.target_position = self.upper_position

    def set_speed(self, speed: float) -> None:
        self.target_speed = speed

    def intake_on(self) -> None:
        self.set_speed(self.intake_speed)

    def intake_off(self) -> None:
        self.set_speed(0)

    def intake_reverse(self) -> None:
        self.set_speed(-self.intake_speed)

    def execute(self) -> None:
        if self.config_limits:
            self._apply_current_limits()
            self.config_limits = False
        if self.target_position > self.upper_limit:
            self.target_position = self.upper_limit
        elif self.target_position < self.lower_limit:
            self.target_position = self.lower_limit

        # self.rotate.set_position(self.target_position)

        self.roller.set_control(DutyCycleOut(self.target_speed))
