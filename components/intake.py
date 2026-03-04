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
    upper_limit = tunable(0)
    lower_limit = tunable(0.3)
    upper_position = tunable(0.32)
    lower_position = tunable(2.0)
    target_position = upper_position

    target_speed = tunable(0.0)

    intake_speed = tunable(0.7)
    outtake_speed = tunable(-0.7)

    config_limits = tunable(False)
    stator_current_limit = tunable(1.0)
    supply_current_limit = tunable(10.0)
    supply_current_lower_limit = tunable(5.0)
    supply_current_lower_time = tunable(1.0)

    rotate = TalonFX(ids.TalonId.ROTATE.id, ids.TalonId.ROTATE.bus)
    roller = TalonFX(ids.TalonId.ROLLER.id, ids.TalonId.ROLLER.bus)
    rotate_encoder = CANcoder(ids.CancoderId.INTAKE.id, ids.CancoderId.INTAKE.bus)
    rotate_request = PositionVoltage(0).with_slot(0)

    def __init__(self):
        motor_config = MotorOutputConfigs()
        motor_config.neutral_mode = NeutralModeValue.BRAKE
        motor_config.inverted = InvertedValue.CLOCKWISE_POSITIVE

        encoder_config = CANcoderConfiguration()

        feedback_config = FeedbackConfigs()
        feedback_config.feedback_remote_sensor_id = ids.CancoderId.INTAKE.id
        feedback_config.feedback_sensor_source = FeedbackSensorSourceValue.FUSED_CANCODER
        feedback_config.sensor_to_mechanism_ratio = 1.0
        feedback_config.rotor_to_sensor_ratio = 135.0

        pid = (
            Slot0Configs()
            .with_k_p(2)
            .with_k_i(0)
            .with_k_d(0.0)
            .with_k_s(-0.012)
            .with_k_v(0)
            .with_k_a(0)
            .with_static_feedforward_sign(
                StaticFeedforwardSignValue.USE_CLOSED_LOOP_SIGN
            )
        )
        closed_loop_config = ClosedLoopGeneralConfigs()

        self.rotate_encoder.configurator.apply(encoder_config)
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
            .with_stator_current_limit_enable(False)
            .with_supply_current_limit(self.supply_current_limit)
            .with_supply_current_limit_enable(False)
            .with_supply_current_lower_limit(self.supply_current_lower_limit)
            .with_supply_current_lower_time(self.supply_current_lower_time)
        )
        self.roller.configurator.apply(current_limits_config)


    def lower_intake(self) -> None:
        # This method would lower the intake out.
        #self.rotate_down()
        ...

    def rotate_down(self) -> None:
        # Move intake to the configured lowered setpoint.
        #self.target_position = self.lower_position
        ...

    def raise_intake(self) -> None:
        # This method would raise the intake up.
        #self.rotate_up()
        ...

    def rotate_up(self) -> None:
        # Move intake to the configured raised setpoint.
        #self.target_position = self.upper_position
        ...

    def set_speed(self, speed: float) -> None:
        self.target_speed = speed

    def intake_on(self) -> None:
        self.set_speed(self.intake_speed)

    def intake_off(self) -> None:
        self.set_speed(0)

    def intake_reverse(self) -> None:
        self.set_speed(-self.intake_speed)

    @feedback
    def get_rotate_position(self) -> float:
        return self.rotate_encoder.get_position().value

    def execute(self) -> None:
        if self.config_limits:
            self._apply_current_limits()
            self.config_limits = False
        if self.target_position > self.upper_limit:
            self.target_position = self.upper_limit
        elif self.target_position < self.lower_limit:
            self.target_position = self.lower_limit

        # self.rotate.set_position(self.target_position)
        #self.rotate.set_control(self.rotate_request.with_position(self.target_position))

        
        self.roller.set_control(DutyCycleOut(self.target_speed))
