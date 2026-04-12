from magicbot import tunable, feedback
from phoenix6.hardware import TalonFX
from phoenix6.controls import DutyCycleOut, MotionMagicVoltage
import ids

from phoenix6.signals import (
    InvertedValue,
    NeutralModeValue,
    StaticFeedforwardSignValue,
)
from phoenix6.configs import (
    CurrentLimitsConfigs,
    FeedbackConfigs,
    MotionMagicConfigs,
    MotorOutputConfigs,
    Slot0Configs,
)


class IntakeComponent:

    # Measure: extend fully, read rotations, measure distance in meters,
    # then set this to distance / rotations.
    meters_per_rotation = 0.01  # One rotation works out to 1cm. Nice!

    out_position = 0.30  # meters
    in_position = 0.00  # meters
    target_position = tunable(0.0)  # meters

    intake_speed = tunable(0.45 * 1.1)
    sushi_speed = tunable(0.425 * 1.1)
    target_intake_speed = tunable(0.0)
    target_sushi_speed = tunable(0.0)

    config_limits = tunable(False)
    stator_current_limit = tunable(60)
    supply_current_limit = tunable(120)
    supply_current_lower_limit = tunable(0)
    supply_current_lower_time = tunable(0.0)

    extend = TalonFX(ids.TalonId.EXTEND.id, ids.TalonId.EXTEND.bus)
    roller = TalonFX(ids.TalonId.ROLLER.id, ids.TalonId.ROLLER.bus)
    sushi = TalonFX(ids.TalonId.SUSHI.id, ids.TalonId.SUSHI.bus)
    extend_request = MotionMagicVoltage(0).with_slot(0)

    def __init__(self):
        extend_motor_config = MotorOutputConfigs()
        # Extend is in coast mode because we can use it like a bumper of sorts.
        extend_motor_config.neutral_mode = NeutralModeValue.COAST
        extend_motor_config.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE

        roller_motor_config = MotorOutputConfigs()
        roller_motor_config.neutral_mode = NeutralModeValue.BRAKE
        roller_motor_config.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE

        sushi_motor_config = MotorOutputConfigs()
        sushi_motor_config.neutral_mode = NeutralModeValue.BRAKE
        sushi_motor_config.inverted = InvertedValue.CLOCKWISE_POSITIVE

        feedback_config = FeedbackConfigs()
        feedback_config.sensor_to_mechanism_ratio = 1.0
        feedback_config.rotor_to_sensor_ratio = 1.0

        pid = (
            Slot0Configs()
            .with_k_p(2.0)
            .with_k_i(0.0)
            .with_k_d(0.0)
            .with_k_s(0.05)
            .with_k_v(0.15)
            .with_k_a(0)
            .with_static_feedforward_sign(
                StaticFeedforwardSignValue.USE_CLOSED_LOOP_SIGN
            )
        )

        motion_magic = (
            MotionMagicConfigs()
            .with_motion_magic_cruise_velocity(60.0)
            .with_motion_magic_acceleration(120.0)
            .with_motion_magic_jerk(600.0)
        )

        self.extend.configurator.apply(extend_motor_config)
        self.extend.configurator.apply(pid, 2.0)
        self.extend.configurator.apply(feedback_config)
        self.extend.configurator.apply(motion_magic)

        self.roller.configurator.apply(roller_motor_config)
        self.sushi.configurator.apply(sushi_motor_config)

    def setup(self):
        self.extend.set_position(0.00)
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
        self.extend.configurator.apply(current_limits_config)

    def _meters_to_rotations(self, meters: float) -> float:
        return meters / self.meters_per_rotation

    def _rotations_to_meters(self, rotations: float) -> float:
        return rotations * self.meters_per_rotation

    def extend_out(self) -> None:
        self.target_position = self.out_position

    def pull_in(self) -> None:
        self.target_position = self.in_position

    def push_out(self) -> None:
        self.target_position = self.out_position

    def set_speed(self, intake_speed: float, sushi_speed: float) -> None:
        self.target_intake_speed = intake_speed
        self.target_sushi_speed = sushi_speed

    def on(self) -> None:
        self.set_speed(self.intake_speed, self.sushi_speed)

    def off(self) -> None:
        self.set_speed(0, 0)

    def reverse(self) -> None:
        self.set_speed(-self.intake_speed, -self.sushi_speed)

    @feedback
    def get_extend_position(self) -> float:
        return self._rotations_to_meters(self.extend.get_position().value)

    @feedback
    def get_intake_on(self) -> bool:
        return self.target_intake_speed != 0

    @feedback
    def extend_motor_temp(self) -> float:
        return self.extend.get_device_temp().value

    @feedback
    def roller_motor_temp(self) -> float:
        return self.roller.get_device_temp().value

    @feedback
    def sushi_motor_temp(self) -> float:
        return self.sushi.get_device_temp().value

    def execute(self) -> None:
        if self.config_limits:
            self._apply_current_limits()
            self.config_limits = False
        if self.target_position > self.out_position:
            self.target_position = self.out_position
        elif self.target_position < self.in_position:
            self.target_position = self.in_position

        target_rotations = self._meters_to_rotations(self.target_position)
        self.extend.set_control(self.extend_request.with_position(target_rotations))
        self.roller.set_control(DutyCycleOut(self.target_intake_speed))
        self.sushi.set_control(DutyCycleOut(self.target_sushi_speed))
