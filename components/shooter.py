from magicbot import tunable, feedback
from phoenix6.hardware import TalonFX
from phoenix6.controls import VelocityTorqueCurrentFOC, VoltageOut
from phoenix6.configs import CurrentLimitsConfigs, MotorOutputConfigs, Slot0Configs
from phoenix6.signals import InvertedValue, NeutralModeValue, StaticFeedforwardSignValue

from components.shot_calculator import ShotCalculatorComponent
from wpimath.units import metersToInches

import ids


class ShooterComponent:
    shot_calc: ShotCalculatorComponent

    shooter_left = TalonFX(ids.TalonId.SHOOTER_LEFT.id, ids.TalonId.SHOOTER_LEFT.bus) # SETUP FOLLOWER ON RIGHT
    shooter_right = TalonFX(ids.TalonId.SHOOTER_RIGHT.id, ids.TalonId.SHOOTER_RIGHT.bus)
    shooter_hood = TalonFX(ids.TalonId.SHOOTER_HOOD.id, ids.TalonId.SHOOTER_HOOD.bus)

    coef = tunable(0.15)
    base = tunable(3)
    hood_rps = tunable(0.0)
    flywheel_rps = tunable(40.0)
    active = tunable(False)

    config_limits = tunable(False)
    stator_current_limit = tunable(40.0)
    supply_current_limit = tunable(60.0)
    supply_current_lower_limit = tunable(40.0)
    supply_current_lower_time = tunable(1.0)

    at_speed_counter = 0
    at_speed_stable = False

    shooter_left_velocity = 0.0
    shooter_right_velocity = 0.0
    shooter_hood_velocity = 0.0

    def __init__(self):
        motor_config = MotorOutputConfigs()
        motor_config.neutral_mode = NeutralModeValue.COAST
        motor_config.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE

        left_pid = (
            Slot0Configs()
            .with_k_p(3.0)
            .with_k_i(0.0)
            .with_k_d(0.0)
            .with_k_s(2.2)
            .with_k_v(0.45)
            .with_k_a(0.0)
            .with_static_feedforward_sign(
                StaticFeedforwardSignValue.USE_CLOSED_LOOP_SIGN
            )
        )

        right_pid = (
            Slot0Configs()
            .with_k_p(3.0)
            .with_k_i(0.0)
            .with_k_d(0.0)
            .with_k_s(2.2)
            .with_k_v(0.45)
            .with_k_a(0.0)
            .with_static_feedforward_sign(
                StaticFeedforwardSignValue.USE_CLOSED_LOOP_SIGN
            )
        )
        hood_pid = (
            Slot0Configs()
            .with_k_p(3.0)
            .with_k_i(0.0)
            .with_k_d(0.0)
            .with_k_s(2.2)
            .with_k_v(0.75)
            .with_k_a(0.0)
            .with_static_feedforward_sign(
                StaticFeedforwardSignValue.USE_CLOSED_LOOP_SIGN
            )
        )


        self.shooter_left.configurator.apply(motor_config)
        self.shooter_left.configurator.apply(left_pid, 0.01)

        self.shooter_right.configurator.apply(motor_config)
        self.shooter_right.configurator.apply(right_pid, 0.01)

        self.shooter_hood.configurator.apply(motor_config)
        self.shooter_hood.configurator.apply(hood_pid, 0.01)

        self.flywheel_velocity_request = VelocityTorqueCurrentFOC(0).with_slot(0)
        self.hood_velocity_request = VelocityTorqueCurrentFOC(0).with_slot(0)
        self.stop_request = VoltageOut(0)

    def setup(self):
        self._apply_current_limits()
        self.shooter_left.set_control(self.stop_request)
        self.shooter_right.set_control(self.stop_request)
        self.shooter_hood.set_control(self.stop_request)

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
        self.shooter_left.configurator.apply(current_limits_config)
        self.shooter_right.configurator.apply(current_limits_config)
        # self.shooter_hood.configurator.apply(current_limits_config)

    def spin_up(self) -> None:
        self.active = True

    def stop(self) -> None:
        self.hood_rps = 0.0
        self.active = False

    def is_active(self) -> bool:
        return self.active

    def is_off(self) -> bool:
        return not self.active

    @feedback
    def shooter_velocity_left(self) -> float:
        return self.shooter_left_velocity

    @feedback
    def shooter_velocity_right(self) -> float:
        return self.shooter_right_velocity

    @feedback
    def hood_velocity(self) -> float:
        return self.shooter_hood_velocity

    @feedback
    def get_shooter_target(self) -> float:
        return self.flywheel_rps

    @feedback
    def get_hood_target(self) -> float:
        return self.hood_rps

    
    @feedback
    def shooter_at_speed(self) -> bool:
        return self.at_speed_stable
    

    def is_at_speed(self) -> bool:
        if not self.active:
            return False

        margin = 0.95
        left_vel = self.shooter_left_velocity
        right_vel = self.shooter_right_velocity
        hood_vel = self.shooter_hood_velocity
        at_speed = left_vel >= self.flywheel_rps * margin and right_vel >= self.flywheel_rps * margin and hood_vel >= self.hood_rps * margin
        if at_speed:
            self.at_speed_counter += 1
        else:
            self.at_speed_counter = 0
        
        self.at_speed_stable = False
        if self.at_speed_counter >= 10:
            self.at_speed_stable = True
        return self.at_speed_stable 

    def calc_rps(self) -> float:
        dist = self.shot_calc.get_field_shot_distance()
        dist_in = metersToInches(dist)
        rps = self.coef * dist_in + self.base
        rps = min(rps, 50)
        rps = max(rps, 10)
        return rps

    def execute(self) -> None:
        self.shooter_left_velocity = abs(self.shooter_left.get_velocity().value)
        self.shooter_right_velocity = abs(self.shooter_right.get_velocity().value)
        self.shooter_hood_velocity = abs(self.shooter_hood.get_velocity().value)

        if self.config_limits:
            self._apply_current_limits()
            self.config_limits = False

        self.hood_rps = self.calc_rps()
        if self.active and self.hood_rps != 0.0:
            self.shooter_left.set_control(self.flywheel_velocity_request.with_velocity(-self.flywheel_rps))
            self.shooter_right.set_control(self.flywheel_velocity_request.with_velocity(self.flywheel_rps))
            self.shooter_hood.set_control(self.hood_velocity_request.with_velocity(self.hood_rps))
        else:
            self.shooter_left.set_control(self.stop_request)
            self.shooter_right.set_control(self.stop_request)
            self.shooter_hood.set_control(self.stop_request)