from magicbot import tunable
from phoenix6.hardware import TalonFX
from phoenix6.controls import Follower, VelocityTorqueCurrentFOC, VoltageOut
from phoenix6.configs import CurrentLimitsConfigs, MotorOutputConfigs, Slot0Configs
from phoenix6.signals import InvertedValue, MotorAlignmentValue, NeutralModeValue, StaticFeedforwardSignValue

from components.drivetrain import DrivetrainComponent
from components.turret import TurretComponent
from wpimath.geometry import Transform2d, Rotation2d

import ids


class ShooterComponent:
    drivetrain: DrivetrainComponent
    turret: TurretComponent

    shooter_front = TalonFX(ids.TalonId.SHOOTER_FRONT.id, ids.TalonId.SHOOTER_FRONT.bus)
    shooter_rear = TalonFX(ids.TalonId.SHOOTER_REAR.id, ids.TalonId.SHOOTER_REAR.bus)

    coef = tunable(0.8)
    base = tunable(3.82)
    target_rps = tunable(0.0)
    active = tunable(False)

    config_limits = tunable(False)
    stator_current_limit = tunable(40.0)
    supply_current_limit = tunable(60.0)
    supply_current_lower_limit = tunable(40.0)
    supply_current_lower_time = tunable(1.0)

    def __init__(self):
        motor_config = MotorOutputConfigs()
        motor_config.neutral_mode = NeutralModeValue.COAST
        motor_config.inverted = InvertedValue.CLOCKWISE_POSITIVE

        # Shooter front: kP 4.0, kS 2.15, kV 0.14
        front_pid = (
            Slot0Configs()
            .with_k_p(3.0)
            .with_k_i(0.0)
            .with_k_d(0.0)
            .with_k_s(2.2)
            .with_k_v(0.025)
            .with_k_a(0.0)
            .with_static_feedforward_sign(
                StaticFeedforwardSignValue.USE_CLOSED_LOOP_SIGN
            )
        )

        # Shooter rear: kP 4.0, kS 2.15, kV 0.14
        rear_pid = (
            Slot0Configs()
            .with_k_p(3.1)
            .with_k_i(0.0)
            .with_k_d(0.0)
            .with_k_s(2.3)
            .with_k_v(0.08)
            .with_k_a(0.0)
            .with_static_feedforward_sign(
                StaticFeedforwardSignValue.USE_CLOSED_LOOP_SIGN
            )
        )

        self.shooter_front.configurator.apply(motor_config)
        self.shooter_front.configurator.apply(front_pid, 0.01)

        rear_motor_config = MotorOutputConfigs()
        rear_motor_config.neutral_mode = NeutralModeValue.COAST
        rear_motor_config.inverted = InvertedValue.CLOCKWISE_POSITIVE

        self.shooter_rear.configurator.apply(rear_motor_config)
        self.shooter_rear.configurator.apply(rear_pid, 0.01)

        self.velocity_request = VelocityTorqueCurrentFOC(0).with_slot(0)
        self.stop_request = VoltageOut(0)
        self.follower_request = Follower(ids.TalonId.SHOOTER_FRONT.id, MotorAlignmentValue.OPPOSED)

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
        self.shooter_front.configurator.apply(current_limits_config)
        self.shooter_rear.configurator.apply(current_limits_config)

    def spin_up(self) -> None:
        self.active = True

    def stop(self) -> None:
        self.target_rps = 0.0
        self.active = False

    def is_active(self) -> bool:
        return self.active

    def is_off(self) -> bool:
        return not self.active

    def is_at_speed(self) -> bool:
        if not self.active or self.target_rps == 0.0:
            return False
        front_vel = abs(self.shooter_front.get_velocity().value)
        rear_vel = abs(self.shooter_rear.get_velocity().value)
        target = abs(self.target_rps)
        return front_vel >= target * 0.95 and rear_vel >= target * 0.95

    def calc_rps(self) -> float:
        from wpimath.units import inchesToMeters, metersToInches
        robot_pose = self.drivetrain.get_pose()
        turret_pose = robot_pose.transformBy(Transform2d(inchesToMeters(-9.0), 0, Rotation2d(0)))
        dist = turret_pose.relativeTo(self.turret.goal_pose.toPose2d()).translation().norm()
        dist_in = metersToInches(dist)
        # rps = 0.8 * dist_in + 3.82
        rps = self.coef * dist_in + self.base
        rps = min(rps, 85)
        rps = max(rps, 44)
        return rps

    def execute(self) -> None:
        if self.config_limits:
            self._apply_current_limits()
            self.config_limits = False

        self.target_rps = self.calc_rps()
        if self.active and self.target_rps != 0.0:
            self.shooter_front.set_control(self.velocity_request.with_velocity(self.target_rps))
            self.shooter_rear.set_control(self.velocity_request.with_velocity(-self.target_rps))
        else:
            self.shooter_front.set_control(self.stop_request)
            self.shooter_rear.set_control(self.stop_request)
        # self.shooter_rear.set_control(self.follower_request) # Follower not working, resorted to setting velocity on both motors
