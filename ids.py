from dataclasses import dataclass


@dataclass
class CANDevice:
    id: int
    bus: str


drive = 'Drive'
shooter = 'Shooter'


class TalonId:
    DRIVE_FL = CANDevice(11, drive)
    DRIVE_FR = CANDevice(12, drive)
    DRIVE_BR = CANDevice(13, drive)
    DRIVE_BL = CANDevice(14, drive)

    TURN_FL = CANDevice(21, drive)
    TURN_FR = CANDevice(22, drive)
    TURN_BR = CANDevice(23, drive)
    TURN_BL = CANDevice(24, drive)

    ROTATE = CANDevice(50, shooter)
    ROLLER = CANDevice(51, shooter)
    KICKER = CANDevice(54, shooter)
    CLIMBER = CANDevice(63, shooter)
    SHOOTER_LEFT = CANDevice(53, shooter)
    SHOOTER_RIGHT = CANDevice(52, shooter)
    SHOOTER_HOOD = CANDevice(57, shooter)


class CancoderId:
    SWERVE_FL = CANDevice(31, drive)
    SWERVE_FR = CANDevice(32, drive)
    SWERVE_BR = CANDevice(33, drive)
    SWERVE_BL = CANDevice(34, drive)
    INTAKE = CANDevice(36, shooter)


class CANdleId:
    CANDLE = CANDevice(5, drive)


class CanId:
    """CAN IDs for miscellaneous devices."""
    PIGEON = CANDevice(41, drive)


class DigitalIn:
    ELEVATOR_LIMIT = 0


class PWM:
    INTAKE_BREAKER = 0
