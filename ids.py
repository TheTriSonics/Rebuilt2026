from dataclasses import dataclass


@dataclass
class CANDevice:
    id: int
    bus: str


drive = 'Drive'
shooter = 'Drive'


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
    KICKER = CANDevice(52, shooter)
    CLIMBER = CANDevice(53, shooter)
    SINGULATOR = CANDevice(54, shooter)
    SHOOTER_FRONT = CANDevice(55, shooter)
    SHOOTER_REAR = CANDevice(56, shooter)
    TURRET_TURN = CANDevice(57, shooter)


class CancoderId:
    SWERVE_FL = CANDevice(31, drive)
    SWERVE_FR = CANDevice(32, drive)
    SWERVE_BR = CANDevice(33, drive)
    SWERVE_BL = CANDevice(34, drive)
    TURRET = CANDevice(35, drive)
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
