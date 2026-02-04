from dataclasses import dataclass


@dataclass
class CANDevice:
    id: int
    bus: str


drive = 'Drive'
shooter = 'Shoot'


class TalonId:
    ROTATE = CANDevice(25, shooter)
    ROLLER = CANDevice(26, shooter)
    KICKER = CANDevice(27, shooter)


class CancoderId:
    MANIP_ARM = CANDevice(35, manipulator)
    MANIP_WRIST = CANDevice(36, manipulator)


class CanId:
    """CAN IDs for miscellaneous devices."""
    CANDI = CANDevice(42, manipulator)


class DigitalIn:
    ELEVATOR_LIMIT = 0


class PWM:
    INTAKE_BREAKER = 0
