from dataclasses import dataclass


@dataclass
class CANDevice:
    id: int
    bus: str


drive = 'Drive'
manipulator = 'Arm'


class TalonId:
    MANIP_ARM = CANDevice(25, manipulator)
    MANIP_WRIST = CANDevice(26, manipulator)
    MANIP_INTAKE = CANDevice(27, manipulator)

    CLIMB = CANDevice(51, manipulator)

    UNUSED = CANDevice(51, drive)
    UNUSED2 = CANDevice(52, drive)

    MANIP_ELEVATOR_LEFT = CANDevice(61, drive)
    MANIP_ELEVATOR_RIGHT = CANDevice(62, drive)


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
