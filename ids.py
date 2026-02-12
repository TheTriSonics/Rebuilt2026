from dataclasses import dataclass


@dataclass
class CANDevice:
    id: int
    bus: str


drive = 'Drive'
shooter = 'Drive'


class TalonId:
    ROTATE = CANDevice(50, shooter)
    ROLLER = CANDevice(51, shooter)
    KICKER = CANDevice(52, shooter)
    CLIMBER = CANDevice(53, shooter)
    SINGULATOR = CANDevice(54, shooter)


class CancoderId:
    ...


class CanId:
    """CAN IDs for miscellaneous devices."""
    ...


class DigitalIn:
    ELEVATOR_LIMIT = 0


class PWM:
    INTAKE_BREAKER = 0
