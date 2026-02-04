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
    CLIMBER = CANDevice(28, shooter)
    SINGULATOR = CANDevice(29, shooter)


class CancoderId:
    ...


class CanId:
    """CAN IDs for miscellaneous devices."""
    ...


class DigitalIn:
    ELEVATOR_LIMIT = 0


class PWM:
    INTAKE_BREAKER = 0
