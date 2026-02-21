from phoenix6.hardware import TalonFX
from phoenix6.controls import DutyCycleOut
import ids as ids 
from magicbot import tunable


class KickerComponent:
    kicker = TalonFX(ids.TalonId.KICKER.id, ids.TalonId.KICKER.bus)

    target_speed = tunable(0.0)
    forward_speed = tunable(0.50)
    reverse_speed = tunable(-0.50)


    def __init__(self):
        ...

    def set_speed(self, speed: float) -> None:
        self.target_speed = speed

    def kicker_forward(self) -> None:
        self.set_speed(self.forward_speed)

    def kicker_off(self) -> None:
        self.set_speed(0.0)

    def kicker_reverse(self) -> None:
        self.set_speed(self.reverse_speed)

    def execute(self) -> None:
        self.kicker.set_control(DutyCycleOut(self.target_speed))
    

