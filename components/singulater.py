#needs to spin both directions, speed control,

from magicbot import tunable
from phoenix6.hardware import CANcoder, TalonFX
from phoenix6.controls import VelocityVoltage
import ids as ids

class SingulaterComponent:

    singulater = TalonFX(ids.TalonId.SINGULATER.id, ids.TalonId.SINGULATER.bus)

    target_speed = tunable(0.0)
    forward_speed = tunable(0.0)
    reverse_speed = tunable(0.0)


    def __init__(self):
        ...

    def singulater_off(self):
         self.set_speed(0)

    def singulater_forward(self):
        self.set_speed(self.forward_speed)

    def singulater_reverse(self):
        self.set_speed(self.reverse_speed)

    def set_speed(self, speed: float) -> None:
        self.target_speed = speed

    def execute(self) -> None:
        self.singulater.set_control(VelocityVoltage(0).with_velocity(self.target_speed))
    
