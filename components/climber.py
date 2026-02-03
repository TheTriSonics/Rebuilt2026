from magicbot import tunable
from wpimath.geometry import Pose3d, Rotation3d, Translation3d
from phoenix6.hardware import CANcoder, TalonFX
from phoenix6.controls import DutyCycleOut, VelocityVoltage
import ids as ids


class ClimberComponent:
    

    # These are set to tunables just so they show up on the dashboard for now
    upper_limit = tunable(100.0)
    lower_limit = tunable(0.0)
    upper_position = tunable(95.0)
    home_position = 0.0
    target_position = upper_position
    
    
    target_speed = tunable(0.0)


    climber = TalonFX(ids.TalonId.CLIMBER.id, ids.TalonId.CLIMBER.bus)


    def __init__(self):
        ...


    def home_climber(self) -> None:
        # This method would set the climber back to home.
        self.target_position = self.home_position

    def raise_climber(self) -> None:
        # This method would raise the climber up.
        self.target_position = self.upper_position

    def set_speed(self, speed: float) -> None:
        self.target_speed = speed


    def execute(self) -> None:
        if self.target_position > self.upper_limit:
            self.target_position = self.upper_limit
        elif self.target_position < self.lower_limit:
            self.target_position = self.lower_limit

        self.climber.set_control(VelocityVoltage(0).with_velocity(self.target_speed))

    
    
                       
     