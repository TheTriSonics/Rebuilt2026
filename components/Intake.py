from magicbot import tunable
from wpimath.geometry import Pose3d, Rotation3d, Translation3d
from phoenix6.hardware import CANcoder, TalonFX
from phoenix6.controls import DutyCycleOut
import ids as ids


class IntakeComponent:
    

    # These are set to tunables just so they show up on the dashboard for now
    upper_limit = tunable(100.0)
    lower_limit = tunable(0.0)
    upper_position = tunable(95.0)
    lower_position = tunable(5.0)
    target_position = upper_position
    
    
    target_speed = tunable(0.0)

    intake_speed = tunable(0.0)
    outtake_speed = tunable(0.0)



    rotate = TalonFX(ids.TalonId.ROTATE.id, ids.TalonId.ROTATE.bus)
    roller = TalonFX(ids.TalonId.ROLLER.id, ids.TalonId.ROLLER.bus)


    def __init__(self):
        ...


    def lower_intake(self) -> None:
        # This method would lower the intake out.
        self.target_position = self.lower_position

    def raise_intake(self) -> None:
        # This method would raise the intake up.
        self.target_position = self.upper_position

    def set_speed(self, speed: float) -> None:
        self.target_speed = speed

    def intake_on(self) -> None:
        self.set_speed(self.intake_speed)

    def intake_off(self) -> None:
        self.set_speed(0)

    def execute(self) -> None:
        if self.target_position > self.upper_limit:
            self.target_position = self.upper_limit
        elif self.target_position < self.lower_limit:
            self.target_position = self.lower_limit

        self.rotate.set_position(self.target_position)

        self.roller.set_control(DutyCycleOut(self.target_speed))

    
    
                       
     