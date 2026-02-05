from wpilib import XboxController


class RebuiltDriver():
   
    def __init__(self) -> None:
        self.controller = XboxController(0)

    def get_right_x(self):
        return self.controller.getRightX()
    
    def get_right_y(self):
        return self.controller.getRightY()
    
    def get_left_x(self):
        return self.controller.getLeftX()
    
    def get_left_y(self):
        return self.controller.getLeftY()
    
    def field_centric(self):
        return self.controller.getRawButtonPressed(8)
    
    def robot_centric(self):
        return self.controller.getRawButtonPressed(7)
    
    def shooter(self):
        return self.controller.getRightBumperButtonPressed()
    
    def intake_on(self):
        return self.controller.getRightTriggerAxis() > 0.1
    
    

    
