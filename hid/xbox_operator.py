from wpilib import XboxController




class RebuiltOperator():

    status = 'off'
   
    def __init__(self) -> None:
        self.controller = XboxController(1)

    def hang_up(self):
       return self.controller.getRightTriggerAxis() > 0.1
    
    def hang_down(self):
        return self.controller.getLeftTriggerAxis() > 0.1
    
    def singulator_forward(self):
        return self.controller.getAButtonPressed()

    def singulator_reverse(self):
        return self.controller.getBButtonPressed()
    
    def intake_up(self):
        return self.controller.getXButtonPressed()
    
    def intake_down(self):
        return self.controller.getYButtonPressed()
    
    def intake_on(self):
        return self.controller.getRightTriggerAxis() > 0.1
    
    def intake_reverse(self):
        return self.controller.getLeftTriggerAxis() > 0.1
    
    def shooter_shoot(self):
        return self.controller.getYButtonPressed()
    
    def turret_movement(self):
        return self.controller.getRightX()

    def hood_movement(self):
        return self.controller.getLeftY()
    
    #def toggle(self):
        if self.controller.getAButtonPressed():
            #if self.status == 'off':
            self.status = 'on'
            #else:
             #   self.status = 'off'
            
            
    #def shoot(self):
        if self.status == 'on':
            return self.controller.getRightBumperPressed()
        print('on')

        
    