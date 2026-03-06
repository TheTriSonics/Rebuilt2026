from wpilib import XboxController


class RebuiltOperator:

    def __init__(self) -> None:
        self.controller = XboxController(1)

    # def hang_up(self):
    #     return self.controller.getRightTriggerAxis() > 0.1

    # def hang_down(self):
    #     return self.controller.getLeftTriggerAxis() > 0.1

    # def singulator_forward(self):
    #     return self.controller.getAButton()

    # def singulator_reverse(self):
    #     return self.controller.getBButton()

    # def kicker_on(self):
    #     return self.controller.getAButton()

    # def intake_up(self):
    #     return self.controller.getXButtonPressed()

    # def intake_down(self):
    #     return self.controller.getYButtonPressed()

    # def intake_on(self):
    #     return self.controller.getYButton()

    # def intake_reverse(self):
    #     return self.controller.getLeftTriggerAxis() > 0.1

    # def shooter_shoot(self):
    #     return self.controller.getRightBumperButton()

    # def intake_spin(self):
    #     return self.controller.getLeftBumperButton()

    # def turret_movement(self):
    #     return self.controller.getRightX()

    # def hood_movement(self):
    #     return self.controller.getLeftY()

    def intake_idle(self):
        return self.controller.getLeftBumperButtonPressed()
    
    def eject(self):
        return self.controller.getXButtonPressed()
    
    def singulating(self):
        return self.controller.getLeftTriggerAxis() > 0.1
    
    def shooter_shoot(self):
        return self.controller.getRightBumperButton()
    
    def intake_on(self):
        return self.controller.getRightTriggerAxis() > 0.1

    def turret_aim_hub(self):
        return self.controller.getPOV() == 0    # D-pad up → hub

    def turret_aim_left(self):
        return self.controller.getPOV() == 270  # D-pad left → left lob

    def turret_aim_right(self):
        return self.controller.getPOV() == 90   # D-pad right → right lob
