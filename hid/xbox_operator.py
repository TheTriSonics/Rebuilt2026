from wpilib import XboxController


class RebuiltOperator:

    def __init__(self) -> None:
        self.controller = XboxController(1)

    def intake_idle(self):
        return self.controller.getLeftBumperButtonPressed()
    
    def eject(self):
        return self.controller.getXButtonPressed()
    
    def fixed_shot(self):
        return self.controller.getLeftTriggerAxis() > 0.1
    
    def shooter_shoot(self):
        return self.controller.getRightBumperButtonPressed()

    def shooter_off(self):
        return self.controller.getAButtonPressed()

    def intake_in(self):
        return self.controller.getYButtonPressed()

    # def intake_out(self):
    #     return self.controller.getBButtonPressed()
    
    def intake_on(self):
        return self.controller.getRightTriggerAxis() > 0.1

    def turret_aim_hub(self):
        return self.controller.getPOV() == 0    # D-pad up → hub

    def turret_aim_left(self):
        return self.controller.getPOV() == 270  # D-pad left → left lob

    def turret_aim_right(self):
        return self.controller.getPOV() == 90   # D-pad right → right lob
    
    def shooter_spin_up(self):
        return self.controller.getBButtonPressed()
    
    def rumble_on(self):
        self.controller.setRumble(XboxController.RumbleType.kLeftRumble, 1.0)
        self.controller.setRumble(XboxController.RumbleType.kRightRumble, 1.0)
    
    def rumble_off(self):
        self.controller.setRumble(XboxController.RumbleType.kLeftRumble, 0.0)
        self.controller.setRumble(XboxController.RumbleType.kRightRumble, 0.0)
