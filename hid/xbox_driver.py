from wpilib import XboxController
from utilities.game import is_sim



def is_linux_sim():
    import platform
    return platform.system() == "Linux" and is_sim()


class RebuiltDriver:

    def __init__(self) -> None:
        self.controller = XboxController(0)

    def get_right_x(self):
        if is_linux_sim():
            return self.controller.getRawAxis(3)
        return self.controller.getRightX()

    def get_right_y(self):
        return self.controller.getRightY()

    def get_left_x(self):
        if is_linux_sim():
            return self.controller.getRawAxis(0)
        return self.controller.getLeftX()

    def get_left_y(self):
        return self.controller.getLeftY()

    def field_centric(self):
        return self.controller.getRawButtonPressed(8)

    def robot_centric(self):
        return self.controller.getRawButtonPressed(7)

    def target_lob_left(self):
        if not self.controller.getRightBumperButton() and self.controller.getLeftBumperButton():
            return True
        return False

    def target_lob_right(self):
        if not self.controller.getLeftBumperButton() and self.controller.getRightBumperButton():
            return True
        return False

    def target_hub(self):
        if self.controller.getRightBumperButton() and self.controller.getLeftBumperButton():
            return True
        return False

    def set_heading_to_vision(self):
        return self.controller.getLeftTriggerAxis() > 0.5

    # def shooter_shoot(self):
    #     return self.controller.getRightBumperButton()

    # def intake_on(self):
    #     return self.controller.getRightTriggerAxis() > 0.1

    # def singulating(self):
    #     return self.controller.getLeftTriggerAxis() > 0.1

    # def go_to_point(self):
    #     return self.controller.getXButtonPressed()

    def reset_yaw(self):
        return self.controller.getPOV() == 180

    def turret_left(self) -> float:
        return 0.6 if self.controller.getPOV() == 270 else 0.0

    def turret_right(self) -> float:
        return -0.6 if self.controller.getPOV() == 90 else 0.0

    def intake_up(self):
        return self.controller.getYButtonPressed()


