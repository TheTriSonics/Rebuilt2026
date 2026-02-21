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

    def shooter(self):
        return self.controller.getRightBumperButton()

    def intake_on(self):
        return self.controller.getRightTriggerAxis() > 0.1

    def intake_reverse(self):
        return self.controller.getLeftTriggerAxis() > 0.1

    def go_to_point(self):
        return self.controller.getXButtonPressed()

    def reset_yaw(self):
        return self.controller.getPOV() == 180




