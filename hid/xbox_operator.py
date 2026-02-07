import wpilib
from wpilib import XboxController

pn = wpilib.SmartDashboard.putNumber
gn = wpilib.SmartDashboard.getNumber
pb = wpilib.SmartDashboard.putBoolean
gb = wpilib.SmartDashboard.getBoolean


class RebuiltOperator:

    def __init__(self) -> None:
        self.controller = XboxController(1)
        pb('/hid/operator/shooter_on', False)
        pb('/hid/operator/shooter_reverse', False)
        pb('/hid/operator/singulator_on', False)
        pb('/hid/operator/singulator_reverse', False)
        pb('/hid/operator/intake_on', False)
        pb('/hid/operator/intake_reverse', False)

    def shooter_on(self) -> bool:
        return self.controller.getRightBumperButtonPressed() or gb('/hid/operator/shooter_on', False)

    def intake_on(self) -> bool:
        return self.controller.getRightTriggerAxis() > 0.1 or gb('/hid/operator/intake_on', False)

    def singulator_on(self) -> bool:
        return False or gb('/hid/operator/singulator_on', False)

    def shooter_revserse(self) -> bool:
        return False or gb('/hid/operator/shooter_reverse', False)

    def intake_revserse(self) -> bool:
        return False or gb('/hid/operator/intake_reverse', False)

    def singulator_revserse(self) -> bool:
        return False or gb('/hid/operator/singulator_reverse', False)
