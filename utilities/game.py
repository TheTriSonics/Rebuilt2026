import wpilib
from wpilib import SendableChooser, SmartDashboard as _SD


# This will default to the blue alliance if a proper link to the driver station has not yet been established
def is_red() -> bool:
    return wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed


_side_chooser: SendableChooser | None = None


def init_side_chooser() -> None:
    global _side_chooser
    _side_chooser = SendableChooser()
    _side_chooser.setDefaultOption("Right", False)
    _side_chooser.addOption("Left", True)
    _SD.putData("Field Side", _side_chooser)


def is_left() -> bool:
    if _side_chooser is None:
        return False
    return bool(_side_chooser.getSelected())


def is_match() -> bool:
    return wpilib.DriverStation.isFMSAttached()


def is_sim() -> bool:
    return wpilib.RobotBase.isSimulation()


def is_auton() -> bool:
    mode = wpilib.SmartDashboard.getString('/robot/mode', '')
    return mode in ['auto']


def is_disabled() -> bool:
    mode = wpilib.SmartDashboard.getString('/robot/mode', '')
    return mode in ['disabled', '']
