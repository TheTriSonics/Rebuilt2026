import wpilib
from sys import platform


# This will default to the blue alliance if a proper link to the driver station has not yet been established
def is_red() -> bool:
    return wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed


def is_match() -> bool:
    return wpilib.DriverStation.isFMSAttached()


def is_sim() -> bool:
    return wpilib.RobotBase.isSimulation()

def is_linux_sim() -> bool:
    return is_sim() and platform.startswith("linux")


def is_auton() -> bool:
    mode = wpilib.SmartDashboard.getString('/robot/mode', '')
    return mode in ['auto']


def is_disabled() -> bool:
    mode = wpilib.SmartDashboard.getString('/robot/mode', '')
    return mode in ['disabled', '']
