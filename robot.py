import wpilib

class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        """Robot initialization function"""
        pass

    def autonomousInit(self):
        """Executed when autonomous mode starts"""
        pass

    def autonomousPeriodic(self):
        """Called periodically during autonomous"""
        pass

    def teleopInit(self):
        """Executed when teleop mode starts"""
        pass

    def teleopPeriodic(self):
        """Called periodically during teleop"""
        print('running teleop!')
        pass

if __name__ == "__main__":
    wpilib.run(MyRobot)
