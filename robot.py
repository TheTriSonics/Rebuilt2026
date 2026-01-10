import magicbot


class MyRobot(magicbot.MagicRobot):
    # Declare components and controllers here

    def createObjects(self):
        # Create logging and such here; actual robot components are above
        ...

    def autonomousInit(self): ...

    def autonomousPeriodic(self):
        # MagicBot handles periodic execution; this never runs but is left as
        # a placeholder to keep others from trying to implement it thinking
        # it will be called.
        ...

    def teleopInit(self): ...

    def teleopPeriodic(self): 
        print('teleop running')

    def disabledPeriodic(self):
        print('robot awaiting instructions')

