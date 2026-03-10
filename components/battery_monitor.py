from wpilib import RobotController
from magicbot import tunable, feedback


class BatteryMonitorComponent:
    warning_voltage = tunable(11.5)
    stop_voltage = tunable(11.0)

    warning_active = tunable(False)
    stop_active = tunable(False)

    def is_warning_active(self) -> bool:
        return self.warning_active

    def is_stop_active(self) -> bool:
        return self.stop_active

    @feedback
    def get_current_voltage(self) -> float:
        return RobotController.getBatteryVoltage()

    def execute(self):
        v = self.get_current_voltage()
        self.warning_active = v < self.warning_voltage
        self.stop_active = v < self.stop_voltage
