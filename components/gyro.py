import math
from collections import deque

from magicbot import feedback, tunable
from phoenix6.hardware import Pigeon2
from wpimath.geometry import Rotation2d

from ids import CanId
from utilities.scalers import clamp_degrees


class GyroComponent:
    bump_roll_threshold = tunable(5.0)  # degrees
    bump_pitch_threshold = tunable(5.0)  # degrees
    bump_z_accel_threshold = tunable(0.7)  # g's — below this, wheels likely airborne

    # --- Drift detection tuning constants ---
    # These control when we declare odometry unreliable.
    # Based on MIBKN log analysis: auton z_accel std ~0.18g,
    # normal teleop ~0.8g, pre-drift ~1.02g.
    DRIFT_Z_ACCEL_STD_THRESHOLD = tunable(0.85)  # g — z_accel std dev above this = rough
    DRIFT_SPEED_THRESHOLD = tunable(2.0)  # m/s — only flag drift when moving fast
    DRIFT_WINDOW_SIZE = 15  # samples (~300ms at 50Hz)
    DRIFT_COOLDOWN_CYCLES = 25  # stay in drift mode for 0.5s after last detection

    def __init__(self):
        self.pigeon = Pigeon2(CanId.PIGEON.id, CanId.PIGEON.bus)
        self.offset = 0.0

        # Rolling window for z_accel drift detection
        self._z_accel_history: deque[float] = deque(maxlen=self.DRIFT_WINDOW_SIZE)
        self._drift_detected = False
        self._drift_cooldown = 0

    def reset_heading(self, heading: float = 0.0) -> None:
        self.offset = heading - self.get_heading()

    @feedback
    def get_heading(self) -> float:
        return clamp_degrees(self.pigeon.get_yaw().value + self.offset)

    def get_Rotation2d(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self.get_heading())

    @feedback
    def get_roll(self) -> float:
        return self.pigeon.get_roll().value

    @feedback
    def get_pitch(self) -> float:
        return self.pigeon.get_pitch().value

    @feedback
    def get_z_accel(self) -> float:
        return self.pigeon.get_acceleration_z().value

    @feedback
    def is_on_bump(self) -> bool:
        return (
            abs(self.get_roll()) > self.bump_roll_threshold
            or abs(self.get_pitch()) > self.bump_pitch_threshold
            or self.get_z_accel() < self.bump_z_accel_threshold
        )

    def update_drift_detection(self, robot_speed: float) -> None:
        """Call once per cycle with the robot's current linear speed (m/s).

        Tracks z_accel variance over a rolling window. When the std dev
        exceeds DRIFT_Z_ACCEL_STD_THRESHOLD and the robot is moving
        faster than DRIFT_SPEED_THRESHOLD, we flag odometry as unreliable.

        A cooldown keeps the flag active briefly after the disturbance
        ends, so vision has time to correct the drift.
        """
        self._z_accel_history.append(self.get_z_accel())

        if len(self._z_accel_history) >= self.DRIFT_WINDOW_SIZE:
            vals = list(self._z_accel_history)
            mean = sum(vals) / len(vals)
            variance = sum((v - mean) ** 2 for v in vals) / len(vals)
            std_dev = math.sqrt(variance)

            if (
                std_dev > self.DRIFT_Z_ACCEL_STD_THRESHOLD
                and robot_speed > self.DRIFT_SPEED_THRESHOLD
            ):
                self._drift_detected = True
                self._drift_cooldown = self.DRIFT_COOLDOWN_CYCLES
            elif self._drift_cooldown > 0:
                self._drift_cooldown -= 1
                if self._drift_cooldown == 0:
                    self._drift_detected = False
            else:
                self._drift_detected = False

    @feedback
    def is_drift_detected(self) -> bool:
        """True when odometry is likely unreliable due to wheel slip.

        Vision should be trusted more heavily when this is True.
        """
        return self._drift_detected

    def execute(self) -> None:
        pass
