"""
Commissioning Controller — Drivetrain Characterization and PID Tuning
======================================================================

PURPOSE
-------
This module exists to validate and tune a newly built swerve drivetrain before
full robot software is deployed. It provides:

  1. SysId-style motor characterization (quasistatic ramp + dynamic step) for
     both turn (steer) and drive motors, producing data for CTRE Hoot log
     analysis in Tuner X.

  2. Heading PID tuning via repeatable snap-to-angle tests.

  3. Translation PID tuning via a drive-straight-and-stop distance test.

All tuning parameters are live-editable from the Shuffleboard "Commissioning"
tab. CTRE SignalLogger start/stop is controlled by the tech controller's Back
button (not robot enable), so you can record only the data you want.

This controller is intentionally standalone — it depends only on
DrivetrainComponent and requires no game-specific code, vision, or other
subsystems. See the Portability section at the bottom of this docstring for
how to carry it into a future robot project.


PREREQUISITES
-------------
- Third Xbox controller plugged into Driver Station USB port 2.
- CTRE Tuner X installed on the pit laptop for Hoot log analysis.
- Robot on a flat, hard floor with at least 2 metres of clear space in every
  direction (quasistatic tests need room to roll; dynamic tests need room to
  decelerate).
- Battery fully charged, ideally >12.5 V. Motor characterization results
  (especially kS and kV) are voltage-dependent — sag shifts the numbers.
  Run tests with a fresh battery and note the voltage shown in the DS.
- Robot code deployed with only DrivetrainComponent + GyroComponent active
  (i.e., the barebones robot.py on this branch).


STEP-BY-STEP TUNING PROCEDURE
------------------------------

Before starting any phase:
  • Open Shuffleboard and navigate to the "Commissioning" tab.
  • Confirm all tunable parameters look reasonable (see Dashboard Layout).
  • Enable the robot in Teleop mode.

----------------------------------------------------------------------
PHASE 1 — Turn Motor Characterization (steer motors, SysId protocol)
----------------------------------------------------------------------
Goal: extract kS, kV, kA for the steer (turn) TalonFX motors so that the
closed-loop position controller in TunerConstants._steer_gains has accurate
feedforward values.

  1. Press Back on the tech controller.
       → 'test_active' turns True; a new .hoot file starts recording.
       → Watch the DS or Shuffleboard to confirm.

  2. Press A (turn_quasistatic_fwd).
       → All steer motors ramp voltage slowly (quasi_ramp_rate_vs V/s).
       → Watch 'steer_velocity_avg' climb gradually on the graph.
       → Let it run until the timeout (max_test_duration_s) stops it, or
         press Start to abort early. The robot will spin in place.

  3. Press B (turn_quasistatic_rev).
       → Same ramp, reverse direction.

  4. Press X (turn_dynamic_fwd).
       → Step voltage (dynamic_step_voltage_v) applied instantly.
       → Watch velocity jump; the robot will spin faster.

  5. Press Y (turn_dynamic_rev).

  6. Press Back to stop the logger.
       → The .hoot file is now complete.

  7. Open Tuner X → Hoot Log Analysis → load the file.
       → Select the steer motor signals (velocity, voltage).
       → Run the SysId routine to extract kS, kV, kA.

  8. Update generated/tuner_constants_swerve.py _steer_gains Slot0:
         kS = <result>, kV = <result>, kA = <result>
     Do NOT hand-edit other values in that file.

----------------------------------------------------------------------
PHASE 2 — Drive Motor Characterization (drive motors, SysId protocol)
----------------------------------------------------------------------
Goal: extract kS, kV, kA for the drive TalonFX motors.

  1. Align the robot so it can roll straight for ~2 m.
  2. Press Back to start a new log.

  3. Press Left Bumper (drive_quasistatic_fwd).
       → Drive motors ramp while steer motors hold 0° (wheels straight).
       → Watch 'drive_velocity_avg' rise on the graph.

  4. Press Right Bumper (drive_quasistatic_rev).

  5. Press Left Trigger >50% (drive_dynamic_fwd).
       → Step voltage; robot accelerates forward quickly.

  6. Press Right Trigger >50% (drive_dynamic_rev).

  7. Press Back to stop the logger.

  8. Repeat SysId analysis in Tuner X for drive motor signals.
     Update _drive_gains Slot0: kS, kV, kA.

----------------------------------------------------------------------
PHASE 3 — Heading PID Tuning
----------------------------------------------------------------------
Goal: tune the DrivetrainComponent heading_controller (ProfiledPIDController)
so the robot snaps to a heading quickly without oscillation.

The current default gain is P=6.0 in drivetrain.py (heading_controller).
Expose it as a @tunable on DrivetrainComponent if you want live adjustments.

  1. Set snap_target_0_deg = 0, snap_target_1_deg = 90, etc. on the dashboard.
  2. Place robot facing approximately 0°.

  3. Press D-pad Up (snap_0).
       → Robot snaps to snap_target_0_deg.
       → Watch 'heading_error_deg' converge to 0 on the graph.

  Interpret the graph:
    • Oscillating (bounces past 0 and back): P is too high. Reduce by 20%.
    • Settles slowly (>1 s to reach ±1°): P is too low. Increase by 25%.
      Consider adding small D if P alone can't settle it fast enough.
    • Settles cleanly in ~0.3–0.7 s: gains are good.

  4. Test all four snap targets (D-pad Up/Right/Down/Left).
     All four should converge within HEADING_TOLERANCE (1°) in < 1 s.

  5. Record the final P (and D if used) values and update heading_controller
     in drivetrain.py.

----------------------------------------------------------------------
PHASE 4 — Translation PID Tuning
----------------------------------------------------------------------
Goal: tune the DrivetrainComponent path_pid_control (PIDController, P=7.0)
so autonomous path-following stops accurately at waypoints.

  1. Mark a start point on the floor with tape.
  2. Set translation_distance_m = 1.0 on the dashboard.

  3. Press Left Stick Button (button 9).
       → Robot drives forward at 1 m/s and stops when odometry reports 1 m
         of travel (or when max_test_duration_s elapses).

  4. Measure the actual stopping distance with a tape measure.

  Interpret the result:
    • Overshot (traveled > 1 m): reduce path_pid_control P.
    • Undershot (< 1 m) or stop was sluggish: increase P.
    • Accurate within ~3 cm: P is good.

  5. Repeat from the same start mark. Aim for < ±3 cm repeatability.
  6. Update path_pid_control P in drivetrain.py.


DASHBOARD LAYOUT (Shuffleboard "Commissioning" tab)
----------------------------------------------------
Create a new tab called "Commissioning". All values live under the NT path
/components/commissioning/ (auto-published by MagicBot).

  Status row:
    current_state      → Text Display    (shows active state name)
    test_active        → Boolean Box     (green = test running)
    elapsed_time       → Number Display  (seconds in current state)

  Telemetry (add as Line Graph, group on one chart):
    applied_voltage    → Line Graph
    drive_velocity_avg → Line Graph
    steer_velocity_avg → Line Graph

  Heading graph:
    heading_error_deg  → Line Graph

  Per-module velocity (add as Line Graph to spot asymmetric friction):
    drive_vel_fl, drive_vel_fr, drive_vel_bl, drive_vel_br → Line Graph

  Tunable parameters (Number Inputs / Sliders):
    quasi_ramp_rate_vs       default 0.5  V/s
    dynamic_step_voltage_v   default 4.0  V
    max_test_duration_s      default 5.0  s
    max_voltage_v            default 7.0  V   (controller-level cap)
    target_module_index      default 4        (0-3 = single module, 4 = all)
    translation_distance_m   default 1.0  m
    snap_target_0_deg        default   0  deg
    snap_target_1_deg        default  90  deg
    snap_target_2_deg        default 180  deg
    snap_target_3_deg        default 270  deg

  DrivetrainComponent also publishes:
    /components/drivetrain/max_char_voltage  (hardware-level voltage cap, default 7.0 V)


PORTABILITY — Using This Module on a Future Robot
-------------------------------------------------
This controller and its HID file are intentionally decoupled from game-year
code. To reuse them in a future MagicBot swerve project:

  1. Copy controllers/commissioning.py and hid/xbox_tech.py verbatim.

  2. In the new project's DrivetrainComponent, add:
       a. Class-level flag:
            _characterization_active: bool = False
          Reset it in on_enable():
            self._characterization_active = False

       b. Class-level tunable:
            max_char_voltage = magicbot.tunable(7.0)

       c. In execute(), wrap the module.set() loop:
            if not self._characterization_active:
                for state, module in zip(desired_states, self.modules, strict=True):
                    module.set(state)
                    module.publish_telemetry()
            else:
                for module in self.modules:
                    module.publish_telemetry()

       d. Add these public methods (see components/drivetrain.py for reference):
            begin_characterization()
            end_characterization()
            stop_characterization()
            apply_drive_voltage(volts, module_index=4)
            apply_steer_voltage(volts, module_index=4)
            get_module_drive_velocities() -> list[float]
            get_module_steer_velocities() -> list[float]

          The controller also uses these pre-existing drivetrain methods:
            snap_to_heading(radians)
            is_heading_aligned() -> bool
            stop_snapping()
            drive_local(vx, vy, omega)
            get_pose() -> Pose2d

  3. In robot.py:
       • Declare  commissioning: Commissioning  as the first (sole) controller.
       • In createObjects(): self._signal_logger_running = False
       • In teleopInit(): instantiate TechController(), call commissioning.engage()
       • In teleopPeriodic(): route button events (see robot.py on this branch).
       • In disabledPeriodic(): self.drivetrain.update_odometry()
"""

import math

from magicbot import StateMachine, feedback, state, tunable
from phoenix6.controls import PositionVoltage
from phoenix6 import SignalLogger

from components.drivetrain import DrivetrainComponent


class Commissioning(StateMachine):
    """Drivetrain characterization and PID tuning state machine.

    See the module docstring above for full usage instructions.
    Inject drivetrain via MagicBot's type-annotation injection.
    """

    drivetrain: DrivetrainComponent

    # --- Tunable parameters (live-editable from Shuffleboard) ---

    # Characterization test settings
    quasi_ramp_rate_vs = tunable(0.5)  # V/s for quasistatic ramp
    dynamic_step_voltage_v = tunable(4.0)  # Step voltage for dynamic tests
    max_test_duration_s = tunable(5.0)  # Hard timeout per test (s)
    max_voltage_v = tunable(7.0)  # Controller-level voltage cap (V)

    # Module selector: 0-3 = individual module, 4 = all modules
    target_module_index = tunable(4)

    # Translation test
    translation_distance_m = tunable(1.0)

    # Heading snap targets (degrees — converted to radians in the state)
    snap_target_0_deg = tunable(0.0)
    snap_target_1_deg = tunable(90.0)
    snap_target_2_deg = tunable(180.0)
    snap_target_3_deg = tunable(270.0)

    # --- Internal bookkeeping (not MagicBot-managed) ---

    def __init__(self) -> None:
        super().__init__()
        self._current_voltage: float = 0.0
        self._drive_velocity_avg: float = 0.0
        self._steer_velocity_avg: float = 0.0
        self._heading_error: float = 0.0
        self._elapsed_time: float = 0.0
        self._snap_target_rad: float = 0.0
        self._test_start_x: float = 0.0
        self._test_start_y: float = 0.0
        self._module_drive_vels: list[float] = [0.0, 0.0, 0.0, 0.0]
        # Reusable PositionVoltage request for wheel-alignment during drive tests
        self._steer_align_request = PositionVoltage(0.0)

    # --- Feedback properties (published to NT under /components/commissioning/) ---

    @feedback
    def applied_voltage(self) -> float:
        return self._current_voltage

    @feedback
    def drive_velocity_avg(self) -> float:
        return self._drive_velocity_avg

    @feedback
    def steer_velocity_avg(self) -> float:
        return self._steer_velocity_avg

    @feedback
    def heading_error_deg(self) -> float:
        return math.degrees(self._heading_error)

    @feedback
    def elapsed_time(self) -> float:
        return self._elapsed_time

    @feedback
    def test_active(self) -> bool:
        return self.is_executing

    @feedback
    def drive_vel_fl(self) -> float:
        return self._module_drive_vels[0]

    @feedback
    def drive_vel_fr(self) -> float:
        return self._module_drive_vels[1]

    @feedback
    def drive_vel_bl(self) -> float:
        return self._module_drive_vels[2]

    @feedback
    def drive_vel_br(self) -> float:
        return self._module_drive_vels[3]

    # --- Internal helpers ---

    def _clamp_voltage(self, volts: float) -> float:
        limit = abs(self.max_voltage_v)
        return max(-limit, min(limit, volts))

    def _update_drive_telemetry(self) -> None:
        vels = self.drivetrain.get_module_drive_velocities()
        self._module_drive_vels = list(vels)
        self._drive_velocity_avg = sum(vels) / len(vels) if vels else 0.0

    def _update_steer_telemetry(self) -> None:
        vels = self.drivetrain.get_module_steer_velocities()
        self._steer_velocity_avg = sum(vels) / len(vels) if vels else 0.0

    def _check_timeout(self, state_tm: float) -> bool:
        """Return True (and transition to idle) if the test has timed out."""
        if state_tm > self.max_test_duration_s:
            self.next_state(self.idle)
            return True
        return False

    def _align_wheels_forward(self) -> None:
        """Command all steer motors to 0 rotation (wheels straight ahead).

        Called on initial_call of drive characterization states so the wheel
        direction is consistent before the drive voltage ramp begins. Uses the
        module's existing steer motor directly via PositionVoltage(0).
        """
        for module in self.drivetrain.modules:
            module.steer.set_control(self._steer_align_request)

    # --- done() override ---

    def done(self) -> None:
        self._current_voltage = 0.0
        self.drivetrain.end_characterization()
        super().done()

    # =========================================================================
    # STATES
    # =========================================================================

    @state(first=True, must_finish=True)
    def idle(self, initial_call: bool) -> None:
        """Idle — all motors zeroed, waiting for a test command."""
        if initial_call:
            SignalLogger.write_string("state", "none")
            self._current_voltage = 0.0
            self._elapsed_time = 0.0
            self.drivetrain.end_characterization()

    # --- Drive motor characterization ---

    @state(must_finish=True)
    def drive_quasistatic_fwd(self, initial_call: bool, state_tm: float) -> None:
        """Drive quasistatic forward — slowly ramp drive voltage, wheels straight."""
        if initial_call:
            SignalLogger.write_string("state", "quasistatic-forward")
            self.drivetrain.begin_characterization()
            self._align_wheels_forward()
        if self._check_timeout(state_tm):
            return
        volts = self._clamp_voltage(self.quasi_ramp_rate_vs * state_tm)
        self.drivetrain.apply_drive_voltage(volts, self.target_module_index)
        self._current_voltage = volts
        self._elapsed_time = state_tm
        self._update_drive_telemetry()

    @state(must_finish=True)
    def drive_quasistatic_rev(self, initial_call: bool, state_tm: float) -> None:
        """Drive quasistatic reverse — slowly ramp drive voltage negative, wheels straight."""
        if initial_call:
            SignalLogger.write_string("state", "quasistatic-reverse")
            self.drivetrain.begin_characterization()
            self._align_wheels_forward()
        if self._check_timeout(state_tm):
            return
        volts = self._clamp_voltage(-self.quasi_ramp_rate_vs * state_tm)
        self.drivetrain.apply_drive_voltage(volts, self.target_module_index)
        self._current_voltage = volts
        self._elapsed_time = state_tm
        self._update_drive_telemetry()

    @state(must_finish=True)
    def drive_dynamic_fwd(self, initial_call: bool, state_tm: float) -> None:
        """Drive dynamic forward — apply constant step voltage, wheels straight."""
        if initial_call:
            SignalLogger.write_string("state", "dynamic-forward")
            self.drivetrain.begin_characterization()
            self._align_wheels_forward()
        if self._check_timeout(state_tm):
            return
        volts = self._clamp_voltage(self.dynamic_step_voltage_v)
        self.drivetrain.apply_drive_voltage(volts, self.target_module_index)
        self._current_voltage = volts
        self._elapsed_time = state_tm
        self._update_drive_telemetry()

    @state(must_finish=True)
    def drive_dynamic_rev(self, initial_call: bool, state_tm: float) -> None:
        """Drive dynamic reverse — apply constant negative step voltage, wheels straight."""
        if initial_call:
            SignalLogger.write_string("state", "dynamic-reverse")
            self.drivetrain.begin_characterization()
            self._align_wheels_forward()
        if self._check_timeout(state_tm):
            return
        volts = self._clamp_voltage(-self.dynamic_step_voltage_v)
        self.drivetrain.apply_drive_voltage(volts, self.target_module_index)
        self._current_voltage = volts
        self._elapsed_time = state_tm
        self._update_drive_telemetry()

    # --- Turn (steer) motor characterization ---

    @state(must_finish=True)
    def turn_quasistatic_fwd(self, initial_call: bool, state_tm: float) -> None:
        """Turn quasistatic forward — slowly ramp steer voltage, drives zeroed."""
        if initial_call:
            SignalLogger.write_string("state", "quasistatic-forward")
            self.drivetrain.begin_characterization()
            self.drivetrain.apply_drive_voltage(0.0)
        if self._check_timeout(state_tm):
            return
        volts = self._clamp_voltage(self.quasi_ramp_rate_vs * state_tm)
        self.drivetrain.apply_steer_voltage(volts, self.target_module_index)
        self._current_voltage = volts
        self._elapsed_time = state_tm
        self._update_steer_telemetry()

    @state(must_finish=True)
    def turn_quasistatic_rev(self, initial_call: bool, state_tm: float) -> None:
        """Turn quasistatic reverse — slowly ramp steer voltage negative, drives zeroed."""
        if initial_call:
            SignalLogger.write_string("state", "quasistatic-reverse")
            self.drivetrain.begin_characterization()
            self.drivetrain.apply_drive_voltage(0.0)
        if self._check_timeout(state_tm):
            return
        volts = self._clamp_voltage(-self.quasi_ramp_rate_vs * state_tm)
        self.drivetrain.apply_steer_voltage(volts, self.target_module_index)
        self._current_voltage = volts
        self._elapsed_time = state_tm
        self._update_steer_telemetry()

    @state(must_finish=True)
    def turn_dynamic_fwd(self, initial_call: bool, state_tm: float) -> None:
        """Turn dynamic forward — apply constant step voltage to steer, drives zeroed."""
        if initial_call:
            SignalLogger.write_string("state", "dynamic-forward")
            self.drivetrain.begin_characterization()
            self.drivetrain.apply_drive_voltage(0.0)
        if self._check_timeout(state_tm):
            return
        volts = self._clamp_voltage(self.dynamic_step_voltage_v)
        self.drivetrain.apply_steer_voltage(volts, self.target_module_index)
        self._current_voltage = volts
        self._elapsed_time = state_tm
        self._update_steer_telemetry()

    @state(must_finish=True)
    def turn_dynamic_rev(self, initial_call: bool, state_tm: float) -> None:
        """Turn dynamic reverse — apply constant negative step voltage to steer, drives zeroed."""
        if initial_call:
            SignalLogger.write_string("state", "dynamic-reverse")
            self.drivetrain.begin_characterization()
            self.drivetrain.apply_drive_voltage(0.0)
        if self._check_timeout(state_tm):
            return
        volts = self._clamp_voltage(-self.dynamic_step_voltage_v)
        self.drivetrain.apply_steer_voltage(volts, self.target_module_index)
        self._current_voltage = volts
        self._elapsed_time = state_tm
        self._update_steer_telemetry()

    # --- Heading PID tuning ---

    @state(must_finish=True)
    def heading_snap_test(self, initial_call: bool, state_tm: float) -> None:
        """Snap to _snap_target_rad and hold until aligned or timeout.

        Uses normal drivetrain execute() — do NOT call begin_characterization().
        The heading controller in DrivetrainComponent drives the robot.
        """
        if initial_call:
            self.drivetrain.snap_to_heading(self._snap_target_rad)
        self._heading_error = self._snap_target_rad - self.drivetrain.get_rotation().radians()
        self._elapsed_time = state_tm
        if self.drivetrain.is_heading_aligned() or self._check_timeout(state_tm):
            self.drivetrain.stop_snapping()
            self.next_state(self.idle)

    # --- Translation PID tuning ---

    @state(must_finish=True)
    def translation_test(self, initial_call: bool, state_tm: float) -> None:
        """Drive straight for translation_distance_m then stop.

        Uses drive_local() so the robot moves in its own forward direction
        regardless of field orientation. Odometry measures distance traveled.
        """
        if initial_call:
            pose = self.drivetrain.get_pose()
            self._test_start_x = pose.x
            self._test_start_y = pose.y
        if self._check_timeout(state_tm):
            self.drivetrain.halt()
            return
        pose = self.drivetrain.get_pose()
        dx = pose.x - self._test_start_x
        dy = pose.y - self._test_start_y
        traveled = math.sqrt(dx * dx + dy * dy)
        self._elapsed_time = state_tm
        if traveled >= self.translation_distance_m:
            self.drivetrain.halt()
            self.next_state(self.idle)
        else:
            self.drivetrain.drive_local(1.0, 0.0, 0.0)

    # =========================================================================
    # PUBLIC ENTRY METHODS (called from robot.py teleopPeriodic)
    # =========================================================================

    def go_idle(self) -> None:
        """Abort current test and return to idle."""
        self.next_state_now(self.idle)

    def go_drive_quasistatic_fwd(self) -> None:
        self.engage(initial_state=self.drive_quasistatic_fwd, force=True)

    def go_drive_quasistatic_rev(self) -> None:
        self.engage(initial_state=self.drive_quasistatic_rev, force=True)

    def go_drive_dynamic_fwd(self) -> None:
        self.engage(initial_state=self.drive_dynamic_fwd, force=True)

    def go_drive_dynamic_rev(self) -> None:
        self.engage(initial_state=self.drive_dynamic_rev, force=True)

    def go_turn_quasistatic_fwd(self) -> None:
        self.engage(initial_state=self.turn_quasistatic_fwd, force=True)

    def go_turn_quasistatic_rev(self) -> None:
        self.engage(initial_state=self.turn_quasistatic_rev, force=True)

    def go_turn_dynamic_fwd(self) -> None:
        self.engage(initial_state=self.turn_dynamic_fwd, force=True)

    def go_turn_dynamic_rev(self) -> None:
        self.engage(initial_state=self.turn_dynamic_rev, force=True)

    def go_heading_snap_test(self, heading_deg: float) -> None:
        """Snap to the given heading (degrees). Call before engaging."""
        self._snap_target_rad = math.radians(heading_deg)
        self.engage(initial_state=self.heading_snap_test, force=True)

    def go_translation_test(self) -> None:
        self.engage(initial_state=self.translation_test, force=True)
