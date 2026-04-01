"""
MotorCommissioning Controller — Generic Motor Characterization and Limit Setup
==============================================================================

PURPOSE
-------
A reusable commissioning controller for any 1–2 TalonFX motors on the robot.
Supports:

  1. SysId-style motor characterization (quasistatic ramp + dynamic step) for
     extracting kS, kV, kA feedforward constants via CTRE Tuner X hoot analysis.

  2. Physical limit saving — put motors in coast mode, manually move the mechanism
     to each limit, and press the dashboard buttons to record and persist the
     encoder position at each limit.

DASHBOARD SETUP
---------------
All configuration is live-editable from Shuffleboard under the
"MotorCommissioning" tab.

  Motor Setup
  -----------
  motor1_label      — Display name only (e.g., "Left Arm"). Does not affect
                      motor lookup; CAN ID is used for that.
  motor1_can_id     — CAN ID of the first motor (integer).
  motor1_bus        — CAN bus name: "Drive" or "Shooter".
  motor1_inverted   — Flip motor1 direction.

  motor2_enabled    — Set True to use a second motor.
  motor2_label      — Display name for motor 2.
  motor2_can_id     — CAN ID of the second motor.
  motor2_bus        — CAN bus name for motor 2.
  motor2_inverted   — Typically True for opposed-side mechanisms (mirror image).
                      Both motors receive the same commanded voltage; inversion
                      is baked into the motor config so the mechanism moves in
                      one direction when positive voltage is applied to both.

  reinitialize      — Press (set True on dashboard) to rebuild motor objects
                      after changing any CAN ID, bus, or inversion setting.
                      Safe to press anytime.

  Encoder Setup
  -------------
  encoder_type      — "internal" (motor's built-in encoder), "external"
                      (standalone CANcoder), or "none" (no position feedback).
  encoder_can_id    — CAN ID of the CANcoder (external only).
  encoder_bus       — CAN bus for the CANcoder.

  SysId Parameters
  ----------------
  quasi_ramp_rate_vs      — Voltage ramp rate for quasistatic tests (V/s).
  dynamic_step_voltage_v  — Step voltage for dynamic tests (V).
  max_test_duration_s     — Hard timeout per test (s); auto-returns to idle.
  max_voltage_v           — Absolute voltage cap applied to all outputs (V).

  Limit Setup
  -----------
  save_upper_limit  — Press (set True on dashboard) while at the upper physical
                      limit to record and persist the current encoder position.
  save_lower_limit  — Same for lower physical limit.
  upper_limit_rot   — Stored upper limit value (rotations). Readable on dashboard.
  lower_limit_rot   — Stored lower limit value (rotations). Readable on dashboard.

  Limits are persisted to /home/lvuser/motor_limits.json and reloaded on startup.

STEP-BY-STEP PROCEDURE
-----------------------

--- Motor Characterization ---
  1. Set motor1_can_id (and motor1_bus if not "Shooter"). Press reinitialize.
     Confirm motors_initialized turns True in the feedback section.

  2. Enable the robot in Teleop mode. Press Back on TechController2 (DS port 3)
     to start the SignalLogger.

  3. Press A → quasistatic forward sweep.
     Press B → quasistatic reverse sweep.
     Press X → dynamic forward step.
     Press Y → dynamic reverse step.
     Each test runs until max_test_duration_s elapses, then auto-returns to idle.
     Press Start at any time to abort a test.

  4. Press Back to stop the logger.

  5. Open CTRE Tuner X → Hoot Log Analysis → load the .hoot file.
     Select the motor's signals (velocity, voltage). Run the SysId routine to
     extract kS, kV, kA. Apply the results to the motor's Slot0Configs.

--- Limit Setup ---
  1. Disable the robot (or keep enabled — motors are in COAST while idle).
     With motors in coast mode, physically push the mechanism to the upper
     physical stop.

  2. Enable Teleop, then set save_upper_limit = True on the dashboard.
     → upper_limit_rot updates to the current encoder position and is
       written to /home/lvuser/motor_limits.json.

  3. Move mechanism to lower stop, then set save_lower_limit = True.
     → lower_limit_rot updates and is persisted.

  4. On next robot boot the limits are automatically reloaded from disk.

NOTE ON MOTOR IDENTIFICATION
-----------------------------
CTRE Phoenix 6 does not expose a runtime API for looking up motors by their
Tuner X "device name". Motors are identified by CAN ID + bus name. Use the
motor1_label / motor2_label tunables as human-readable notes on the dashboard
(they are purely cosmetic and have no effect on motor configuration).
"""

import json
import pathlib

import wpilib
from magicbot import StateMachine, feedback, state, tunable
from phoenix6 import SignalLogger
from phoenix6.configs import TalonFXConfiguration
from phoenix6.controls import VoltageOut
from phoenix6.hardware import CANcoder, TalonFX
from phoenix6.signals import InvertedValue, NeutralModeValue

from utilities.game import is_sim


_LIMITS_FILE = pathlib.Path("/home/lvuser/motor_limits.json")


class MotorCommissioning(StateMachine):
    """Generic motor characterization and limit-setup state machine.

    Requires no component injection — all motors are instantiated directly
    from dashboard-configured CAN IDs. Call reinitialize after changing
    any CAN ID or bus setting.
    """

    # --- Motor 1 ---
    motor1_label = tunable("Motor 1")  # display name only
    motor1_can_id = tunable(0)
    motor1_bus = tunable("Shooter")
    motor1_inverted = tunable(False)

    # --- Motor 2 ---
    motor2_enabled = tunable(False)
    motor2_label = tunable("Motor 2")
    motor2_can_id = tunable(0)
    motor2_bus = tunable("Shooter")
    motor2_inverted = tunable(True)  # default True: opposed-side convention

    # --- Reinitialize button ---
    reinitialize = tunable(False)

    # --- Encoder ---
    # encoder_type is a SendableChooser dropdown — see __init__
    encoder_can_id = tunable(0)
    encoder_bus = tunable("Shooter")

    # --- SysId parameters ---
    quasi_ramp_rate_vs = tunable(0.5)
    dynamic_step_voltage_v = tunable(4.0)
    max_test_duration_s = tunable(5.0)
    max_voltage_v = tunable(7.0)

    # --- Limit control (momentary dashboard buttons + stored values) ---
    save_upper_limit = tunable(False)
    save_lower_limit = tunable(False)
    upper_limit_rot = tunable(0.0)
    lower_limit_rot = tunable(0.0)

    # --- Dashboard buttons (momentary — set True to trigger, auto-reset after one loop) ---
    dash_emergency_stop = tunable(False)
    dash_quasistatic_fwd = tunable(False)
    dash_quasistatic_rev = tunable(False)
    dash_dynamic_fwd = tunable(False)
    dash_dynamic_rev = tunable(False)

    def __init__(self) -> None:
        super().__init__()
        self._motor1: TalonFX | None = None
        self._motor2: TalonFX | None = None
        self._ext_encoder: CANcoder | None = None
        self._motors_initialized: bool = False
        self._first_idle: bool = True
        self._current_voltage: float = 0.0
        self._current_position_rot: float = 0.0
        self._current_velocity_rps: float = 0.0
        self._elapsed_time: float = 0.0

        self._encoder_chooser: wpilib.SendableChooser = wpilib.SendableChooser()
        self._encoder_chooser.setDefaultOption("Internal (motor encoder)", "internal")
        self._encoder_chooser.addOption("External (CANcoder)", "external")
        self._encoder_chooser.addOption("None", "none")
        wpilib.SmartDashboard.putData("motor_commissioning/encoder_type", self._encoder_chooser)

    # =========================================================================
    # FEEDBACK (published to NT)
    # =========================================================================

    @feedback
    def current_position_rot(self) -> float:
        return self._current_position_rot

    @feedback
    def current_velocity_rps(self) -> float:
        return self._current_velocity_rps

    @feedback
    def applied_voltage(self) -> float:
        return self._current_voltage

    @feedback
    def motors_initialized(self) -> bool:
        return self._motors_initialized

    @feedback
    def elapsed_time(self) -> float:
        return self._elapsed_time

    # =========================================================================
    # INTERNAL HELPERS
    # =========================================================================

    def _init_motors(self) -> None:
        """Create/recreate TalonFX and optional CANcoder from current tunable values."""
        self._motors_initialized = False

        m1_cfg = TalonFXConfiguration()
        m1_cfg.motor_output.neutral_mode = NeutralModeValue.COAST
        m1_cfg.motor_output.inverted = (
            InvertedValue.CLOCKWISE_POSITIVE
            if self.motor1_inverted
            else InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        )
        self._motor1 = TalonFX(self.motor1_can_id, self.motor1_bus)
        self._motor1.configurator.apply(m1_cfg)
        self._motor1.set_control(VoltageOut(0.0))

        if self.motor2_enabled:
            m2_cfg = TalonFXConfiguration()
            m2_cfg.motor_output.neutral_mode = NeutralModeValue.COAST
            m2_cfg.motor_output.inverted = (
                InvertedValue.CLOCKWISE_POSITIVE
                if self.motor2_inverted
                else InvertedValue.COUNTER_CLOCKWISE_POSITIVE
            )
            self._motor2 = TalonFX(self.motor2_can_id, self.motor2_bus)
            self._motor2.configurator.apply(m2_cfg)
            self._motor2.set_control(VoltageOut(0.0))
        else:
            self._motor2 = None

        if self._enc_type() == "external":
            self._ext_encoder = CANcoder(self.encoder_can_id, self.encoder_bus)
        else:
            self._ext_encoder = None

        self._motors_initialized = True

    def _enc_type(self) -> str:
        """Return the selected encoder type string, defaulting to 'internal'."""
        return self._encoder_chooser.getSelected() or "internal"

    def _get_encoder_position(self) -> float:
        """Return current position in rotations from the configured encoder."""
        enc = self._enc_type()
        if enc == "external" and self._ext_encoder is not None:
            return self._ext_encoder.get_absolute_position().value
        if enc == "internal" and self._motor1 is not None:
            return self._motor1.get_position().value
        return 0.0

    def _get_encoder_velocity(self) -> float:
        """Return current velocity in rotations/s from the configured encoder."""
        enc = self._enc_type()
        if enc == "external" and self._ext_encoder is not None:
            return self._ext_encoder.get_velocity().value
        if enc == "internal" and self._motor1 is not None:
            return self._motor1.get_velocity().value
        return 0.0

    def _apply_voltage(self, volts: float) -> None:
        """Apply clamped voltage to all configured motors."""
        if self._motor1 is None:
            return
        capped = max(-abs(self.max_voltage_v), min(abs(self.max_voltage_v), volts))
        self._motor1.set_control(VoltageOut(capped))
        if self._motor2 is not None:
            self._motor2.set_control(VoltageOut(capped))
        self._current_voltage = capped

    def _zero_motors(self) -> None:
        """Send VoltageOut(0) to all configured motors."""
        if self._motor1 is not None:
            self._motor1.set_control(VoltageOut(0.0))
        if self._motor2 is not None:
            self._motor2.set_control(VoltageOut(0.0))
        self._current_voltage = 0.0

    def _clamp_voltage(self, volts: float) -> float:
        limit = abs(self.max_voltage_v)
        return max(-limit, min(limit, volts))

    def _check_timeout(self, state_tm: float) -> bool:
        """Return True (and transition to idle) if the test has timed out."""
        if state_tm > self.max_test_duration_s:
            self.next_state(self.idle)
            return True
        return False

    def _update_telemetry(self) -> None:
        self._current_position_rot = self._get_encoder_position()
        self._current_velocity_rps = self._get_encoder_velocity()

    def _save_limits_to_file(self) -> None:
        """Persist upper and lower limits to JSON on the RoboRIO."""
        if is_sim():
            return
        try:
            _LIMITS_FILE.write_text(
                json.dumps({"upper": self.upper_limit_rot, "lower": self.lower_limit_rot})
            )
        except OSError:
            pass

    def _load_limits_from_file(self) -> None:
        """Load persisted limits from JSON; silently defaults to 0.0 if missing."""
        if is_sim():
            return
        try:
            data = json.loads(_LIMITS_FILE.read_text())
            self.upper_limit_rot = float(data.get("upper", 0.0))
            self.lower_limit_rot = float(data.get("lower", 0.0))
        except (OSError, json.JSONDecodeError, ValueError):
            pass

    def done(self) -> None:
        self._current_voltage = 0.0
        self._zero_motors()
        super().done()

    # =========================================================================
    # STATES
    # =========================================================================

    @state(first=True, must_finish=True)
    def idle(self, initial_call: bool) -> None:
        """Idle — motors in COAST mode. Handles limit saves and re-initialization."""
        if initial_call:
            # Lazy first-time setup: tunables and NT are fully initialized by the
            # time the first state executes, so we load limits and init motors here
            # instead of in setup() (which triggers a MagicBot ClassVar annotation bug).
            if self._first_idle:
                self._first_idle = False
                self._load_limits_from_file()
                self._init_motors()
            SignalLogger.write_string("state", "none")
            self._zero_motors()
            self._elapsed_time = 0.0

        self._update_telemetry()

        if self.reinitialize:
            self.reinitialize = False
            self._init_motors()

        if self.save_upper_limit:
            self.save_upper_limit = False
            self.upper_limit_rot = self._current_position_rot
            self._save_limits_to_file()

        if self.save_lower_limit:
            self.save_lower_limit = False
            self.lower_limit_rot = self._current_position_rot
            self._save_limits_to_file()

    @state(must_finish=True)
    def quasistatic_fwd(self, initial_call: bool, state_tm: float) -> None:
        """Quasistatic forward — slowly ramp voltage positive."""
        if initial_call:
            SignalLogger.write_string("state", "quasistatic-forward")
        if self._check_timeout(state_tm):
            return
        self._apply_voltage(self._clamp_voltage(self.quasi_ramp_rate_vs * state_tm))
        self._update_telemetry()
        self._elapsed_time = state_tm

    @state(must_finish=True)
    def quasistatic_rev(self, initial_call: bool, state_tm: float) -> None:
        """Quasistatic reverse — slowly ramp voltage negative."""
        if initial_call:
            SignalLogger.write_string("state", "quasistatic-reverse")
        if self._check_timeout(state_tm):
            return
        self._apply_voltage(self._clamp_voltage(-self.quasi_ramp_rate_vs * state_tm))
        self._update_telemetry()
        self._elapsed_time = state_tm

    @state(must_finish=True)
    def dynamic_fwd(self, initial_call: bool, state_tm: float) -> None:
        """Dynamic forward — apply constant step voltage positive."""
        if initial_call:
            SignalLogger.write_string("state", "dynamic-forward")
        if self._check_timeout(state_tm):
            return
        self._apply_voltage(self._clamp_voltage(self.dynamic_step_voltage_v))
        self._update_telemetry()
        self._elapsed_time = state_tm

    @state(must_finish=True)
    def dynamic_rev(self, initial_call: bool, state_tm: float) -> None:
        """Dynamic reverse — apply constant step voltage negative."""
        if initial_call:
            SignalLogger.write_string("state", "dynamic-reverse")
        if self._check_timeout(state_tm):
            return
        self._apply_voltage(self._clamp_voltage(-self.dynamic_step_voltage_v))
        self._update_telemetry()
        self._elapsed_time = state_tm
