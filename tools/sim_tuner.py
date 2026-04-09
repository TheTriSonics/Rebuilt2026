#!/usr/bin/env python3
"""
Simulation Physics Tuner

Replays joystick inputs from real-match wpilog files through a headless
physics model and compares the resulting trajectory to the real robot's
FusedPose.  Optimizes the physics parameters in physics.py so the
simulator better matches reality.

Usage:
    uv run python tools/sim_tuner.py <logfile.wpilog>
    uv run python tools/sim_tuner.py <logfile.wpilog> --optimize
    uv run python tools/sim_tuner.py <logfile.wpilog> --plot
    uv run python tools/sim_tuner.py <logfile.wpilog> --optimize --plot

The tool focuses on teleop periods where joystick inputs drive the robot
(not autonomous path following).
"""

from __future__ import annotations

import argparse
import math
import struct
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from wpiutil.log import DataLogReader


# ── Robot constants (mirrored from the codebase) ─────────────────────────

# From generated/tuner_constants_swerve.py
WHEEL_RADIUS = 0.0508  # 2 inches in meters
DRIVE_GEAR_RATIO = 5.54
STEER_GEAR_RATIO = 25.0
WHEEL_CIRCUMFERENCE = WHEEL_RADIUS * math.tau
DRIVE_MOTOR_REV_TO_METERS = WHEEL_CIRCUMFERENCE / DRIVE_GEAR_RATIO

# Module positions (inches → meters)
FL_X, FL_Y = 0.0254 * 11, 0.0254 * 11.625
FR_X, FR_Y = 0.0254 * 11, -0.0254 * 11.625
BL_X, BL_Y = -0.0254 * 11, 0.0254 * 11.625
BR_X, BR_Y = -0.0254 * 11, -0.0254 * 11.625

# From robot.py
DEFAULT_MAX_SPEED = 8.0  # m/s
DEFAULT_MAX_ROTATION = 4 * math.tau  # rad/s

# From drivetrain.py
DEFAULT_MAX_LINEAR_ACCEL = 16.0  # m/s²
DEFAULT_MAX_LINEAR_DECEL = 72.0  # m/s²
DEFAULT_MAX_ANGULAR_ACCEL = 40.0  # rad/s²
SLOW_MODE_DIVISOR = 4.0


# ── Joystick processing (mirrored from utilities/scalers.py) ─────────────


def apply_deadzone(value: float, threshold: float) -> float:
    if abs(value) < threshold:
        return 0.0
    return (value - math.copysign(threshold, value)) / (1 - threshold)


def map_exponential(value: float, base: float) -> float:
    return math.copysign((base ** abs(value) - 1) / (base - 1), value)


def rescale_js(value: float, deadzone: float, exponential: float = 1.5) -> float:
    return map_exponential(apply_deadzone(value, deadzone), exponential + 1)


# ── Data structures ──────────────────────────────────────────────────────


@dataclass
class LogData:
    """Extracted time-series from a wpilog file."""

    # Joystick samples (times in seconds relative to enable)
    joy_t: np.ndarray = field(default_factory=lambda: np.array([]))
    joy_left_x: np.ndarray = field(default_factory=lambda: np.array([]))
    joy_left_y: np.ndarray = field(default_factory=lambda: np.array([]))
    joy_right_x: np.ndarray = field(default_factory=lambda: np.array([]))
    joy_right_trigger: np.ndarray = field(default_factory=lambda: np.array([]))

    # Fused pose (ground truth)
    pose_t: np.ndarray = field(default_factory=lambda: np.array([]))
    pose_x: np.ndarray = field(default_factory=lambda: np.array([]))
    pose_y: np.ndarray = field(default_factory=lambda: np.array([]))
    pose_heading: np.ndarray = field(default_factory=lambda: np.array([]))  # radians

    # Gyro heading
    heading_t: np.ndarray = field(default_factory=lambda: np.array([]))
    heading_deg: np.ndarray = field(default_factory=lambda: np.array([]))

    # Z-acceleration (for tilt detection)
    z_accel_t: np.ndarray = field(default_factory=lambda: np.array([]))
    z_accel: np.ndarray = field(default_factory=lambda: np.array([]))

    # Tanker mode changes
    tanker_t: list[float] = field(default_factory=list)
    tanker_mode: list[str] = field(default_factory=list)

    # Alliance info
    is_red: bool = False

    # Teleop bounds (relative to enable)
    teleop_start: float = 0.0
    teleop_end: float = 0.0


@dataclass
class PhysicsParams:
    """Tunable physics parameters.

    These map back to fudge-factors in physics.py:
      omega_gain  → `speeds.omega *= <value>`
      linear_gain → related to kV in SimpleTalonFXMotorSim
    """

    # Steady-state speed response (1.0 = perfect tracking)
    linear_gain: float = 1.0
    # Rotation speed scaling (physics.py: speeds.omega *= 0.35)
    omega_gain: float = 1.0


@dataclass
class SimState:
    """Current state of the simulated robot."""

    x: float = 0.0
    y: float = 0.0
    heading: float = 0.0  # radians
    # Previously commanded speeds (for rate limiting)
    cmd_vx: float = 0.0
    cmd_vy: float = 0.0
    cmd_omega: float = 0.0


# ── Log reader ───────────────────────────────────────────────────────────


def read_log(path: str | Path) -> LogData:
    """Parse a wpilog file and extract driving-relevant time series."""
    reader = DataLogReader(str(path))

    # First pass: build entry ID → name/type map
    entries: dict[int, dict] = {}
    for record in reader:
        if record.isStart():
            data = record.getStartData()
            entries[data.entry] = {"name": data.name, "type": data.type}

    # Second pass: extract data
    reader = DataLogReader(str(path))

    enable_time: float | None = None
    auton_end_time: float | None = None
    disable_time: float | None = None
    is_red = False

    # Raw collection lists
    joy_raw: list[tuple[float, list[float]]] = []
    pose_raw: list[tuple[float, float, float, float]] = []
    heading_raw: list[tuple[float, float]] = []
    z_accel_raw: list[tuple[float, float]] = []
    tanker_raw: list[tuple[float, str]] = []
    alliance_set = False

    for record in reader:
        if record.isControl():
            continue

        eid = record.getEntry()
        info = entries.get(eid)
        if not info:
            continue
        name = info["name"]
        ts = record.getTimestamp() / 1e6  # microseconds → seconds

        # Track enable/disable/autonomous transitions
        if name == "DS:enabled":
            val = record.getBoolean()
            if val and enable_time is None:
                enable_time = ts
            elif not val and enable_time is not None and disable_time is None:
                # First disable after enable that's past autonomous
                if auton_end_time is not None:
                    disable_time = ts

        if name == "DS:autonomous":
            val = record.getBoolean()
            # Transition from autonomous=True to autonomous=False = teleop start
            if not val and enable_time is not None and auton_end_time is None:
                auton_end_time = ts

        if name == "NT:/FMSInfo/IsRedAlliance" and not alliance_set:
            is_red = record.getBoolean()
            alliance_set = True

        # Skip records before enable
        if enable_time is None:
            continue
        t = ts - enable_time

        if name == "DS:joystick0/axes":
            axes = record.getFloatArray()
            joy_raw.append((t, list(axes)))

        elif name == "NT:FusedPose":
            raw = record.getRaw()
            if len(raw) == 24:
                x, y, theta = struct.unpack("<ddd", raw)
                pose_raw.append((t, x, y, theta))

        elif name == "NT:/components/gyro/heading":
            heading_raw.append((t, record.getDouble()))

        elif name == "NT:/components/gyro/z_accel":
            z_accel_raw.append((t, record.getDouble()))

        elif name == "NT:/components/tanker/state/current_state":
            tanker_raw.append((t, record.getString()))

    if enable_time is None:
        print("ERROR: Robot was never enabled in this log file")
        sys.exit(1)

    # Determine teleop bounds
    teleop_start = (auton_end_time - enable_time) if auton_end_time else 0.0
    if disable_time:
        teleop_end = disable_time - enable_time
    elif pose_raw:
        teleop_end = pose_raw[-1][0]
    else:
        teleop_end = joy_raw[-1][0] if joy_raw else 0.0

    # Filter to teleop-only joystick data
    teleop_joy = [(t, a) for t, a in joy_raw if teleop_start <= t <= teleop_end]
    teleop_pose = [(t, x, y, h) for t, x, y, h in pose_raw if teleop_start <= t <= teleop_end]
    teleop_heading = [(t, h) for t, h in heading_raw if teleop_start <= t <= teleop_end]
    teleop_tanker = [(t, m) for t, m in tanker_raw if t <= teleop_end]

    log = LogData(is_red=is_red, teleop_start=teleop_start, teleop_end=teleop_end)

    if teleop_joy:
        log.joy_t = np.array([t for t, _ in teleop_joy])
        # XboxController axis mapping: 0=leftX, 1=leftY, 2=leftTrig, 3=rightTrig, 4=rightX, 5=rightY
        log.joy_left_x = np.array([a[0] if len(a) > 0 else 0.0 for _, a in teleop_joy])
        log.joy_left_y = np.array([a[1] if len(a) > 1 else 0.0 for _, a in teleop_joy])
        log.joy_right_x = np.array([a[4] if len(a) > 4 else 0.0 for _, a in teleop_joy])
        log.joy_right_trigger = np.array([a[3] if len(a) > 3 else 0.0 for _, a in teleop_joy])

    if teleop_pose:
        log.pose_t = np.array([t for t, _, _, _ in teleop_pose])
        log.pose_x = np.array([x for _, x, _, _ in teleop_pose])
        log.pose_y = np.array([y for _, _, y, _ in teleop_pose])
        log.pose_heading = np.array([h for _, _, _, h in teleop_pose])

    if teleop_heading:
        log.heading_t = np.array([t for t, _ in teleop_heading])
        log.heading_deg = np.array([h for _, h in teleop_heading])

    teleop_z_accel = [(t, z) for t, z in z_accel_raw if teleop_start <= t <= teleop_end]
    if teleop_z_accel:
        log.z_accel_t = np.array([t for t, _ in teleop_z_accel])
        log.z_accel = np.array([z for _, z in teleop_z_accel])

    log.tanker_t = [t for t, _ in teleop_tanker]
    log.tanker_mode = [m for _, m in teleop_tanker]

    return log


# ── Interpolation helper ─────────────────────────────────────────────────


def interp_at(times: np.ndarray, values: np.ndarray, t: float) -> float:
    """Linearly interpolate a time-series at time t (sample-and-hold at edges)."""
    if len(times) == 0:
        return 0.0
    if t <= times[0]:
        return float(values[0])
    if t >= times[-1]:
        return float(values[-1])
    idx = np.searchsorted(times, t, side="right") - 1
    if idx >= len(times) - 1:
        return float(values[-1])
    dt = times[idx + 1] - times[idx]
    if dt == 0:
        return float(values[idx])
    frac = (t - times[idx]) / dt
    return float(values[idx] + frac * (values[idx + 1] - values[idx]))


def get_tanker_mode(log: LogData, t: float) -> str:
    """Get the tanker mode at time t."""
    mode = ""
    for mt, mm in zip(log.tanker_t, log.tanker_mode, strict=False):
        if mt <= t:
            mode = mm
        else:
            break
    return mode


# ── Tilt / pose-confidence detection ─────────────────────────────────────

# Nominal gravity along the IMU Z axis (~1g in whatever units the Pigeon2
# reports — often raw sensor counts or m/s², check your log).  When the
# robot tilts the rolling-average drifts away from this value.
_GRAVITY_NOMINAL = 1.0  # Will be auto-calibrated from the first few seconds

# Rolling-window size (seconds) for z-accel averaging.
_TILT_WINDOW_S = 0.5

# How far the rolling average can deviate from nominal before we consider
# the pose untrustworthy (fraction of nominal, e.g. 0.15 = 15%).
_TILT_THRESHOLD = 0.15


def compute_pose_confidence(log: LogData) -> np.ndarray:
    """Per-pose-sample confidence weight in [0, 1].

    Uses the rolling mean of Z-acceleration to detect tilt / impacts.
    When the robot is flat on the carpet the Z-accel averages out to
    ~1 g over any half-second window.  When tilted or slamming into
    walls the average drops (gravity component shifts to X/Y axes),
    so we reduce confidence.

    Also flags poses that are clearly off-field (field is ~16.5 x 8.2 m
    with some margin).

    Returns an array the same length as log.pose_t.
    """
    n_pose = len(log.pose_t)
    if n_pose == 0:
        return np.array([])

    confidence = np.ones(n_pose)

    # ── Z-accel tilt weighting ──
    if len(log.z_accel_t) > 10:
        # Auto-calibrate: use median of first 2 seconds as "nominal"
        early_mask = log.z_accel_t < log.z_accel_t[0] + 2.0
        if np.any(early_mask):
            nominal = float(np.median(np.abs(log.z_accel[early_mask])))
        else:
            nominal = float(np.median(np.abs(log.z_accel)))
        if nominal < 0.01:
            nominal = _GRAVITY_NOMINAL  # fallback

        # Compute rolling mean of |z_accel| over _TILT_WINDOW_S
        half_window = _TILT_WINDOW_S / 2.0
        for i, t in enumerate(log.pose_t):
            mask = (log.z_accel_t >= t - half_window) & (log.z_accel_t <= t + half_window)
            if np.sum(mask) < 3:
                continue
            window_mean = float(np.mean(np.abs(log.z_accel[mask])))
            deviation = abs(window_mean - nominal) / nominal
            if deviation > _TILT_THRESHOLD:
                # Scale from 1 → 0 as deviation goes from threshold → 2×threshold
                confidence[i] = max(0.0, 1.0 - (deviation - _TILT_THRESHOLD) / _TILT_THRESHOLD)

    # ── Off-field detection ──
    # 2026 Rebuilt field is approximately 16.54 m × 8.21 m.
    # Give 0.5 m margin before flagging.
    FIELD_X_MAX = 17.0
    FIELD_Y_MAX = 8.7
    FIELD_MIN = -0.5
    for i in range(n_pose):
        x, y = log.pose_x[i], log.pose_y[i]
        if x < FIELD_MIN or x > FIELD_X_MAX or y < FIELD_MIN or y > FIELD_Y_MAX:
            confidence[i] = 0.0

    return confidence


# ── Rate limiter (mirrors drivetrain._rate_limit_speeds) ─────────────────


def rate_limit(
    desired_vx: float,
    desired_vy: float,
    desired_omega: float,
    prev_vx: float,
    prev_vy: float,
    prev_omega: float,
    dt: float,
    max_accel: float = DEFAULT_MAX_LINEAR_ACCEL,
    max_decel: float = DEFAULT_MAX_LINEAR_DECEL,
    max_ang_accel: float = DEFAULT_MAX_ANGULAR_ACCEL,
) -> tuple[float, float, float]:
    """Apply acceleration limits matching drivetrain._rate_limit_speeds."""
    if dt <= 0:
        return desired_vx, desired_vy, desired_omega

    dvx = desired_vx - prev_vx
    dvy = desired_vy - prev_vy
    linear_delta = math.sqrt(dvx * dvx + dvy * dvy)

    if linear_delta > 0:
        prev_speed = math.sqrt(prev_vx**2 + prev_vy**2)
        desired_speed = math.sqrt(desired_vx**2 + desired_vy**2)
        limit = max_decel if desired_speed < prev_speed else max_accel
        max_delta = limit * dt
        if linear_delta > max_delta:
            scale = max_delta / linear_delta
            dvx *= scale
            dvy *= scale

    domega = desired_omega - prev_omega
    max_omega_delta = max_ang_accel * dt
    if abs(domega) > max_omega_delta:
        domega = math.copysign(max_omega_delta, domega)

    return prev_vx + dvx, prev_vy + dvy, prev_omega + domega


# ── Simulation replay ────────────────────────────────────────────────────


def simulate(
    log: LogData,
    params: PhysicsParams,
    dt: float = 0.02,
    use_logged_heading: bool = True,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Replay joystick inputs through the physics model.

    The simulation can run in two modes:
    - use_logged_heading=True (default): Uses the real robot's FusedPose
      heading for both field-relative conversion AND the robot→field
      pose integration transform. This isolates linear speed tuning from
      heading errors. The sim heading output is the *integrated* sim heading
      for comparison purposes only.
    - use_logged_heading=False: Fully headless — the sim integrates its
      own heading and uses it everywhere. Errors compound but gives the
      full picture.

    Returns:
        (times, sim_x, sim_y, sim_heading) arrays.
    """
    if len(log.joy_t) == 0 or len(log.pose_t) == 0:
        return np.array([]), np.array([]), np.array([]), np.array([])

    t_start = log.joy_t[0]
    t_end = log.joy_t[-1]

    # Initialize sim state to match real robot at teleop start
    state = SimState()
    state.x = interp_at(log.pose_t, log.pose_x, t_start)
    state.y = interp_at(log.pose_t, log.pose_y, t_start)
    state.heading = interp_at(log.pose_t, log.pose_heading, t_start)

    max_speed = DEFAULT_MAX_SPEED
    max_rotation = DEFAULT_MAX_ROTATION

    times = []
    xs = []
    ys = []
    headings = []

    t = t_start
    while t <= t_end:
        # Record state
        times.append(t)
        xs.append(state.x)
        ys.append(state.y)
        headings.append(state.heading)

        # Heading to use for field-relative conversion and pose integration
        if use_logged_heading:
            effective_heading = interp_at(log.pose_t, log.pose_heading, t)
        else:
            effective_heading = state.heading

        # Read joystick at this time
        left_x = interp_at(log.joy_t, log.joy_left_x, t)
        left_y = interp_at(log.joy_t, log.joy_left_y, t)
        right_x = interp_at(log.joy_t, log.joy_right_x, t)
        right_trigger = interp_at(log.joy_t, log.joy_right_trigger, t)
        slow_mode = right_trigger > 0.5

        # Apply joystick processing (mirrors robot.py teleopPeriodic)
        cmd_x = -rescale_js(left_y, 0.05, 1.0) * max_speed
        cmd_y = -rescale_js(left_x, 0.05, 1.0) * max_speed
        cmd_omega = -rescale_js(right_x, 0.10, 2.0) * max_rotation

        # Slow mode
        if slow_mode:
            cmd_x /= SLOW_MODE_DIVISOR
            cmd_y /= SLOW_MODE_DIVISOR
            cmd_omega /= SLOW_MODE_DIVISOR

        # Determine drive mode
        mode = get_tanker_mode(log, t)

        # Convert joystick commands to robot-frame chassis speeds
        if mode in ("drive_field", "drive_auto_target"):
            # Alliance flip
            fx = -cmd_x if log.is_red else cmd_x
            fy = -cmd_y if log.is_red else cmd_y
            # ChassisSpeeds.fromFieldRelativeSpeeds
            cos_h = math.cos(effective_heading)
            sin_h = math.sin(effective_heading)
            vx = fx * cos_h + fy * sin_h
            vy = -fx * sin_h + fy * cos_h
            omega = cmd_omega
        elif mode == "drive_local":
            vx = cmd_x
            vy = cmd_y
            omega = cmd_omega
        else:
            # Path following / unknown modes: zero command
            vx = 0.0
            vy = 0.0
            omega = 0.0

        # Rate limiting (mirrors drivetrain._rate_limit_speeds)
        vx, vy, omega = rate_limit(
            vx,
            vy,
            omega,
            state.cmd_vx,
            state.cmd_vy,
            state.cmd_omega,
            dt,
        )
        state.cmd_vx = vx
        state.cmd_vy = vy
        state.cmd_omega = omega

        # Apply physics response: gain scaling
        actual_vx = vx * params.linear_gain
        actual_vy = vy * params.linear_gain
        actual_omega = omega * params.omega_gain

        # Integrate pose (robot-frame velocities → field-frame)
        cos_h = math.cos(effective_heading)
        sin_h = math.sin(effective_heading)
        state.x += (actual_vx * cos_h - actual_vy * sin_h) * dt
        state.y += (actual_vx * sin_h + actual_vy * cos_h) * dt

        # Integrate heading (always from sim omega, for heading comparison)
        state.heading += actual_omega * dt
        state.heading = math.atan2(math.sin(state.heading), math.cos(state.heading))

        t += dt

    return np.array(times), np.array(xs), np.array(ys), np.array(headings)


def simulate_windowed(
    log: LogData,
    params: PhysicsParams,
    window_s: float = 3.0,
    dt: float = 0.02,
    use_logged_heading: bool = True,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Run simulation in windows, resetting pose to FusedPose each window.

    This prevents small per-step errors from accumulating over the full
    match.  Each window starts from the real robot pose and measures how
    far the sim drifts over window_s seconds.  The output arrays are the
    same shape as the full simulation and can be compared to FusedPose
    directly — the per-window drift is the meaningful signal.
    """
    if len(log.joy_t) == 0 or len(log.pose_t) == 0:
        return np.array([]), np.array([]), np.array([]), np.array([])

    t_start = log.joy_t[0]
    t_end = log.joy_t[-1]
    max_speed = DEFAULT_MAX_SPEED
    max_rotation = DEFAULT_MAX_ROTATION

    times: list[float] = []
    xs: list[float] = []
    ys: list[float] = []
    headings: list[float] = []

    window_start = t_start
    while window_start < t_end:
        window_end = min(window_start + window_s, t_end)

        # Reset state to real robot pose at window start
        state = SimState()
        state.x = interp_at(log.pose_t, log.pose_x, window_start)
        state.y = interp_at(log.pose_t, log.pose_y, window_start)
        state.heading = interp_at(log.pose_t, log.pose_heading, window_start)

        t = window_start
        while t <= window_end:
            times.append(t)
            xs.append(state.x)
            ys.append(state.y)
            headings.append(state.heading)

            if use_logged_heading:
                effective_heading = interp_at(log.pose_t, log.pose_heading, t)
            else:
                effective_heading = state.heading

            left_x = interp_at(log.joy_t, log.joy_left_x, t)
            left_y = interp_at(log.joy_t, log.joy_left_y, t)
            right_x = interp_at(log.joy_t, log.joy_right_x, t)
            right_trigger = interp_at(log.joy_t, log.joy_right_trigger, t)
            slow_mode = right_trigger > 0.5

            cmd_x = -rescale_js(left_y, 0.05, 1.0) * max_speed
            cmd_y = -rescale_js(left_x, 0.05, 1.0) * max_speed
            cmd_omega = -rescale_js(right_x, 0.10, 2.0) * max_rotation

            if slow_mode:
                cmd_x /= SLOW_MODE_DIVISOR
                cmd_y /= SLOW_MODE_DIVISOR
                cmd_omega /= SLOW_MODE_DIVISOR

            mode = get_tanker_mode(log, t)

            if mode in ("drive_field", "drive_auto_target"):
                fx = -cmd_x if log.is_red else cmd_x
                fy = -cmd_y if log.is_red else cmd_y
                cos_h = math.cos(effective_heading)
                sin_h = math.sin(effective_heading)
                vx = fx * cos_h + fy * sin_h
                vy = -fx * sin_h + fy * cos_h
                omega = cmd_omega
            elif mode == "drive_local":
                vx = cmd_x
                vy = cmd_y
                omega = cmd_omega
            else:
                vx, vy, omega = 0.0, 0.0, 0.0

            vx, vy, omega = rate_limit(
                vx,
                vy,
                omega,
                state.cmd_vx,
                state.cmd_vy,
                state.cmd_omega,
                dt,
            )
            state.cmd_vx = vx
            state.cmd_vy = vy
            state.cmd_omega = omega

            actual_vx = vx * params.linear_gain
            actual_vy = vy * params.linear_gain
            actual_omega = omega * params.omega_gain

            cos_h = math.cos(effective_heading)
            sin_h = math.sin(effective_heading)
            state.x += (actual_vx * cos_h - actual_vy * sin_h) * dt
            state.y += (actual_vx * sin_h + actual_vy * cos_h) * dt
            state.heading += actual_omega * dt
            state.heading = math.atan2(math.sin(state.heading), math.cos(state.heading))

            t += dt

        window_start = window_end

    return np.array(times), np.array(xs), np.array(ys), np.array(headings)


# ── Error metric ─────────────────────────────────────────────────────────


def compute_error(
    log: LogData,
    sim_t: np.ndarray,
    sim_x: np.ndarray,
    sim_y: np.ndarray,
    sim_heading: np.ndarray,
    confidence: np.ndarray | None = None,
) -> dict[str, float]:
    """Compare simulated trajectory to logged FusedPose.

    Uses per-sample confidence weights from tilt detection to down-weight
    unreliable pose data (tilting, off-field, impacts).

    Returns dict with position_rmse, heading_rmse, combined_error, and
    fraction of samples that were high-confidence.
    """
    if len(sim_t) == 0 or len(log.pose_t) == 0:
        return {
            "position_rmse": float("inf"),
            "heading_rmse": float("inf"),
            "combined_error": float("inf"),
            "confidence_frac": 0.0,
        }

    # Interpolate logged pose at sim timesteps
    real_x = np.interp(sim_t, log.pose_t, log.pose_x)
    real_y = np.interp(sim_t, log.pose_t, log.pose_y)
    real_h = np.interp(sim_t, log.pose_t, log.pose_heading)

    # Interpolate confidence weights at sim timesteps
    if confidence is not None and len(confidence) == len(log.pose_t):
        weights = np.interp(sim_t, log.pose_t, confidence)
    else:
        weights = np.ones_like(sim_t)

    # Position error (meters)
    dx = sim_x - real_x
    dy = sim_y - real_y
    pos_error = np.sqrt(dx**2 + dy**2)

    # Heading error (radians), handling wrap-around
    dh = sim_heading - real_h
    dh = np.arctan2(np.sin(dh), np.cos(dh))

    # Weighted RMSE
    w_sum = float(np.sum(weights))
    if w_sum < 1.0:
        w_sum = 1.0  # avoid division by zero
    pos_rmse = float(np.sqrt(np.sum(weights * pos_error**2) / w_sum))
    heading_rmse = float(np.sqrt(np.sum(weights * dh**2) / w_sum))

    combined = pos_rmse + 0.5 * heading_rmse
    confidence_frac = float(np.mean(weights > 0.5))

    return {
        "position_rmse": pos_rmse,
        "heading_rmse": heading_rmse,
        "heading_rmse_deg": math.degrees(heading_rmse),
        "combined_error": combined,
        "max_pos_error": float(np.max(pos_error)),
        "confidence_frac": confidence_frac,
    }


# ── Optimizer ────────────────────────────────────────────────────────────


def optimize_params(
    log: LogData,
    initial: PhysicsParams | None = None,
    use_logged_heading: bool = True,
) -> PhysicsParams:
    """Find physics parameters that minimize trajectory error.

    By default uses logged heading (use_logged_heading=True) so that
    position error from heading drift doesn't pollute the linear gain
    optimization.
    """
    from scipy.optimize import minimize

    if initial is None:
        initial = PhysicsParams()

    confidence = compute_pose_confidence(log)

    def objective(x: np.ndarray) -> float:
        params = PhysicsParams(
            linear_gain=float(x[0]),
            omega_gain=float(x[1]),
        )
        sim_t, sim_x, sim_y, sim_h = simulate_windowed(
            log,
            params,
            use_logged_heading=use_logged_heading,
        )
        error = compute_error(log, sim_t, sim_x, sim_y, sim_h, confidence)
        return error["combined_error"]

    x0 = np.array([initial.linear_gain, initial.omega_gain])

    bounds = [
        (0.1, 2.0),  # linear_gain
        (0.05, 2.0),  # omega_gain
    ]

    print("Optimizing physics parameters...")
    print(f"  Initial: linear_gain={x0[0]:.3f}, omega_gain={x0[1]:.3f}")
    print(f"  use_logged_heading={use_logged_heading}")

    err0 = objective(x0)
    print(f"  Initial combined error: {err0:.4f}")

    result = minimize(
        objective,
        x0,
        method="Nelder-Mead",
        bounds=bounds,
        options={"maxiter": 200, "xatol": 0.001, "fatol": 0.001, "disp": True},
    )

    best = PhysicsParams(
        linear_gain=float(result.x[0]),
        omega_gain=float(result.x[1]),
    )

    print("\nOptimized parameters:")
    print(f"  linear_gain  = {best.linear_gain:.4f}")
    print(
        f"  omega_gain   = {best.omega_gain:.4f}  (→ physics.py: speeds.omega *= {best.omega_gain:.4f})"
    )
    print(f"  Final combined error: {result.fun:.4f}")

    return best


# ── Visualization ────────────────────────────────────────────────────────


def plot_comparison(
    log: LogData,
    sim_t: np.ndarray,
    sim_x: np.ndarray,
    sim_y: np.ndarray,
    sim_heading: np.ndarray,
    confidence: np.ndarray | None = None,
    title: str = "Sim vs Real",
    save_path: str | None = None,
) -> None:
    """Plot simulated vs real trajectory with confidence shading."""
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 3, figsize=(18, 10))
    fig.suptitle(title, fontsize=14)

    # Helper: shade low-confidence regions on a time-series axis
    def shade_low_confidence(ax):
        if confidence is None or len(confidence) == 0:
            return
        low_mask = confidence < 0.5
        if not np.any(low_mask):
            return
        low_times = log.pose_t[low_mask]
        gaps = np.where(np.diff(low_times) > 0.5)[0]
        starts = [low_times[0]] + [low_times[g + 1] for g in gaps]
        ends = [low_times[g] for g in gaps] + [low_times[-1]]
        for s, e in zip(starts, ends, strict=False):
            ax.axvspan(s, e, alpha=0.15, color="orange", label=None)

    # Top-left: XY trajectory (bird's eye)
    ax = axes[0][0]
    ax.plot(log.pose_x, log.pose_y, "b-", linewidth=1.5, label="Real (FusedPose)", alpha=0.8)
    ax.plot(sim_x, sim_y, "r--", linewidth=1.5, label="Simulated", alpha=0.8)
    if len(log.pose_x) > 0:
        ax.plot(log.pose_x[0], log.pose_y[0], "bo", markersize=8)
    if len(sim_x) > 0:
        ax.plot(sim_x[0], sim_y[0], "ro", markersize=8)
    # Shade low-confidence poses on XY plot
    if confidence is not None:
        low_mask = confidence < 0.5
        if np.any(low_mask):
            ax.scatter(
                log.pose_x[low_mask],
                log.pose_y[low_mask],
                c="orange",
                s=3,
                alpha=0.4,
                label="Low confidence",
                zorder=0,
            )
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("Field Trajectory (XY)")
    ax.legend(fontsize=8)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)

    # Top-middle: X vs time
    ax = axes[0][1]
    shade_low_confidence(ax)
    ax.plot(log.pose_t, log.pose_x, "b-", linewidth=1, label="Real X", alpha=0.8)
    ax.plot(sim_t, sim_x, "r--", linewidth=1, label="Sim X", alpha=0.8)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("X (m)")
    ax.set_title("X Position vs Time")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # Top-right: Y vs time
    ax = axes[0][2]
    shade_low_confidence(ax)
    ax.plot(log.pose_t, log.pose_y, "b-", linewidth=1, label="Real Y", alpha=0.8)
    ax.plot(sim_t, sim_y, "r--", linewidth=1, label="Sim Y", alpha=0.8)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Y (m)")
    ax.set_title("Y Position vs Time")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # Bottom-left: Heading vs time
    ax = axes[1][0]
    shade_low_confidence(ax)
    real_heading_deg = np.degrees(np.interp(sim_t, log.pose_t, log.pose_heading))
    ax.plot(sim_t, real_heading_deg, "b-", linewidth=1, label="Real heading", alpha=0.8)
    ax.plot(sim_t, np.degrees(sim_heading), "r--", linewidth=1, label="Sim heading", alpha=0.8)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Heading (deg)")
    ax.set_title("Heading vs Time")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # Bottom-middle: Position error vs time
    ax = axes[1][1]
    shade_low_confidence(ax)
    real_x_interp = np.interp(sim_t, log.pose_t, log.pose_x)
    real_y_interp = np.interp(sim_t, log.pose_t, log.pose_y)
    pos_err = np.sqrt((sim_x - real_x_interp) ** 2 + (sim_y - real_y_interp) ** 2)
    ax.plot(sim_t, pos_err, "k-", linewidth=1)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Position Error (m)")
    ax.set_title("Position Error vs Time")
    ax.grid(True, alpha=0.3)

    # Bottom-right: Z-accel and confidence
    ax = axes[1][2]
    if len(log.z_accel_t) > 0:
        ax.plot(log.z_accel_t, log.z_accel, "gray", linewidth=0.3, alpha=0.5, label="Z-accel (raw)")
        # Rolling mean
        window = max(1, int(0.5 / np.median(np.diff(log.z_accel_t))))
        if window < len(log.z_accel):
            kernel = np.ones(window) / window
            rolling = np.convolve(np.abs(log.z_accel), kernel, mode="same")
            ax.plot(
                log.z_accel_t,
                rolling,
                "g-",
                linewidth=1,
                label=f"|Z-accel| rolling mean ({_TILT_WINDOW_S}s)",
            )
    if confidence is not None:
        ax2 = ax.twinx()
        ax2.plot(log.pose_t, confidence, "orange", linewidth=1, label="Confidence")
        ax2.set_ylabel("Confidence", color="orange")
        ax2.set_ylim(-0.05, 1.1)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Z-accel")
    ax.set_title("Tilt Detection")
    ax.legend(fontsize=8, loc="upper left")
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=150)
        print(f"Plot saved to {save_path}")
    else:
        plt.show()


# ── CLI ──────────────────────────────────────────────────────────────────


def main():
    parser = argparse.ArgumentParser(description="Tune physics.py parameters using real match logs")
    parser.add_argument("logfile", help="Path to .wpilog file")
    parser.add_argument(
        "--optimize", action="store_true", help="Run optimizer to find best physics parameters"
    )
    parser.add_argument("--plot", action="store_true", help="Show trajectory comparison plots")
    parser.add_argument(
        "--save-plot", type=str, default=None, help="Save plot to file instead of showing"
    )
    parser.add_argument(
        "--no-logged-heading",
        action="store_true",
        help="Don't use logged heading — fully headless sim (errors compound but tests everything)",
    )
    parser.add_argument(
        "--linear-gain", type=float, default=1.0, help="Linear speed gain (default: 1.0)"
    )
    parser.add_argument(
        "--omega-gain", type=float, default=1.0, help="Angular speed gain (default: 1.0)"
    )

    args = parser.parse_args()

    # Read log
    log_path = Path(args.logfile)
    if not log_path.exists():
        print(f"ERROR: File not found: {log_path}")
        sys.exit(1)

    print(f"Reading {log_path.name}...")
    log = read_log(log_path)

    print(f"  Alliance: {'Red' if log.is_red else 'Blue'}")
    print(
        f"  Teleop period: {log.teleop_start:.1f}s - {log.teleop_end:.1f}s "
        f"({log.teleop_end - log.teleop_start:.1f}s)"
    )
    print(f"  Joystick samples: {len(log.joy_t)}")
    print(f"  Pose samples: {len(log.pose_t)}")
    print(f"  Z-accel samples: {len(log.z_accel_t)}")
    print(f"  Tanker modes: {set(log.tanker_mode)}")

    if len(log.joy_t) == 0:
        print("ERROR: No joystick data in teleop period")
        sys.exit(1)
    if len(log.pose_t) == 0:
        print("ERROR: No pose data in teleop period")
        sys.exit(1)

    # Compute pose confidence using tilt detection
    confidence = compute_pose_confidence(log)
    high_conf = float(np.mean(confidence > 0.5)) * 100
    low_conf_count = int(np.sum(confidence < 0.5))
    print(
        f"  Pose confidence: {high_conf:.0f}% high-confidence, "
        f"{low_conf_count} low-confidence samples"
    )
    if low_conf_count > 0:
        # Show when low-confidence periods occur
        low_mask = confidence < 0.5
        low_times = log.pose_t[low_mask]
        if len(low_times) > 0:
            # Group into contiguous periods
            gaps = np.where(np.diff(low_times) > 1.0)[0]
            starts = [low_times[0]] + [low_times[g + 1] for g in gaps]
            ends = [low_times[g] for g in gaps] + [low_times[-1]]
            print("  Low-confidence periods:")
            for s, e in zip(starts, ends, strict=False):
                print(f"    t={s:.1f}s - {e:.1f}s ({e - s:.1f}s)")

    use_logged_heading = not args.no_logged_heading

    # Set up parameters
    params = PhysicsParams(
        linear_gain=args.linear_gain,
        omega_gain=args.omega_gain,
    )

    if args.optimize:
        params = optimize_params(log, params, use_logged_heading=use_logged_heading)

    # Run simulation (windowed to prevent cumulative drift)
    print(
        f"\nSimulating with: linear_gain={params.linear_gain:.3f}, "
        f"omega_gain={params.omega_gain:.3f}, "
        f"use_logged_heading={use_logged_heading}, window=3.0s"
    )

    sim_t, sim_x, sim_y, sim_h = simulate_windowed(
        log,
        params,
        use_logged_heading=use_logged_heading,
    )

    # Compute errors (with confidence weighting)
    errors = compute_error(log, sim_t, sim_x, sim_y, sim_h, confidence)
    print("\nError metrics (confidence-weighted):")
    print(f"  Position RMSE:     {errors['position_rmse']:.3f} m")
    print(f"  Max position error:{errors['max_pos_error']:.3f} m")
    print(f"  Heading RMSE:      {errors['heading_rmse_deg']:.1f} deg")
    print(f"  Combined error:    {errors['combined_error']:.3f}")
    print(f"  High-confidence:   {errors['confidence_frac'] * 100:.0f}% of samples")

    # Map back to physics.py values
    print("\n── Suggested physics.py changes ──")
    print(f"  speeds.omega *= {params.omega_gain:.4f}  (currently 0.35)")
    if abs(params.linear_gain - 1.0) > 0.05:
        # kV is inversely related to speed: higher kV = slower motor
        suggested_kv = 2.7 / params.linear_gain
        print(f"  kV = {suggested_kv:.2f}  (currently 2.7, linear_gain={params.linear_gain:.3f})")
    else:
        print(f"  kV = 2.7  (linear_gain={params.linear_gain:.3f}, no change needed)")

    if args.plot or args.save_plot:
        title = f"Sim Tuner: {log_path.stem}"
        plot_comparison(
            log,
            sim_t,
            sim_x,
            sim_y,
            sim_h,
            confidence=confidence,
            title=title,
            save_path=args.save_plot,
        )


if __name__ == "__main__":
    main()
