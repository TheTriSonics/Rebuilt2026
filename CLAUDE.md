# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

FRC Team 4003 (TriSonics) Rebuilt 2026 robot code. RobotPy (Python) with MagicBot framework, swerve drivetrain, PhotonVision AprilTag localization, and Phoenix 6 motor control.

## Commands

- **Deploy to roboRIO:** `robotpy deploy`
- **Simulate:** `robotpy sim`
- **Lint:** `ruff check .` (fix with `--fix`)
- **Format:** `ruff format .`
- **Syntax check a file:** `python3 -m py_compile <file>`

Package management uses `uv` (see `uv.lock`). Python 3.13.

## Architecture

### MagicBot Component System

`robot.py` declares components and controllers as class-level type annotations. MagicBot auto-creates them, injects cross-component dependencies, then calls each component's `execute()` **in declaration order** every 20ms loop. This order matters — vision must run before drivetrain so pose estimates feed into odometry.

**Execution order:** kicker → intake → gyro → **vision** → **drivetrain** → shot_calc → shooter → battery_monitor

**Controllers** (`controllers/`) are `StateMachine` subclasses that coordinate multi-component sequences (e.g., drive mode switching, shooting). They are declared before components in `robot.py`.

**Components** (`components/`) encapsulate hardware subsystems. Each has an `execute()` method called automatically by MagicBot. Use `magicbot.tunable()` for runtime-adjustable parameters.

### Vision Subprocess Architecture

Vision processing runs in a **separate OS process** (`vision_worker.py`) to avoid GIL contention on the roboRIO's limited CPU:

- `robot.py` launches `vision_worker.py` via `subprocess.Popen` (real robot only, not sim)
- Worker connects to the robot's NetworkTables server as a client on `127.0.0.1`
- Worker reads 3 PhotonVision cameras, computes multi-tag pose estimates, fuses via inverse-variance weighting, publishes fused result to `/vision_worker/fused_estimate`
- Main process (`VisionComponent._execute_worker()`) reads the fused pose from NT and calls `addVisionMeasurement()` once per cycle
- In simulation, `VisionComponent._execute_inline()` processes cameras directly because `physics.py` injects simulated data through the PhotonCamera objects

**Critical constraint:** `vision_worker.py` must NEVER import `wpilib` or `hal` — doing so on the roboRIO will kill the main robot process.

### Autonomous Modes

Autonomous routines (`autonomous/`) extend `AutonBase` (which extends `AutonomousStateMachine`). They use Choreo trajectories with event markers and support alliance mirroring via `choreo_utils.mirrored()`.

### Simulation

`physics.py` provides a full simulation including swerve module physics (`SimpleTalonFXMotorSim`, `Falcon500MotorSim`), gyroscope simulation, and PhotonVision camera simulation with AprilTag field layout.

## Key Conventions

- CAN device IDs are centralized in `ids.py` using `TalonId` and `CancoderId` enums
- Swerve constants come from `generated/tuner_constants_swerve.py` (auto-generated, do not hand-edit)
- NetworkTables debug topics follow `/components/{name}/{metric}` pattern
- Camera offsets use `Transform3d` with inches converted via `units.inchesToMeters()`

## Ruff Config

Line length 100, target Python 3.13. Rules: E, W, F, I, B, UP. E501 and I001 are ignored.
