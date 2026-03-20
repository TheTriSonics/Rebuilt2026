# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

FRC Team 4003 (TriSonics) robot code for the 2026 Rebuilt season. Python-based using RobotPy and MagicBot framework.

## Common Commands

```bash
# Install dependencies (uses uv package manager)
uv sync

# Deploy to robot
python -m robotpy deploy

# Run simulation
python -m robotpy sim

# Lint
uv run ruff check .

# Format
uv run ruff format .

# Sync robotpy dependencies to robot
python -m robotpy sync
```

## Architecture

### Framework: MagicBot

This robot uses **MagicBot**, not WPILib's command-based framework. Key differences:

- **Components** are declared as type-annotated class attributes on `MyRobot` (in `robot.py`). MagicBot auto-creates and injects them.
- **Declaration order matters** — components' `execute()` methods run in the order they're declared on the robot class.
- **Controllers** (`controllers/`) are `StateMachine` subclasses that orchestrate multi-step behaviors across components. They must be declared before components.
- Components expose setter methods (called during teleop/auton) and an `execute()` method (called automatically every loop by MagicBot). Setters set intent; `execute()` acts on it.
- `@tunable` creates dashboard-editable parameters. `@feedback` publishes telemetry.

### Component Execution Order (as declared in robot.py)

1. **KickerComponent** — ball kicker motor
2. **IntakeComponent** — intake rotation + roller
3. **GyroComponent** — Pigeon2 IMU heading
4. **VisionComponent** — 3-camera AprilTag pose estimation (PhotonVision)
5. **DrivetrainComponent** — swerve drive kinematics, odometry, pose estimation
6. **ShotCalculatorComponent** — target detection and trajectory math
7. **ShooterComponent** — flywheel + hood control
8. **BatteryMonitorComponent** — voltage monitoring

### Controllers (StateMachines)

- **Tanker** (`controllers/tanker.py`) — drive mode FSM: field-relative, robot-relative, auto-targeting, trajectory following
- **GasPump** (`controllers/gaspump.py`) — shooting sequence FSM with staggered spin-up to avoid voltage sag

### Autonomous

- Lives in `autonomous/` using `AutonomousStateMachine`
- `autonomous/base.py` provides `AutonBase` with pose-setting and Choreo event parsing
- Trajectory files in `deploy/choreo/` (`.traj` format)
- Alliance/side mirroring via `utilities/choreo_utils.py`

### Vision Pipeline

- 3 PhotonVision cameras (Rear_Right, Rear_Left, Rear) doing AprilTag detection
- Multi-tag pose estimation with inverse-variance fusion in `VisionComponent`
- Feeds into `DrivetrainComponent`'s `SwerveDrive4PoseEstimator` Kalman filter
- Rejection criteria: ambiguity ≥0.2, Z>0.2m, distance>2.0m, pose divergence>1.0m

### Swerve Drivetrain

- 4 `SwerveModule` instances with TalonFX motors + CANcoders
- Auto-generated motor gains and kinematics in `generated/tuner_constants_swerve.py` (from CTRE Tuner X — do not hand-edit)
- Two CAN buses: drive bus (swerve hardware + Pigeon2) and shooter bus (intake, shooter, kicker, climber)

### Hardware IDs

All CAN device IDs are centralized in `ids.py`.

### HID Controllers

- `hid/xbox_driver.py` — driver joystick (port 0)
- `hid/xbox_operator.py` — operator joystick (port 1)

## Key Conventions

- Python 3.13, linted with ruff (line length 100)
- Vendor libraries: phoenix6 (CTRE motors), photonlibpy (PhotonVision), choreolib (trajectories)
- Simulation support via `physics.py` and `is_sim()` checks
- Team number: 4003
