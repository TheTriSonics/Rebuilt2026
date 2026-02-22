# Rebuilt 2026

FRC Team 4003 — The TriSonics — 2026 robot code for the FIRST Robotics Competition *Rebuilt* season.

Built with [RobotPy](https://robotpy.readthedocs.io/) and the [MagicBot](https://robotpy.readthedocs.io/projects/utilities/en/latest/magicbot.html) framework. Uses a CTRE swerve drivetrain, PhotonVision for AprilTag-based pose estimation, and Choreo for autonomous trajectory following.

## Project Structure

```
robot.py              Main robot class — ties everything together
ids.py                CAN IDs and hardware port constants
physics.py            Simulation physics model
components/           Low-level hardware abstractions
  drivetrain.py       Swerve drive (CTRE Phoenix 6)
  gyro.py             IMU / gyroscope
  vision.py           PhotonVision AprilTag pose estimation (3 cameras)
  turret.py           Rotating turret
  shooter.py          Dual-flywheel shooter
  kicker.py           Kicker wheel (feeds balls into shooter)
  singulator.py       Singulator (feeds one ball at a time)
  intake.py           Ball intake
  climber.py          End-game climber
  leds.py             LED status indicators
  battery_monitor.py  Battery voltage monitoring
controllers/          MagicBot state machines that coordinate components
  tanker.py           Drive mode manager (field/robot centric, pose targeting, path following)
  gaspump.py          Staggered shooting sequence (shooter → kicker → singulator)
hid/                  Human Interface Devices
  xbox_driver.py      Driver controller mapping (port 0)
  xbox_operator.py    Operator controller mapping (port 1)
autonomous/           Autonomous routines
deploy/               Choreo trajectories and PathPlanner configs
generated/            CTRE swerve tuner constants
utilities/            Helper functions (joystick scaling, game state queries)
```

## Controls

### Driver Controller (Xbox — Port 0)

| Input | Action |
|---|---|
| Left Stick | Translate (X/Y movement) |
| Right Stick X | Rotate |
| Right Bumper | Shoot |
| Right Trigger | Intake on |
| Left Trigger | Intake reverse |
| X Button | Drive to target AprilTag pose |
| Start (button 8) | Switch to field-centric driving |
| Back (button 7) | Switch to robot-centric driving |
| D-Pad Down | Reset yaw |
| D-Pad Left | Turret nudge left |
| D-Pad Right | Turret nudge right |

### Operator Controller (Xbox — Port 1)

| Input | Action |
|---|---|
| Right Stick X | Turret manual control |
| Left Stick Y | Hood adjustment |
| Right Bumper | Shoot |
| Right Trigger | Hang up |
| Left Trigger | Hang down |
| A Button | Singulator forward |
| B Button | Singulator reverse |
| X Button | Intake up |
| Y Button | Intake down |

## Getting Started

### Prerequisites

- Python 3.13+
- [uv](https://docs.astral.sh/uv/) (Python package manager)

### Clone and Run

```bash
git clone git@github.com:TheTriSonics/Rebuilt2026.git
cd Rebuilt2026
```

Install dependencies:

```bash
uv sync
```

Run in simulation:

```bash
uv run robotpy sim
```

Deploy to the robot:

```bash
uv run robotpy deploy
```

Run tests:

```bash
uv run robotpy test
```

### Linting

```bash
uv run ruff check .
uv run ruff format .
```
