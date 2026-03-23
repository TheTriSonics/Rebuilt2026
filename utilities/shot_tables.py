"""Shared lookup tables for the shooter and shot calculator.

All tables live here so both components can import them without circular
dependencies.

ALL VALUES ARE DUMMIES -- replace with real data measured on the robot.
"""

from utilities.interpolation import InterpolatingTable, BilinearInterpolatingTable

# ── Distance (meters) -> hood RPS ────────────────────────────────────────
# Collect data: shoot from measured distances, record the hood speed that
# makes the ball go in consistently.
HOOD_RPS_TABLE = InterpolatingTable(
    {
        1.0: 8.0,
        1.5: 9.0,
        2.0: 10.0,
        2.5: 11.0,
        3.0: 14.0,
        3.5: 17.0,
        4.0: 20.0,
        4.5: 24.0,
        5.0: 28.0,
        6.0: 33.0,
        7.0: 38.0,
        8.0: 42.0,
    }
)

# ── Distance (meters) -> flywheel RPS ────────────────────────────────────
# Currently constant 40 RPS at all distances.  The table is here so you can
# vary it per-distance later if needed (e.g. run slower up close to reduce
# ball damage / current draw, or faster at extreme range).
FLYWHEEL_RPS_TABLE = InterpolatingTable(
    {
        1.0: 40.0,
        2.0: 40.0,
        3.0: 40.0,
        4.0: 40.0,
        5.0: 40.0,
        6.0: 40.0,
        7.0: 40.0,
        8.0: 40.0,
    }
)

# ── (Hood RPS, Flywheel RPS) -> flight time (seconds) ───────────────────
# Flight time is physically determined by launch parameters, not distance.
# Hood RPS controls launch angle (arc height) -- dominant factor.
# Flywheel RPS controls exit speed -- higher speed = shorter flight time
# at the same angle.
#
# This is a 2D bilinear interpolation table:
#   rows = hood RPS breakpoints
#   cols = flywheel RPS breakpoints
#   values[row][col] = flight time in seconds
#
# DUMMY VALUES -- measure on the robot using high-speed video or sensor
# timestamps at various hood/flywheel combinations.
#
# Reading the table:
#   - Moving DOWN (higher hood RPS) = steeper arc = longer flight time
#   - Moving RIGHT (higher flywheel RPS) = faster exit = shorter flight time
_FLIGHT_TIME_HOOD_KEYS = [8.0, 10.0, 14.0, 20.0, 28.0, 33.0, 42.0]
_FLIGHT_TIME_FLYWHEEL_KEYS = [30.0, 35.0, 40.0, 45.0, 50.0]
_FLIGHT_TIME_VALUES = [
    # flywheel:  30     35     40     45     50
    [0.25, 0.22, 0.20, 0.18, 0.16],  # hood  8 RPS (flat, close)
    [0.32, 0.28, 0.25, 0.23, 0.21],  # hood 10 RPS
    [0.44, 0.39, 0.35, 0.32, 0.29],  # hood 14 RPS
    [0.56, 0.50, 0.45, 0.41, 0.38],  # hood 20 RPS
    [0.68, 0.62, 0.55, 0.50, 0.46],  # hood 28 RPS
    [0.78, 0.72, 0.65, 0.59, 0.54],  # hood 33 RPS
    [0.95, 0.88, 0.80, 0.73, 0.67],  # hood 42 RPS (high arc, far)
]
FLIGHT_TIME_TABLE = BilinearInterpolatingTable(
    _FLIGHT_TIME_HOOD_KEYS,
    _FLIGHT_TIME_FLYWHEEL_KEYS,
    _FLIGHT_TIME_VALUES,
)

# ── Radial velocity -> flywheel RPS compensation ────────────────────────
# How much flywheel RPS to add/subtract per 1 m/s of radial robot velocity.
# Positive radial velocity (moving toward target) = need LESS flywheel speed.
RADIAL_VELOCITY_TO_RPS = 2.0  # RPS per m/s
