from choreo.trajectory import SwerveTrajectory, SwerveSample
from choreo.util.field_dimensions import FIELD_WIDTH


def _mirror_sample(s: SwerveSample) -> SwerveSample:
    return SwerveSample(
        s.timestamp,
        s.x,
        FIELD_WIDTH - s.y,
        -s.heading,
        s.vx,
        -s.vy,
        -s.omega,
        s.ax,
        -s.ay,
        -s.alpha,
        [s.fx[1], s.fx[0], s.fx[3], s.fx[2]],
        [-s.fy[1], -s.fy[0], -s.fy[3], -s.fy[2]],
    )


def mirrored(traj: SwerveTrajectory) -> SwerveTrajectory:
    """Return a left-right mirror of the trajectory (Y-axis reflection).
    Use when a Right-side path should be run from the Left side of the same alliance."""
    return SwerveTrajectory(
        traj.name,
        [_mirror_sample(s) for s in traj.samples],
        traj.splits,
        traj.events,  # timestamps unchanged — events fire at same times
    )
