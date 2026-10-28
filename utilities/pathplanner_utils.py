# File for all of the PathPlanner related paths
from functools import cache

from pathplannerlib.config import RobotConfig
from pathplannerlib.path import PathPlannerPath
from pathplannerlib.trajectory import PathPlannerTrajectory
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds


@cache
def robot_config() -> RobotConfig:
    """Robot config from deploy/pathplanner/settings.json (edited via the PathPlanner GUI)."""
    return RobotConfig.fromGUISettings()


def load_pp_trajectory(name: str, red: bool, left: bool) -> PathPlannerTrajectory:
    """Load deploy/pathplanner/paths/<name>.path and generate a trajectory for it.

    Paths are drawn as blue alliance, right side. `left` mirrors across the
    field's long axis (same as choreo_utils.mirrored) and `red` flips to the
    red alliance. Generation is slow in Python, so do this while disabled
    (e.g. from get_initial_pose), not in the middle of auton.
    """
    path = PathPlannerPath.fromPathFile(name)
    if left:
        path = path.mirrorPath()
    if red:
        path = path.flipPath()

    start_pose = path.getStartingHolonomicPose()
    start_rotation = start_pose.rotation() if start_pose else Rotation2d()
    return path.generateTrajectory(ChassisSpeeds(), start_rotation, robot_config())


def pp_event_pose(name: str, traj: PathPlannerTrajectory, event: str) -> Pose2d:
    """Pose where the event marker named `event` in <name>.path is reached, on a
    trajectory from load_pp_trajectory (so already flipped/mirrored). Returns
    Pose2d() if there's no such marker, same as AutonBase.get_event_pose."""
    markers = [
        m for m in PathPlannerPath.fromPathFile(name).getEventMarkers() if m.triggerName == event
    ]
    if not markers:
        return Pose2d()
    position = markers[0].waypointRelativePos
    for state in traj.getStates():
        if state.waypointRelativePos >= position:
            return state.pose
    return traj.getEndState().pose
