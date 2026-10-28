# Spatial (not time-based) paths for the point-to-point follower in
# utilities/path_follower.py. A DrivePath is a dense list of evenly spaced
# points on the field, each with a robot heading and a speed cap. There is no
# time in here at all: how fast to go is decided by the follower as it drives.
#
# Paths can come from:
#   - PathPlanner .path files (we read the bezier curves ourselves, no pathplannerlib)
#   - Choreo .traj files (just the positions/headings, timing is ignored)
#   - DrivePath.through(), which builds bezier curves on the fly in a few milliseconds
import copy
import json
import math
import os
from collections.abc import Sequence

import numpy as np
import wpilib
from choreo.trajectory import SwerveTrajectory
from choreo.util.field_dimensions import FIELD_LENGTH, FIELD_WIDTH
from wpimath.geometry import Pose2d, Rotation2d, Translation2d

# Distance between points after resampling, in meters
SPACING = 0.02

# Number of points averaged when smoothing curvature
CURVATURE_SMOOTHING = 5


class DrivePath:
    """A path on the field as evenly spaced points with headings and speed caps.

    Attributes the follower uses: s (distance along the path at each point),
    x, y, heading (radians, unwrapped), speed_cap (m/s), curvature (1/m),
    heading_rate (rad per meter traveled), length, max_speed, max_accel,
    end_speed and events (name -> distance along the path).
    """

    def __init__(
        self,
        xy: np.ndarray,
        heading: np.ndarray,
        speed_cap: np.ndarray | None = None,
        events: Sequence[tuple[str, int]] = (),
        max_speed: float = math.inf,
        max_accel: float = math.inf,
        end_speed: float = 0.0,
        name: str = "",
    ) -> None:
        """Build a path from raw points. Use the from_*/through constructors
        instead of calling this directly.

        xy: (N, 2) field positions, heading: (N,) radians, speed_cap: (N,) m/s,
        events: (name, index into the raw points) pairs.
        """
        xy = np.asarray(xy, dtype=float)
        heading = np.unwrap(np.asarray(heading, dtype=float))
        if speed_cap is None:
            speed_cap = np.full(len(xy), math.inf)

        # Drop repeated points so distance along the path keeps increasing
        raw_s = _arc_length(xy)
        keep = np.concatenate(([True], np.diff(raw_s) > 1e-9))
        raw_s_kept = raw_s[keep]

        self.name = name
        self.length = float(raw_s[-1])
        self.max_speed = max_speed
        self.max_accel = max_accel
        self.end_speed = end_speed

        # Resample to even spacing so "distance along the path" is just index * SPACING
        n = max(2, math.ceil(self.length / SPACING) + 1)
        self.s = np.linspace(0.0, self.length, n)
        self.x = np.interp(self.s, raw_s_kept, xy[keep, 0])
        self.y = np.interp(self.s, raw_s_kept, xy[keep, 1])
        self.heading = np.interp(self.s, raw_s_kept, heading[keep])
        self.speed_cap = np.interp(self.s, raw_s_kept, np.asarray(speed_cap)[keep])
        self.events = {event: float(raw_s[idx]) for event, idx in events}
        self._compute_curvature()

    def _compute_curvature(self) -> None:
        """Fill in curvature and heading_rate from the current x, y and heading."""
        ds = max(self.length / (len(self.s) - 1), 1e-9)
        tangent = np.unwrap(np.arctan2(np.gradient(self.y), np.gradient(self.x)))
        curvature = np.abs(np.gradient(tangent, ds))
        # Light smoothing so a single noisy point doesn't slam the brakes. Padding
        # with the end values keeps the array the same length, even on tiny paths.
        padded = np.pad(curvature, CURVATURE_SMOOTHING // 2, mode="edge")
        kernel = np.ones(CURVATURE_SMOOTHING) / CURVATURE_SMOOTHING
        self.curvature = np.convolve(padded, kernel, mode="valid")
        self.heading_rate = np.gradient(self.heading, ds)  # rad per meter traveled

    # ---- Construction -------------------------------------------------------

    @staticmethod
    def from_pathplanner(name: str) -> "DrivePath":
        """Load deploy/pathplanner/paths/<name>.path (blue alliance, right side).

        Keeps the bezier shape, rotation targets, constraint zones (as speed
        caps), global max velocity/acceleration, goal end velocity and event
        markers. Point-towards zones are ignored.
        """
        path_file = os.path.join(
            wpilib.getDeployDirectory(), "pathplanner", "paths", name + ".path"
        )
        with open(path_file) as f:
            data = json.load(f)

        waypoints = data["waypoints"]
        segments = []
        for a, b in zip(waypoints, waypoints[1:], strict=False):
            segments.append(
                (
                    _xy(a["anchor"]),
                    _xy(a["nextControl"]),
                    _xy(b["prevControl"]),
                    _xy(b["anchor"]),
                )
            )
        xy, wrp = _sample_beziers(segments)

        # Rotation targets are placed by "waypoint relative position", which is
        # segment index + bezier t. Headings are blended between them by distance.
        start_rot = (data.get("idealStartingState") or {}).get("rotation", 0.0)
        end_rot = data["goalEndState"]["rotation"]
        targets = sorted(
            [(0.0, start_rot)]
            + [(t["waypointRelativePos"], t["rotationDegrees"]) for t in data["rotationTargets"]]
            + [(float(len(segments)), end_rot)]
        )
        knot_pos = np.array([p for p, _ in targets])
        knot_heading = np.unwrap(np.radians([r for _, r in targets]))
        raw_s = _arc_length(xy)
        heading = np.interp(raw_s, np.interp(knot_pos, wrp, raw_s), knot_heading)

        constraints = data["globalConstraints"]
        speed_cap = np.full(len(xy), math.inf)
        for zone in data["constraintZones"]:
            if zone["constraints"].get("unlimited"):
                continue
            inside = (wrp >= zone["minWaypointRelativePos"]) & (
                wrp <= zone["maxWaypointRelativePos"]
            )
            speed_cap[inside] = np.minimum(speed_cap[inside], zone["constraints"]["maxVelocity"])

        events = [
            (m["name"], _index_of(wrp, m["waypointRelativePos"])) for m in data["eventMarkers"]
        ]
        unlimited = constraints.get("unlimited", False)
        return DrivePath(
            xy,
            heading,
            speed_cap,
            events,
            max_speed=math.inf if unlimited else constraints["maxVelocity"],
            max_accel=math.inf if unlimited else constraints["maxAcceleration"],
            end_speed=data["goalEndState"]["velocity"],
            name=name,
        )

    @staticmethod
    def from_choreo(traj: SwerveTrajectory) -> "DrivePath":
        """Use a Choreo trajectory's shape, headings and events, ignoring its timing."""
        samples = traj.samples
        xy = np.array([(s.x, s.y) for s in samples])
        heading = np.array([s.heading for s in samples])
        times = np.array([s.timestamp for s in samples])
        events = [(e.event, _index_of(times, e.timestamp)) for e in traj.events]
        return DrivePath(xy, heading, events=events, name=traj.name)

    @staticmethod
    def through(
        start: Pose2d,
        end: Pose2d,
        via: Sequence[Translation2d] = (),
        start_direction: Rotation2d | None = None,
        end_direction: Rotation2d | None = None,
        max_speed: float = math.inf,
    ) -> "DrivePath":
        """Build a smooth bezier path on the fly, in field coordinates.

        The robot turns from start's heading to end's heading evenly along the
        way. start_direction/end_direction are the direction of *travel* at
        each end (pass the robot's velocity direction to start smoothly while
        moving). If left out they point at the neighbouring point.
        """
        points = [start.translation(), *via, end.translation()]
        pts = np.array([(p.x, p.y) for p in points])

        # Travel direction at each point: Catmull-Rom style through the middle points
        directions = []
        for i in range(len(pts)):
            if i == 0 and start_direction is not None:
                d = np.array([start_direction.cos(), start_direction.sin()])
            elif i == len(pts) - 1 and end_direction is not None:
                d = np.array([end_direction.cos(), end_direction.sin()])
            else:
                d = pts[min(i + 1, len(pts) - 1)] - pts[max(i - 1, 0)]
            norm = np.hypot(*d)
            directions.append(d / norm if norm > 1e-9 else np.zeros(2))

        segments = []
        for i in range(len(pts) - 1):
            handle = np.hypot(*(pts[i + 1] - pts[i])) / 3
            segments.append(
                (
                    pts[i],
                    pts[i] + directions[i] * handle,
                    pts[i + 1] - directions[i + 1] * handle,
                    pts[i + 1],
                )
            )
        xy, _ = _sample_beziers(segments)

        raw_s = _arc_length(xy)
        turn = (end.rotation() - start.rotation()).radians()
        fraction = raw_s / raw_s[-1] if raw_s[-1] > 0 else np.ones(len(raw_s))
        heading = start.rotation().radians() + turn * fraction
        return DrivePath(xy, heading, max_speed=max_speed, name="on the fly")

    # ---- Alliance / side ----------------------------------------------------

    def for_side(self, red: bool, left: bool) -> "DrivePath":
        """Paths are drawn blue alliance, right side. Mirror for left (same as
        choreo_utils.mirrored), then rotate the field for red."""
        if not (red or left):
            return self
        x, y, heading = self.x, self.y, self.heading
        if left:
            y, heading = FIELD_WIDTH - y, -heading
        if red:
            x, y, heading = FIELD_LENGTH - x, FIELD_WIDTH - y, heading + math.pi
        path = copy.copy(self)
        path.x, path.y, path.heading = x, y, heading
        path._compute_curvature()
        return path

    # ---- Queries ------------------------------------------------------------

    def pose_at(self, index: int) -> Pose2d:
        """Robot pose (position and heading) at the given point index."""
        return Pose2d(self.x[index], self.y[index], self.heading[index])

    def start_pose(self) -> Pose2d:
        """Robot pose at the start of the path."""
        return self.pose_at(0)

    def end_pose(self) -> Pose2d:
        """Robot pose at the end of the path."""
        return self.pose_at(len(self.s) - 1)

    def preview(self, every: int = 10) -> list[Pose2d]:
        """Every `every`th pose plus the last one, for drawing on a dashboard."""
        idx = list(range(0, len(self.s), every))
        if idx[-1] != len(self.s) - 1:
            idx.append(len(self.s) - 1)
        return [self.pose_at(i) for i in idx]


def _xy(point: dict) -> np.ndarray:
    """A PathPlanner {"x": .., "y": ..} point as a numpy array."""
    return np.array([point["x"], point["y"]])


def _arc_length(xy: np.ndarray) -> np.ndarray:
    """Distance along a polyline at each of its points, starting at 0."""
    return np.concatenate(([0.0], np.cumsum(np.hypot(*np.diff(xy, axis=0).T))))


def _index_of(sorted_values: np.ndarray, value: float) -> int:
    """Index of the first entry >= value, clamped to the last entry."""
    return min(int(np.searchsorted(sorted_values, value)), len(sorted_values) - 1)


def _sample_beziers(segments: list) -> tuple[np.ndarray, np.ndarray]:
    """Densely sample cubic beziers. Returns the points and, for each point,
    segment index + t (PathPlanner's "waypoint relative position")."""
    xy_parts, wrp_parts = [], []
    for i, (p0, p1, p2, p3) in enumerate(segments):
        control_len = np.hypot(*(p1 - p0)) + np.hypot(*(p2 - p1)) + np.hypot(*(p3 - p2))
        t = np.linspace(0.0, 1.0, max(2, math.ceil(control_len / 0.005)))
        if i > 0:
            t = t[1:]  # the first point is the previous segment's last point
        u = 1 - t
        pts = (
            np.outer(u**3, p0)
            + np.outer(3 * u**2 * t, p1)
            + np.outer(3 * u * t**2, p2)
            + np.outer(t**3, p3)
        )
        xy_parts.append(pts)
        wrp_parts.append(i + t)
    return np.concatenate(xy_parts), np.concatenate(wrp_parts)
