# Point-to-point path follower. Instead of asking "where should the robot be at
# time t?" (Choreo / PathPlanner style), every loop it asks "where is the robot
# along the path?" and drives toward a point a little further ahead. If the
# robot gets bumped or slowed down, it just picks up from wherever it is.
#
# Speed is planned by distance, not time:
#   - never faster than max_speed, the path's own limits, or what the curve allows
#     (sideways acceleration on a turn of radius r is v^2 / r)
#   - slows down early enough to make those limits and to stop at the end
#   - speeds up no faster than max_accel
import math
from dataclasses import dataclass

import numpy as np
from wpimath.geometry import Pose2d
from wpimath import angleModulus

from utilities.drive_path import DrivePath


@dataclass
class FollowerConfig:
    max_speed: float = 4.0  # m/s
    max_accel: float = 3.0  # m/s^2
    max_decel: float = 3.0  # m/s^2
    max_lateral_accel: float = 4.0  # m/s^2, how hard we're allowed to corner
    lookahead: float = 0.3  # m, how far ahead of the robot to aim...
    lookahead_per_speed: float = 0.15  # ...plus this many seconds of travel
    end_kp: float = 4.0  # (m/s) per m of distance left when settling at the end
    heading_kp: float = 5.0  # (rad/s) per rad of heading error
    max_omega: float = 3 * math.pi  # rad/s
    position_tolerance: float = 0.03  # m
    heading_tolerance: float = math.radians(2.0)


class PathFollower:
    # How far ahead of the last known spot to look for the robot each loop.
    # Searching only forward keeps paths that cross themselves from skipping ahead.
    SEARCH_AHEAD = 1.0  # m

    def __init__(self, path: DrivePath, config: FollowerConfig, start_speed: float = 0.0):
        self.path = path
        self.config = config
        self.speed = start_speed
        self.index = 0
        self.finished = False
        self.lookahead_pose = path.start_pose()
        self._spacing = path.s[1] if path.length > 0 else 1.0
        self._end_index = len(path.s) - 1
        self._end_pose = path.end_pose()
        self._profile = self._plan_speeds()

    def _plan_speeds(self) -> np.ndarray:
        cfg, path = self.config, self.path
        cap = np.minimum(path.speed_cap, min(cfg.max_speed, path.max_speed))
        with np.errstate(divide="ignore"):
            cap = np.minimum(cap, np.sqrt(cfg.max_lateral_accel / path.curvature))
        # Walk backwards so we start slowing down before each slow spot. The end
        # of the path is handled separately in update(), using the robot's real
        # distance to the end.
        two_a_ds = 2 * cfg.max_decel * self._spacing
        for i in range(len(cap) - 2, -1, -1):
            cap[i] = min(cap[i], math.sqrt(cap[i + 1] ** 2 + two_a_ds))
        return cap

    @property
    def progress(self) -> float:
        """Distance along the path in meters."""
        return float(self.path.s[self.index])

    def passed(self, event: str) -> bool:
        """True once the robot is past the named event marker."""
        return event in self.path.events and self.progress >= self.path.events[event]

    def update(self, pose: Pose2d, dt: float) -> tuple[float, float, float]:
        """Returns field-relative (vx, vy, omega) to drive this loop."""
        cfg, path = self.config, self.path
        rx, ry = pose.x, pose.y

        # Where are we along the path? Closest point in a window ahead of last time.
        hi = min(self.index + int(self.SEARCH_AHEAD / self._spacing) + 1, self._end_index + 1)
        window = slice(self.index, hi)
        dist_sq = (path.x[window] - rx) ** 2 + (path.y[window] - ry) ** 2
        self.index += int(np.argmin(dist_sq))

        # Distance left: along the path, but never less than straight-line to the
        # end so we still drive in if we're off to the side of the last point
        to_end = math.hypot(self._end_pose.x - rx, self._end_pose.y - ry)
        remaining = max(path.length - self.progress, to_end)

        max_accel = min(cfg.max_accel, path.max_accel)
        speed = min(
            float(self._profile[self.index]),
            self.speed + max_accel * dt,
            math.sqrt(path.end_speed**2 + 2 * cfg.max_decel * remaining),
        )
        if path.end_speed == 0:
            speed = min(speed, cfg.end_kp * remaining)
        self.speed = speed

        # Aim at a point ahead of us on the path
        ahead = cfg.lookahead + cfg.lookahead_per_speed * speed
        target = min(self.index + int(ahead / self._spacing), self._end_index)
        dx, dy = path.x[target] - rx, path.y[target] - ry
        norm = math.hypot(dx, dy)
        vx, vy = (speed * dx / norm, speed * dy / norm) if norm > 1e-6 else (0.0, 0.0)
        self.lookahead_pose = path.pose_at(target)

        # Heading: P on the error plus how fast the path wants us to be turning
        heading_error = angleModulus(path.heading[self.index] - pose.rotation().radians())
        omega = cfg.heading_kp * heading_error + path.heading_rate[self.index] * speed
        omega = max(-cfg.max_omega, min(cfg.max_omega, omega))

        end_heading_error = angleModulus(
            self._end_pose.rotation().radians() - pose.rotation().radians()
        )
        if path.end_speed > 0:
            # Meant to roll straight into something else, so don't wait to settle
            self.finished = self.index == self._end_index
        else:
            self.finished = (
                to_end < cfg.position_tolerance and abs(end_heading_error) < cfg.heading_tolerance
            )
        return vx, vy, omega
