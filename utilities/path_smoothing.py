# Pick bezier control points automatically so a path through a set of
# waypoints is as smooth as possible: a natural cubic spline, which is the
# curve through the points with the least total (squared) acceleration.
# Same idea as FRCProfileGenerator, with two upgrades:
#   - segments are weighted by their length (chord-length spacing), so uneven
#     waypoint spacing doesn't cause bulges and overshoot
#   - the start/end direction of travel can be pinned
#
# The Dart version in the PathPlanner fork (auto-smooth) uses the same math.
import numpy as np


def smooth_controls(
    points: np.ndarray,
    start_dir: np.ndarray | None = None,
    end_dir: np.ndarray | None = None,
    uniform: bool = False,
) -> tuple[np.ndarray, np.ndarray]:
    """Control points for the cubic beziers through `points` ((n+1, 2) array).

    Returns (first, second): for segment i (points[i] -> points[i+1]) the
    bezier is points[i], first[i], second[i], points[i+1].

    start_dir/end_dir pin the direction of travel at the ends (any length,
    only the direction is used); leave them None for a "natural" end with no
    bend. uniform=True reproduces the original FRCProfileGenerator solve,
    where every segment counts the same regardless of length.
    """
    p = np.asarray(points, dtype=float)
    n = len(p) - 1
    if n < 1:
        raise ValueError("need at least two points")
    chord = np.maximum(np.hypot(*np.diff(p, axis=0).T), 1e-6)
    h = np.ones(n) if uniform else chord
    slope = np.diff(p, axis=0) / h[:, None]  # direction (and stretch) of each segment

    # Unknowns are the tangent D_i at every point. Requiring the curve's
    # second derivative to match on both sides of each interior point gives
    # one equation per point; the ends get either a pinned tangent or "no bend".
    A = np.zeros((n + 1, n + 1))
    b = np.zeros((n + 1, 2))
    for i in range(1, n):
        A[i, i - 1] = h[i]
        A[i, i] = 2 * (h[i - 1] + h[i])
        A[i, i + 1] = h[i - 1]
        b[i] = 3 * (h[i] * slope[i - 1] + h[i - 1] * slope[i])
    # A pinned tangent is ~1 per meter with chord spacing, ~segment length with uniform
    if start_dir is None:
        A[0, 0], A[0, 1], b[0] = 2, 1, 3 * slope[0]
    else:
        A[0, 0], b[0] = 1, _unit(start_dir) * (chord[0] if uniform else 1)
    if end_dir is None:
        A[n, n - 1], A[n, n], b[n] = 1, 2, 3 * slope[n - 1]
    else:
        A[n, n], b[n] = 1, _unit(end_dir) * (chord[-1] if uniform else 1)
    D = np.linalg.solve(A, b)

    first = p[:-1] + D[:-1] * (h[:, None] / 3)
    second = p[1:] - D[1:] * (h[:, None] / 3)
    return first, second


def _unit(v: np.ndarray) -> np.ndarray:
    v = np.asarray(v, dtype=float)
    return v / np.hypot(*v)
