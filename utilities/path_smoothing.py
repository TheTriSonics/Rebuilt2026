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
) -> tuple[np.ndarray, np.ndarray]:
    """Control points for the cubic beziers through `points` ((n+1, 2) array).

    Returns (first, second): for segment i (points[i] -> points[i+1]) the
    bezier is points[i], first[i], second[i], points[i+1].

    start_dir/end_dir pin the direction of travel at the ends (any length,
    only the direction is used). Leave them None, or pass a zero-length
    vector, for a "natural" end with no bend.
    """
    p = np.asarray(points, dtype=float)
    n = len(p) - 1
    if n < 1:
        raise ValueError("need at least two points")
    h = np.maximum(np.hypot(*np.diff(p, axis=0).T), 1e-6)
    slope = np.diff(p, axis=0) / h[:, None]  # unit direction of each segment
    start_dir, end_dir = _unit(start_dir), _unit(end_dir)

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
    if start_dir is None:
        A[0, 0], A[0, 1], b[0] = 2, 1, 3 * slope[0]
    else:
        A[0, 0], b[0] = 1, start_dir
    if end_dir is None:
        A[n, n - 1], A[n, n], b[n] = 1, 2, 3 * slope[n - 1]
    else:
        A[n, n], b[n] = 1, end_dir
    D = np.linalg.solve(A, b)

    first = p[:-1] + D[:-1] * (h[:, None] / 3)
    second = p[1:] - D[1:] * (h[:, None] / 3)
    return first, second


def _unit(v: np.ndarray | None) -> np.ndarray | None:
    """v scaled to length 1, or None if v is None or (nearly) zero length."""
    if v is None:
        return None
    v = np.asarray(v, dtype=float)
    norm = np.hypot(*v)
    return v / norm if norm > 1e-9 else None
