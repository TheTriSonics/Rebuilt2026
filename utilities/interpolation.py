"""Linear and bilinear interpolation tables for FRC shooter tuning.

Usage (1D):
    table = InterpolatingTable({
        1.0: 10.0,
        2.0: 20.0,
        3.0: 25.0,
    })
    value = table.get(1.5)  # returns 15.0 (interpolated)
    value = table.get(0.5)  # returns 10.0 (clamped to lowest)
    value = table.get(5.0)  # returns 25.0 (clamped to highest)

Usage (2D — bilinear):
    table = BilinearInterpolatingTable(
        row_keys=[10.0, 20.0, 30.0],    # e.g. hood RPS
        col_keys=[30.0, 40.0, 50.0],    # e.g. flywheel RPS
        values=[
            [0.25, 0.22, 0.20],  # hood=10: flat shot
            [0.45, 0.40, 0.36],  # hood=20: mid arc
            [0.70, 0.62, 0.55],  # hood=30: high arc
        ],
    )
    value = table.get(15.0, 35.0)  # bilinear interpolation
"""

from bisect import bisect_right


class InterpolatingTable:
    def __init__(self, data: dict[float, float]):
        if len(data) < 2:
            raise ValueError("InterpolatingTable needs at least 2 data points")
        self._keys = sorted(data.keys())
        self._values = [data[k] for k in self._keys]

    def get(self, key: float) -> float:
        if key <= self._keys[0]:
            return self._values[0]
        if key >= self._keys[-1]:
            return self._values[-1]

        idx = bisect_right(self._keys, key) - 1
        k0 = self._keys[idx]
        k1 = self._keys[idx + 1]
        t = (key - k0) / (k1 - k0)
        return self._values[idx] + t * (self._values[idx + 1] - self._values[idx])


class BilinearInterpolatingTable:
    """2D interpolation table using bilinear interpolation on a grid.

    row_keys and col_keys define the grid axes (must be sorted ascending).
    values[i][j] is the output at (row_keys[i], col_keys[j]).
    Inputs outside the grid are clamped to the nearest edge.
    """

    def __init__(
        self,
        row_keys: list[float],
        col_keys: list[float],
        values: list[list[float]],
    ):
        if len(row_keys) < 2 or len(col_keys) < 2:
            raise ValueError("BilinearInterpolatingTable needs at least 2 keys per axis")
        if len(values) != len(row_keys):
            raise ValueError(f"Expected {len(row_keys)} rows of values, got {len(values)}")
        for i, row in enumerate(values):
            if len(row) != len(col_keys):
                raise ValueError(f"Row {i} has {len(row)} values, expected {len(col_keys)}")
        self._rows = row_keys
        self._cols = col_keys
        self._values = values

    def _clamp_and_find(self, keys: list[float], key: float) -> tuple[int, int, float]:
        """Find the bracketing indices and interpolation fraction for a key."""
        if key <= keys[0]:
            return 0, 0, 0.0
        if key >= keys[-1]:
            last = len(keys) - 1
            return last, last, 0.0

        idx = bisect_right(keys, key) - 1
        t = (key - keys[idx]) / (keys[idx + 1] - keys[idx])
        return idx, idx + 1, t

    def get(self, row_key: float, col_key: float) -> float:
        r0, r1, rt = self._clamp_and_find(self._rows, row_key)
        c0, c1, ct = self._clamp_and_find(self._cols, col_key)

        # Bilinear interpolation: interp along cols for each row, then between rows
        val_r0 = self._values[r0][c0] + ct * (self._values[r0][c1] - self._values[r0][c0])
        val_r1 = self._values[r1][c0] + ct * (self._values[r1][c1] - self._values[r1][c0])
        return val_r0 + rt * (val_r1 - val_r0)
