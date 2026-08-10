from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, Tuple


@dataclass(frozen=True)
class Vector3:
    x: float
    y: float
    z: float

    def __add__(self, other: "Vector3") -> "Vector3":
        return Vector3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other: "Vector3") -> "Vector3":
        return Vector3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, scalar: float) -> "Vector3":
        return Vector3(self.x * scalar, self.y * scalar, self.z * scalar)

    @property
    def squared_norm(self) -> float:
        return self.x * self.x + self.y * self.y + self.z * self.z


@dataclass(frozen=True)
class Matrix3:
    values: Tuple[Tuple[float, float, float], Tuple[float, float, float], Tuple[float, float, float]]

    def multiply(self, vector: Vector3) -> Vector3:
        return Vector3(
            sum(self.values[0][column] * (vector.x, vector.y, vector.z)[column] for column in range(3)),
            sum(self.values[1][column] * (vector.x, vector.y, vector.z)[column] for column in range(3)),
            sum(self.values[2][column] * (vector.x, vector.y, vector.z)[column] for column in range(3)),
        )


class CalibrationGrid:
    _required_columns = ("x", "y", "z", "A11", "A12", "A13", "A21", "A22", "A23", "A31", "A32", "A33")

    def __init__(self, matrices: Dict[Tuple[float, float, float], Matrix3], xs: Iterable[float], ys: Iterable[float], zs: Iterable[float]):
        self._matrices = matrices
        self._xs, self._ys, self._zs = tuple(sorted(xs)), tuple(sorted(ys)), tuple(sorted(zs))

    @classmethod
    def from_csv(cls, path: str | Path) -> "CalibrationGrid":
        with Path(path).open("r", newline="", encoding="utf-8-sig") as file:
            reader = csv.DictReader(file)
            if reader.fieldnames is None:
                raise ValueError("Calibration CSV has no header.")
            field_names = {name.strip() for name in reader.fieldnames}
            missing = set(cls._required_columns) - field_names
            if missing:
                raise ValueError("Missing calibration columns: " + ", ".join(sorted(missing)))
            matrices: Dict[Tuple[float, float, float], Matrix3] = {}
            xs, ys, zs = set(), set(), set()
            for line, row in enumerate(reader, start=2):
                try:
                    x, y, z = (float(row[name]) for name in ("x", "y", "z"))
                    matrix = Matrix3(tuple(tuple(float(row["A{0}{1}".format(r, c)]) for c in range(1, 4)) for r in range(1, 4)))
                except (TypeError, ValueError) as error:
                    raise ValueError("Invalid number at CSV line {0}.".format(line)) from error
                key = (x, y, z)
                if key in matrices:
                    raise ValueError("Duplicate grid point at CSV line {0}.".format(line))
                matrices[key] = matrix
                xs.add(x)
                ys.add(y)
                zs.add(z)
        expected = len(xs) * len(ys) * len(zs)
        if not matrices or len(matrices) != expected:
            raise ValueError("Calibration map must be a complete rectangular grid.")
        return cls(matrices, xs, ys, zs)

    @property
    def minimum_position(self) -> Vector3:
        return Vector3(self._xs[0], self._ys[0], self._zs[0])

    @property
    def maximum_position(self) -> Vector3:
        return Vector3(self._xs[-1], self._ys[-1], self._zs[-1])

    def interpolate(self, position: Vector3) -> Matrix3:
        x_index, x_fraction = self._interval(self._xs, position.x)
        y_index, y_fraction = self._interval(self._ys, position.y)
        z_index, z_fraction = self._interval(self._zs, position.z)
        result = [[0.0] * 3 for _ in range(3)]
        for ix in range(2):
            for iy in range(2):
                for iz in range(2):
                    weight = (x_fraction if ix else 1.0 - x_fraction) * (y_fraction if iy else 1.0 - y_fraction) * (z_fraction if iz else 1.0 - z_fraction)
                    matrix = self._matrices[(self._xs[x_index + ix], self._ys[y_index + iy], self._zs[z_index + iz])]
                    for row in range(3):
                        for column in range(3):
                            result[row][column] += weight * matrix.values[row][column]
        return Matrix3(tuple(tuple(row) for row in result))

    @staticmethod
    def _interval(values: Tuple[float, ...], target: float) -> Tuple[int, float]:
        if target < values[0] or target > values[-1]:
            raise ValueError("Target position is outside the calibrated volume.")
        for index in range(len(values) - 1):
            if target <= values[index + 1]:
                return index, (target - values[index]) / (values[index + 1] - values[index])
        return len(values) - 2, 1.0
