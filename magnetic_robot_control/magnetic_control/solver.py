from __future__ import annotations

from dataclasses import dataclass
from itertools import product
from math import isfinite
from typing import List, Optional

from .calibration import Matrix3, Vector3


@dataclass(frozen=True)
class CurrentSolution:
    currents: Vector3
    predicted_field: Vector3
    field_error: Vector3
    is_limited: bool


class CurrentSolver:
    def __init__(self, maximum_current_amps: float, regularization: float = 1e-6):
        if maximum_current_amps <= 0 or regularization < 0:
            raise ValueError("Current limit must be positive and regularization non-negative.")
        self.maximum_current_amps = maximum_current_amps
        self.regularization = regularization

    def solve(self, matrix: Matrix3, target_field: Vector3) -> CurrentSolution:
        best_currents: Optional[Vector3] = None
        best_cost = float("inf")
        for mode in product((-1, 0, 1), repeat=3):
            currents = self._solve_active_set(matrix, target_field, mode)
            if currents is None:
                continue
            error = matrix.multiply(currents) - target_field
            cost = error.squared_norm + self.regularization * currents.squared_norm
            if cost < best_cost:
                best_currents, best_cost = currents, cost
        if best_currents is None:
            raise ValueError("Unable to solve the actuation matrix.")
        predicted = matrix.multiply(best_currents)
        error = predicted - target_field
        limit = self.maximum_current_amps - 1e-9
        is_limited = any(abs(value) >= limit for value in (best_currents.x, best_currents.y, best_currents.z))
        return CurrentSolution(best_currents, predicted, error, is_limited)

    def _solve_active_set(self, matrix: Matrix3, target: Vector3, mode: tuple[int, int, int]) -> Optional[Vector3]:
        fixed = [value * self.maximum_current_amps for value in mode]
        free = [index for index, value in enumerate(mode) if value == 0]
        rhs = [target.x, target.y, target.z]
        for row in range(3):
            rhs[row] -= sum(matrix.values[row][column] * fixed[column] for column in range(3) if column not in free)
        if free:
            normal = [[sum(matrix.values[row][free[i]] * matrix.values[row][free[j]] for row in range(3)) + (self.regularization if i == j else 0.0) for j in range(len(free))] for i in range(len(free))]
            vector = [sum(matrix.values[row][free[i]] * rhs[row] for row in range(3)) for i in range(len(free))]
            result = self._gaussian_solve(normal, vector)
            if result is None:
                return None
            for index, value in zip(free, result):
                fixed[index] = value
        if any(not isfinite(value) or abs(value) > self.maximum_current_amps + 1e-9 for value in fixed):
            return None
        return Vector3(*fixed)

    @staticmethod
    def _gaussian_solve(matrix: List[List[float]], vector: List[float]) -> Optional[List[float]]:
        size = len(vector)
        augmented = [matrix[row][:] + [vector[row]] for row in range(size)]
        for pivot in range(size):
            best = max(range(pivot, size), key=lambda row: abs(augmented[row][pivot]))
            if abs(augmented[best][pivot]) < 1e-12:
                return None
            augmented[pivot], augmented[best] = augmented[best], augmented[pivot]
            for row in range(pivot + 1, size):
                factor = augmented[row][pivot] / augmented[pivot][pivot]
                for column in range(pivot, size + 1):
                    augmented[row][column] -= factor * augmented[pivot][column]
        result = [0.0] * size
        for row in range(size - 1, -1, -1):
            result[row] = (augmented[row][size] - sum(augmented[row][column] * result[column] for column in range(row + 1, size))) / augmented[row][row]
        return result
