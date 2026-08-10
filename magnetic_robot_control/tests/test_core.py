import tempfile
import unittest
from pathlib import Path
import sys
import importlib.util

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import numpy as np

from magnetic_control import CalibrationGrid, CurrentSolver, Matrix3, Vector3
from vision_processing import detect_black_ring_center


class MagneticControlTests(unittest.TestCase):
    def test_solver_recovers_identity_solution(self):
        matrix = Matrix3(((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)))
        solution = CurrentSolver(2.0).solve(matrix, Vector3(0.5, -1.0, 1.5))
        self.assertAlmostEqual(solution.currents.x, 0.5, places=5)
        self.assertAlmostEqual(solution.currents.y, -1.0, places=5)
        self.assertAlmostEqual(solution.currents.z, 1.5, places=5)

    def test_solver_limits_current(self):
        matrix = Matrix3(((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)))
        solution = CurrentSolver(1.0).solve(matrix, Vector3(3.0, 0.0, 0.0))
        self.assertAlmostEqual(solution.currents.x, 1.0, places=5)
        self.assertTrue(solution.is_limited)

    def test_grid_interpolation(self):
        header = "x,y,z,A11,A12,A13,A21,A22,A23,A31,A32,A33\n"
        rows = []
        for x in (0.0, 1.0):
            for y in (0.0, 1.0):
                for z in (0.0, 1.0):
                    value = x + y + z
                    rows.append("{0},{1},{2},{3},0,0,0,{3},0,0,0,{3}\n".format(x, y, z, value))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "grid.csv"
            path.write_text(header + "".join(rows), encoding="utf-8")
            matrix = CalibrationGrid.from_csv(path).interpolate(Vector3(0.5, 0.5, 0.5))
        self.assertEqual(matrix.values[0][0], 1.5)

    def test_black_ring_detection(self):
        if importlib.util.find_spec("cv2") is None:
            self.skipTest("opencv-python is not installed in the current test runtime.")
        frame = np.full((480, 640, 3), 255, dtype=np.uint8)
        center_x = 265
        center_y = 195
        radius = 60
        thickness = 12
        import cv2

        cv2.circle(frame, (center_x, center_y), radius, (0, 0, 0), thickness)
        result, mask = detect_black_ring_center(frame)
        self.assertIsNotNone(mask)
        self.assertTrue(result.detected)
        self.assertAlmostEqual(result.centroid_x, center_x, delta=4.0)
        self.assertAlmostEqual(result.centroid_y, center_y, delta=4.0)
        self.assertGreater(result.deviation, 0.0)


if __name__ == "__main__":
    unittest.main()
