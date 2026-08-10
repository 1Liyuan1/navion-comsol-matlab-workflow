from __future__ import annotations

import importlib
import os
import sys
from typing import Sequence


_dll_directory_handles = []


class JakaRobot:
    COORD_BASE = 0
    COORD_JOINT = 1
    ABS = 0
    INCR = 1

    def __init__(self, ip_address: str, sdk_directory: str | None = None):
        self.ip_address = ip_address
        self._sdk_directory = sdk_directory or os.environ.get("JAKA_SDK_PYTHON_PATH")
        self._robot = None
        self._jkrc = None

    @property
    def is_connected(self) -> bool:
        return self._robot is not None

    def connect(self) -> None:
        if not self._sdk_directory:
            raise RuntimeError("Set JAKA_SDK_PYTHON_PATH to the directory containing jkrc.pyd and jakaAPI.dll.")
        path = os.path.abspath(self._sdk_directory)
        if path not in sys.path:
            sys.path.insert(0, path)
        if hasattr(os, "add_dll_directory"):
            _dll_directory_handles.append(os.add_dll_directory(path))
        self._jkrc = importlib.import_module("jkrc")
        self._robot = self._jkrc.RC(self.ip_address)
        self._check(self._call_first(("login", "log_in")))

    def power_on_and_enable(self) -> None:
        self._require_robot()
        self._check(self._robot.power_on())
        self._check(self._robot.enable_robot())

    def power_on(self) -> None:
        self._require_robot()
        self._check(self._robot.power_on())

    def power_off(self) -> None:
        self._require_robot()
        self._check(self._robot.power_off())

    def enable(self) -> None:
        self._require_robot()
        self._check(self._robot.enable_robot())

    def disable(self) -> None:
        self._require_robot()
        self._check(self._robot.disable_robot())

    def move_linear(self, pose: Sequence[float], speed: float, blocking: bool = True) -> None:
        self._require_robot()
        if len(pose) != 6:
            raise ValueError("A Cartesian pose must contain six values.")
        self._check(self._robot.linear_move(tuple(pose), self.ABS, blocking, speed))

    def move_joint(self, joints: Sequence[float], speed: float, blocking: bool = True) -> None:
        self._require_robot()
        if len(joints) != 6:
            raise ValueError("A joint target must contain six values.")
        self._check(self._robot.joint_move(tuple(joints), self.ABS, blocking, speed))

    def jog_cartesian(self, axis: int, speed: float, distance: float) -> None:
        self._require_robot()
        if axis < 0 or axis > 5:
            raise ValueError("Cartesian jog axis must be 0-5.")
        self._check(self._robot.jog(axis, self.INCR, self.COORD_BASE, float(speed), float(distance)))

    def jog_joint(self, joint_index: int, speed: float, distance: float) -> None:
        self._require_robot()
        if joint_index < 0 or joint_index > 5:
            raise ValueError("Joint jog index must be 0-5.")
        self._check(self._robot.jog(joint_index, self.INCR, self.COORD_JOINT, float(speed), float(distance)))

    def get_tcp_pose(self) -> tuple[float, float, float, float, float, float]:
        self._require_robot()
        response = self._robot.get_actual_tcp_position()
        self._check(response)
        if not isinstance(response, tuple) or len(response) < 2:
            raise RuntimeError("JAKA SDK did not return a TCP pose.")
        return tuple(response[1])

    def get_joint_position(self) -> tuple[float, float, float, float, float, float]:
        self._require_robot()
        response = self._robot.get_actual_joint_position()
        self._check(response)
        if not isinstance(response, tuple) or len(response) < 2:
            raise RuntimeError("JAKA SDK did not return joint positions.")
        return tuple(response[1])

    def stop(self) -> None:
        self._require_robot()
        self._check(self._robot.jog_stop(-1))

    def disconnect(self) -> None:
        if self._robot is not None:
            self._check(self._call_first(("logout", "log_out")))
            self._robot = None
            self._jkrc = None

    def _require_robot(self) -> None:
        if self._robot is None:
            raise RuntimeError("Robot is not connected.")

    def _call_first(self, method_names):
        self._require_robot()
        for method_name in method_names:
            method = getattr(self._robot, method_name, None)
            if method is not None:
                return method()
        raise RuntimeError("JAKA SDK does not expose any of these methods: {0}.".format(", ".join(method_names)))

    @staticmethod
    def _check(response) -> None:
        code = response[0] if isinstance(response, tuple) else response
        if code != 0:
            raise RuntimeError("JAKA SDK returned error code {0}.".format(code))
