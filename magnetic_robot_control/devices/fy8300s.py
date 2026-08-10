from __future__ import annotations

from abc import ABC, abstractmethod


class ThreeChannelGenerator(ABC):
    @abstractmethod
    def set_channel_amplitude_vpp(self, channel: int, voltage: float) -> None:
        raise NotImplementedError

    @abstractmethod
    def set_outputs_enabled(self, enabled: bool) -> None:
        raise NotImplementedError

    def emergency_zero(self) -> None:
        for channel in (1, 2, 3):
            self.set_channel_amplitude_vpp(channel, 0.0)
        self.set_outputs_enabled(False)


class Fy8300s(ThreeChannelGenerator):
    def __init__(self, port: str):
        self.port = port

    def set_channel_amplitude_vpp(self, channel: int, voltage: float) -> None:
        raise RuntimeError("FY8300S protocol is not configured. Do not connect outputs until verified commands are supplied.")

    def set_outputs_enabled(self, enabled: bool) -> None:
        raise RuntimeError("FY8300S protocol is not configured. Do not connect outputs until verified commands are supplied.")
