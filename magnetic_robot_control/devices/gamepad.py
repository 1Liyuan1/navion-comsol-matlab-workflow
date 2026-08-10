from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class GamepadState:
    connected: bool
    name: str
    left_x: float = 0.0
    left_y: float = 0.0
    right_y: float = 0.0
    hat_x: int = 0
    hat_y: int = 0
    button_a: bool = False
    button_b: bool = False
    axes: tuple[float, ...] = ()
    buttons: tuple[bool, ...] = ()
    hats: tuple[tuple[int, int], ...] = ()


class Gamepad:
    """Small wrapper around pygame so the GUI can start without a gamepad."""

    def __init__(self):
        self._pygame = None
        self._joystick = None

    @property
    def is_open(self) -> bool:
        return self._joystick is not None

    def open(self, index: int = 0) -> None:
        try:
            import pygame
        except ImportError as error:
            raise RuntimeError("Gamepad support requires pygame. Run: python -m pip install -r requirements.txt") from error

        self._pygame = pygame
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() <= index:
            raise RuntimeError("No gamepad was found. Connect a USB/Bluetooth controller first.")
        self._joystick = pygame.joystick.Joystick(index)
        self._joystick.init()

    def close(self) -> None:
        if self._joystick is not None:
            self._joystick.quit()
        self._joystick = None
        if self._pygame is not None:
            self._pygame.joystick.quit()
        self._pygame = None

    def poll(self) -> GamepadState:
        if self._pygame is None or self._joystick is None:
            return GamepadState(False, "No gamepad")
        self._pygame.event.pump()
        axis_count = self._joystick.get_numaxes()
        button_count = self._joystick.get_numbuttons()
        axes = tuple(self._axis(index, axis_count) for index in range(axis_count))
        buttons = tuple(self._button(index, button_count) for index in range(button_count))
        hats = tuple(self._joystick.get_hat(index) for index in range(self._joystick.get_numhats()))
        hat_x, hat_y = (0, 0)
        if self._joystick.get_numhats() > 0:
            hat_x, hat_y = hats[0]
        return GamepadState(
            connected=True,
            name=self._joystick.get_name(),
            left_x=axes[0] if len(axes) > 0 else 0.0,
            left_y=-(axes[1] if len(axes) > 1 else 0.0),
            right_y=-(axes[3] if len(axes) > 3 else 0.0),
            hat_x=hat_x,
            hat_y=hat_y,
            button_a=buttons[0] if len(buttons) > 0 else False,
            button_b=buttons[1] if len(buttons) > 1 else False,
            axes=axes,
            buttons=buttons,
            hats=hats,
        )

    def _axis(self, index: int, axis_count: int) -> float:
        if self._joystick is None or index >= axis_count:
            return 0.0
        value = self._joystick.get_axis(index)
        if abs(value) < 0.05:
            return 0.0
        return max(-1.0, min(1.0, value))

    def _button(self, index: int, button_count: int) -> bool:
        if self._joystick is None or index >= button_count:
            return False
        return bool(self._joystick.get_button(index))
