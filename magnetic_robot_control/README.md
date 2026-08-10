# Magnetic Robot Control

The application uses the official JAKA Python extension (`jkrc.pyd`) and the
standard Python library. It does not use the legacy C# project.

Set `JAKA_SDK_PYTHON_PATH` to the directory containing both `jkrc.pyd` and
`jakaAPI.dll`, for example:

```powershell
$env:JAKA_SDK_PYTHON_PATH = 'C:\path\to\Windows\python3\x64'
python app.py
```

The Python interpreter must be 64-bit and ABI-compatible with `jkrc.pyd`.
Double-click `run.bat` to launch the desktop application. Run `python -m unittest discover -s tests -v` for offline checks.

For a USB camera, install the optional preview dependencies once:

```powershell
python -m pip install -r requirements.txt
```

Then enter its Windows camera device index (normally `0`) and select `Start camera`.

The JAKA robot panel has two control modes:

- `常规`: use the upper-computer TCP target controls.
- `手动`: use a USB/Bluetooth gamepad for low-speed step movement. The left stick
  commands Cartesian X/Y steps, the right stick commands Cartesian Z steps. In
  joint mode, the left stick left/right moves the selected joint. Use the D-pad
  to select J1-J6, press A to confirm joint jog mode, and press B to return to
  Cartesian mode. If the A/B buttons or axes differ from the default mapping,
  use the raw input display to set the A/B button numbers and X/Y/Z axis
  numbers. Start with a low jog speed and small jog step.

Power on, power off, enable, disable, TCP reading, and stop-motion controls stay
in the shared robot-control area and are available in both modes.

Robot and gamepad connection buttons act as status indicators. Green means the
connection succeeded, red means the last connection attempt failed, and gray
means idle/disconnected.

The actuation map CSV must have the columns:
`x,y,z,A11,A12,A13,A21,A22,A23,A31,A32,A33`.

Coordinates are in the calibration frame and use the file's length unit.
Magnetic field units are also taken from the calibration process. Record both
units before using the result on hardware.
