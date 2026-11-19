# USB digital microscope (UVC) control module

Controls a USB digital microscope through OpenCV and saves still images for the
experiment record. The paper's cell uses the Sanwa Supply **400-CAM106**. This model,
like many USB microscopes, appears to the OS as a UVC (USB Video Class) camera, so no
vendor driver is needed and it is handled with the same `cv2.VideoCapture` as a webcam.

## Files

- `microscope_controller.py` — `MicroscopeController` (subclass of `WebcamController`): video (UVC)
- `um22_serial.py` — `UM22SerialController`: serial communication with the control MCU (LED on/off and brightness, focus motor, status and model read-out)

## Differences from the webcam driver

| Item | `WebcamController` | `MicroscopeController` |
|---|---|---|
| Output directory | `captured_images/` | `microscope_images/` |
| Default resolution | 1280x720 | **3840x2160** (change with `shared_devices.microscope_resolution` in config.yaml) |
| Frames discarded before capture | 5 | 10 (`warmup_frames`) |
| Exposure | auto | auto; can be fixed manually with `exposure=` |
| Uniform-frame detection | none | warns (`is_uniform`) |

From a flow it is used through the shared-device actions `capture_microscope` (capture),
`microscope_led` (LED) and `microscope_focus` (focus); none of them requires a `robot_id`.
`SharedDevices.capture_microscope(file_path)`, `set_microscope_led(on, level)` and
`focus_microscope(mode, ...)` each call the classes in this module.

## LED illumination control (UM22 family)

Inside the 400-CAM106 is a Vitiny **UM22**: behind the single USB cable sits an internal
hub with the UVC camera (video) and a **CP210x USB serial port (control MCU)** attached to
it. LED on/off and brightness, zoom and autofocus are controlled over this serial port.
The COM port number differs from PC to PC, so set it in `config.yaml` as
`shared_devices.microscope_port` (leave it empty for capture only, without LED control).

Protocol (reverse-engineered from the manufacturer's UM Viewer 1.000.096 and verified on
the instrument on 2026-09-22):

| Item | Details |
|---|---|
| Line settings | 115200 bps, 8 data bits, no parity, **2 stop bits**; commands are **CR-terminated** |
| Reply | `@` + two hex digits + CR (e.g. `@0C`) |
| Read | `R00` + address. 0x00 = status (`01` LED on / `03` off), 0x06 = LED level, 0x10 = LED mode, 0x2F/2E/2D = firmware version |
| Model | `S00` + address, ten times from 0x2C down to 0x23 (`UM22TW0100`) |
| Write | `W` + four hex digits of ((address << 8) \| key \| value) |
| LED on/off | **toggle** `W0159` (same as the LED button in UM Viewer). `set_led(on)` reads the status and sends it only when needed |
| LED brightness | `W06` + level (factory 12; 4 is dim, 12 and above is bright enough to saturate the image) |
| **Caution** | **Asserting RTS resets the MCU and power-cycles the camera** (about 7 s). pyserial asserts RTS on open by default, so set `rts=False` before opening (this class already does) |

## Focusing (focus motor)

| Item | Details |
|---|---|
| Lens position read-out | `R0009` (high byte) and `R000A` (low byte). Observed 346–1766 on the instrument; factory 1568 |
| Busy detection | bit 0x04 of the status register (`R0000`) is set. Motor mode is `R0004` (0 idle, 2 AF search in progress) |
| AF mode | `W0154` manual / `W0155` one-shot AF / `W0156` continuous AF |
| Go to position | `W18` + high byte, `W19` + low byte, then `W016E` (1755 → 1568 moves exactly, in 4.6 s) |
| Step move | press with `W0152` (in) / `W0153` (out), release with `W0192` / `W0193`. On the instrument, out increases the position value |
| Side effect of writing the LED level | writing `W06xx` turns the LED on. To change the brightness and leave the LED off afterwards, use `set_led(False, level=...)`; the level write turns it on briefly before it is toggled off again |

`autofocus()` sends AF, waits for the motor to stop and returns `(position, converged)`.
When the position or status cannot be read repeatedly it raises `ConnectionError` (a lost
connection is not mistaken for "stopped"). `request_stop()` stops an operation in progress
from another thread, and the exception and interrupt paths also send manual mode (`W0154`)
to stop the motor. `step_focus()` returns `(position, completed)`.
**AF only works while the video stream is running** (`SharedDevices` keeps reading frames
during focusing). On the instrument, with a printed digit "4" as the target, AF swept back
and forth across the full range once and converged to position 1600 in about 40 s (sharpness 6.0, versus 2–4 at
other positions).
**Without texture in the field of view, AF never converges and keeps searching** (more than
60 s on blank paper). In that case the timeout returns the motor to manual mode and stops the
search. If a white target saturates the image, lower the LED level or shorten the exposure
before running AF.
For reproducibility, read the position with `get_motor_position()` once focus has been found,
record it, and return to the same position next time with `goto_position()` (in a flow,
`mode: "position"`).

The stand-alone AF example and self-check below do not open the camera themselves:
something else (another thread or process) must keep reading frames from the
microscope's video stream throughout the autofocus, or it will not converge.

```python
with UM22SerialController("COM10") as scope:
    scope.set_led(True, level=4)
    pos, ok = scope.autofocus(timeout_s=60)     # -> (1234, True)
    scope.goto_position(1568)                    # to a recorded position
    scope.step_focus("out", steps=2)
```

Self-check: `python -m src.devices.microscope.um22_serial COM10 af` / `... goto 1568`

```python
from src.devices.microscope import UM22SerialController

with UM22SerialController("COM10") as scope:
    print(scope.get_model(), scope.get_firmware())   # UM22TW0100 01.03.00
    scope.set_led(False)                              # off
    scope.set_led(True, level=8)                      # on, brightness 8
```

Self-check: `python -m src.devices.microscope.um22_serial COM10 [on|off]`

## Usage

```python
from src.devices.microscope import MicroscopeController

with MicroscopeController(camera_index=2) as scope:      # the index differs per machine
    print(scope.get_camera_info())
    scope.capture_and_save("sample_001.jpg")             # -> microscope_images/sample_001.jpg
```

Self-check (connects, lists the cameras and takes one picture):

```bash
python -m src.devices.microscope.microscope_controller 2   # argument: camera index
```

## Resolution and data volume

Measured (DirectShow, 2026-09-23): 3840x2160 ≈ 14 fps, 1280x720 ≈ 6 fps, 1920x1080 ≈ 2 fps.
4K is both the sharpest and the fastest, so it is the default. One JPEG is 0.4–2 MB
(depending on the detail in the subject; text on a white background is 0.4 MB), so if an
experiment takes hundreds of pictures or disk space or transfer bandwidth is limited, reduce
the resolution with `microscope_resolution: "1280x720"` (≈ 100–200 KB per picture).

## Identifying the device (400-CAM106)

- In Device Manager it appears as **UVC Video Device** (VID `EB1A` / PID `279F`, an eMPIA
  chip). The Logitech C920 webcam is VID `046D`.
- The OpenCV index follows the enumeration order and changes with the PC and the USB port.
  On Windows, listing the DirectShow names with `pygrabber` shows which index is which (the
  self-check command above prints them).
- Once known, set the index in `config.yaml` as `shared_devices.microscope_index`.

## Notes

- If the lens cap is still on the objective or the microscope is pressed against the
  subject, the image is a uniform single colour. `capture_image` warns in that case (the
  capture itself still happens).
- The 400-CAM106 has its own LED illumination. Brightness is adjusted on the device.
- Exposure is OpenCV's `CAP_PROP_EXPOSURE` (on Windows a logarithmic value, roughly −13 to
  −1). Specify `exposure=` only when auto exposure saturates.
- If another application (the Windows Camera app, the manufacturer's UM Viewer, etc.) has
  the microscope open, no frames can be obtained (the connection succeeds but `read()` keeps
  failing).
- **The manufacturer's UM Viewer, on "connect", sends a query at 115200 bps to every COM port
  on the PC.** This reaches the Dobot, Picus 2 and balance ports too, so do not start UM
  Viewer during an experiment.
- **Green cast (white subjects photographed green): cause and remedy.** After power-on
  (including an MCU reset), this model does not apply its white-balance gains until the host
  has written the UVC white-balance control once, so R and B are about half of G (confirmed on
  the instrument on 2026-09-22/23; the image only looks white when it is saturated). The
  manufacturer's UM Viewer writes WB on connect, which is why it looked fine there.
  `MicroscopeController.connect()` writes a manual WB of 5000 K (this model's default) by
  default (`white_balance=`; `None` skips it), so normally no remedy is needed when going
  through saigen. Media Foundation cannot write WB, which is why on Windows the camera is
  opened with DirectShow first, falling back to Media Foundation; if the WB write fails,
  `connect()` logs a warning and the green cast may remain. The LED brightness is not the cause (once
  WB is applied, the image is neutral even at level 4).
- **AF converges only while the video stream is running** (the AF figure of merit comes from
  the video pipeline). `SharedDevices.focus_microscope` keeps reading camera frames while
  focusing, so nothing needs to be done when used from a flow. When using
  `UM22SerialController` on its own, open a `cv2.VideoCapture` and keep calling `read()`
  while calling `autofocus()`.
- An MCU reset (`reset_mcu()`) keeps the lens position and resets the LED level to its
  factory default of 12.

## Dependencies

- `opencv-python` (`numpy`)
- `pygrabber` (Windows only; used by the self-check command to show camera names. Works without it)
- `pyserial` (LED control; imported lazily inside `um22_serial.py`, so not needed for capture only)
