# USB webcam (process photos)

Driver for a USB webcam (UVC) read through OpenCV. SAIGEN uses it for process
photos of the cell: the flow action `capture_and_save` (a shared device, no
`robot_id`) takes one picture and saves it as a JPEG. The USB digital microscope
driver in `../microscope/` subclasses `WebcamController` and adds the LED and
focus control.

## Configuration

```yaml
shared_devices:
  camera_index: 0     # OpenCV device index of the webcam used by capture_and_save
```

The camera is opened only when a flow contains `capture_and_save`. The driver's own
default index is 1; the value from `config.yaml` (or the `.env` override, see
`docs/setup.md`) is what `SharedDevices` passes in. The capture backend is chosen per
OS: Media Foundation on Windows, Video4Linux2 on Linux, AVFoundation on macOS.

## Usage

```python
from src.devices.webcam import WebcamController

with WebcamController(camera_index=0, resolution=(1280, 720)) as cam:
    path = cam.capture_and_save()                # captured_images/<YYYYMMDD>/capture_<YYYYMMDD_HHMMSS>.jpg
    path = cam.capture_and_save("step_01.jpg")   # captured_images/step_01.jpg
```

## API

- `WebcamController(camera_index=1, resolution=None)`: `resolution` is requested
  from the camera as `(width, height)`; the default is 1280x720, and the size the
  camera actually delivers is logged at INFO. The save directory
  `captured_images/` (relative to the current working directory) is created on
  construction.
- `connect() -> bool`: opens the camera. Returns `False` (and logs the reason) instead
  of raising when the index cannot be opened. `disconnect()` releases it. The context
  manager calls both.
- `capture_image() -> numpy.ndarray | None`: reads and discards five frames so the
  exposure can settle, then returns one frame (BGR). `None` on failure.
- `save_image(image, filename=None) -> str | None`: with `filename=None` the file goes
  to `captured_images/<YYYYMMDD>/capture_<timestamp>.jpg`; an absolute path is used
  as given; a relative path is placed under `captured_images/` unless it already starts
  with that directory. Missing parent directories are created. Returns the path written,
  `None` on failure.
- `capture_and_save(filename=None) -> str | None`: the two above in one call. This is what
  `SharedDevices.capture_and_save` uses; a `None` result is logged as a warning.
- `show_preview(duration=0)`: opens an OpenCV window (`q` closes it; `duration` seconds
  if given). Needs a display; not for headless use.
- `get_camera_info() -> dict`: index, width, height, fps and backend name.
- `capture_experiment_image(step_name, camera_index=1)`: convenience function that
  opens the camera, saves `experiment_<step_name>_<timestamp>.jpg` and closes it again.

## Connection check

Run by a human (no robot is involved; the camera is the only device opened):

```bash
python -m src.devices.webcam.webcam_controller 0      # the argument is the camera index (default 1)
```

It prints the camera information, shows a five-second preview window and saves two
images under `captured_images/`.
