# Imaging module

Appearance-imaging system for SAIGEN, the automated experimentation platform (see the
[repository README](../../README.md) for the paper, in press at STAM Methods, that this
module's results belong to). It backlights a sample vial with an RGB light,
photographs it under white, red, green and blue illumination with a fixed
camera, and turns those photographs into cropped side-by-side comparisons and
depth-resolved transmitted-light intensity profiles — the material behind the
paper's "Imaging system for appearance evaluation" section and Figure 6.

## System configuration

### Hardware

| Component | Model | Interface | Notes |
|---|---|---|---|
| Light | NEEWER RGB62 | Bluetooth LE | White (CCT) and single-colour (HSI) modes; controlled by [`neewer_light.py`](neewer_light.py) |
| Camera | Logitech C920n (HD Pro Webcam C920) | USB | Video + control abstracted by [`hw.py`](hw.py) |
| Holding stand | 3D-printed | — | Fixes the relative positions of light, vial and camera; STL and Fusion 360 sources in [`cad/imaging-stand/`](../../cad/imaging-stand/), together with a light shield placed over the stand |
| Control PC | — | — | Runs `camera_server.py`, `acquire.py`, `make_panel.py` |

See [`docs/bom.md`](../../docs/bom.md) for the full platform bill of materials.

### Geometry

The stand holds the light, sample vial and camera on one fixed optical axis so
every photograph is a backlit transmission image of the same region:

```
 NEEWER RGB62        sample vial            Logitech C920n
 (backlight)  -->  (screw-top vial,   -->  (USB, fixed focus
                    suspension below         and exposure)
                    clear supernatant)
   |____________________|____________________|
        all three fixed by the stand (cad/imaging-stand/)
```

### Software-fixed conditions

Imaging conditions are fixed in software rather than by hand, so repeated
photographs are comparable:

- **Camera** — [`config/camera_reference.json`](config/camera_reference.json)
  is the single source of truth: exposure 20 (macOS `exposure-time-abs`
  units; logged Windows equivalent `-9`), gain 0, focus 180, white balance
  4600 K manual. It stores this once per platform (macOS `uvc-util` control
  names under `"reference"`, Windows logical names under `"win32"`), applied
  through `hw.py`'s logical control names; `camera_server.py`'s `/api/health`
  and the acquisition scripts check the live camera against it. Every run
  records the settings **read back from the camera** at capture time
  (`camera_settings` in `summary.json`), not these reference constants.
- **Light** — [`config/light_reference.json`](config/light_reference.json)
  documents the illumination conditions used by `acquire.py`: a white CCT
  shot at 5600 K (used only for detecting the vial geometry; its brightness
  is auto-swept unless fixed with `--white-intensity`), and three HSI
  (single-colour) shots at hue 0° / 120° / 240° (red / green / blue), each at
  1 % intensity. This file is not read by any module at run time — it is a
  human-readable record of constants that are hard-coded in `acquire.py`.

**Two hard rules, both established by measurement on the C920:**

1. **White balance must stay manual at 4600 K. Never switch it back to
   auto.** The C920 has slow, hidden internal white-balance state; every time
   it is switched between auto and manual it settles into a different,
   non-reproducible colour state. Only an uninterrupted manual 4600 K holds a
   consistent colour balance.
2. **Changing exposure while the camera server's stream is running has no
   effect on the frames it serves.** The control write succeeds and reads
   back correctly, but the live MJPEG stream keeps using the exposure that
   was in effect when it started. To change exposure: set the new value,
   then restart `camera_server.py`. Adjusting brightness *during* a
   measurement should be done on the light, not the camera.

The three illumination wavelengths (625 / 525 / 465 nm) are the LED's
**nominal** values, not independently measured (±10-15 nm uncertainty); this
does not affect relative or time-series comparisons. Each single-colour LED
saturates its own dominant camera channel (red→R, green→G, blue→B) even at
1 % intensity, so photometry is always done on the **secondary** channel
(red→G, green→B, blue→G) — see `acquire.py`'s `COLORS` table and
`od_profile()`.

## Module map

| File | Role |
|---|---|
| [`paths.py`](paths.py) | Shared directory layout (`IMAGING_DATA_DIR`, `config/`, `examples/`) and `uvc-util` path resolution |
| [`hw.py`](hw.py) | Platform abstraction for the camera (macOS: ffmpeg + uvc-util; Windows: OpenCV/DirectShow) and the cross-platform measurement file lock |
| [`neewer_light.py`](neewer_light.py) | BLE control of the NEEWER RGB62 (CCT and HSI modes; macOS CoreBluetooth / Windows WinRT+bleak backends) |
| [`capture.py`](capture.py) | Thin HTTP client used by `acquire.py` to trigger captures and read/write camera controls through `camera_server.py` |
| [`camera_server.py`](camera_server.py) + [`static/`](static/) | Local web server (port 8799): live preview, camera/light controls, spot-measurement launcher, analysis viewer, and the HTML/JS front end |
| [`acquire.py`](acquire.py) | Capture sequence (white → red → green → blue) and the optical-density analysis (`od_profile`, sediment-front detection, figure generation) |
| [`geometry.py`](geometry.py) | Vial geometry detection from a white backlit photo (meniscus, cap, body, liquid bottom) |
| [`make_panel.py`](make_panel.py) | Publication panel figure: four cropped photographs plus the depth-resolved transmittance/OD curve |
| [`config/`](config/) | `camera_reference.json` (single source of truth for camera settings), `camera_local.example.json` (per-PC camera selection template), `light_reference.json` (documentation of the light constants) |
| [`examples/`](examples/) | One complete example run (see below) |

## Installation

From the repository root:

```bash
pip install -r requirements.txt
```

Platform-specific notes for this module:

- **macOS** — video comes from `ffmpeg` (AVFoundation) and camera control
  from `uvc-util`; both are external executables, **not bundled** here.
  Install `ffmpeg` with `brew install ffmpeg`; build `uvc-util` from
  [github.com/jtfrey/uvc-util](https://github.com/jtfrey/uvc-util) and put it
  on `PATH`, or point the `UVC_UTIL` environment variable at the binary (see
  `paths.uvc_util_path()`). The light backend uses PyObjC (`pyobjc-framework-CoreBluetooth`,
  `pyobjc-framework-libdispatch`), which `requirements.txt` installs on macOS only.
- **Windows** — video and camera control both go through OpenCV's DirectShow
  backend, which is why `requirements.txt` pins `opencv-python==4.12.0.88`:
  4.13+ (including 5.0) breaks get/set of `CAP_PROP_WHITE_BALANCE_BLUE_U` on
  the C920, the property used to hold the required manual 4600 K white
  balance. `pygrabber` (already listed) enumerates DirectShow devices and
  pulls in `comtypes` automatically. The light backend prefers piggybacking
  on an already-connected device via the `winrt` packages (not pinned) and
  falls back automatically to scanning with `bleak` (already listed) if they
  are absent. On a PC with more than one webcam, copy
  `config/camera_local.example.json` to `config/camera_local.json`
  (git-ignored) and set `win32_device_name`/`win32_device_path` to pick the
  C920 unambiguously — `hw.py` refuses to start on zero or multiple matches.
- **macOS camera selection** — ffmpeg opens the camera by name
  (`HD Pro Webcam C920`) and `hw.py` looks up the `uvc-util` index of that
  same name (`uvc-util -d`), so controls always go to the streamed camera;
  it refuses to start if the name is missing or matches more than one
  device.
- **The light** must be powered on before starting `camera_server.py`. On
  macOS, `neewer_light.py` piggybacks onto an existing BLE connection rather
  than opening its own, so it can share a connection already held by the
  NEEWER Control Center app instead of competing with it. If more than one
  NEEWER light is reachable, `neewer_light.py` refuses to guess: set
  `NEEWER_DEVICE_ID` in the repository's `.env` (or the environment) to part
  of the device id shown in the error message (macOS: peripheral UUID;
  Windows: WinRT device id or Bluetooth address). Every BLE write is
  checked; a device error or a missing acknowledgement aborts the command.
- **`IMAGING_DATA_DIR`** (environment variable) sets where captured photos
  and analysis output go. Default: `imaging_data/` at the repository root
  (`photos/` and `analysis/` subdirectories; see `paths.py`). The location is
  anchored to the module file, not to the current directory, so the camera
  server and a hand-run `acquire.py` always write to the same place. A
  relative value is resolved against the directory the server was started
  from, and the server passes the resolved path (and its actual `--port`,
  as `IMAGING_SERVER_URL`) to the `acquire.py` child it launches.

## Usage

### 1. Start the camera server

```bash
python camera_server.py
```

Open `http://localhost:8799`. The page has five tabs:

- **Camera** — sliders for exposure, gain, focus and white balance (Kelvin),
  auto-exposure/auto-focus/auto-white-balance toggles, and a manual
  **Capture** button; a warning banner reminds you to keep white balance
  manual at 4600 K, and flags that exposure changes need a server restart to
  reach the live preview. While a measurement holds the lock, control writes
  from this tab (or any other client) are refused with HTTP 409; only the
  running acquisition, which registers an owner token, may write.
- **Light** — On/Off, and either White (CCT: brightness + colour temperature)
  or Monochrome (HSI: hue + saturation + brightness, with quick Red 625 /
  Green 525 / Blue 465 buttons) control.
- **Measure** — a named spot measurement: Full (white+RGB) or White-only
  mode, folder and sample name, and toggles for a locked frame, fixed
  monochromatic LED intensity, or fixed white LED intensity (white-only
  mode). This is the UI front end for `acquire.py`, run as a subprocess and
  polled via `/api/spectral/status`.
- **Analysis** — a Refresh button and the most recent run's `spectral.png`.
- **Conditions** — one-shot frame (geometry) calibration, and named presets
  that save/apply/verify a full camera+light+geometry configuration.
  Applying a preset checks every camera write and reads the settings back;
  a failed write or a differing read-back is reported and the light and
  geometry are left untouched. If the stream is stale afterwards (macOS),
  the response says the server must be restarted before measuring: per
  rule 2 above, neither the live view nor captured photos see the change
  until then.

### 2. Apply and verify the reference camera settings

Set the camera to the values in `config/camera_reference.json` (exposure 20 /
gain 0 / focus 180 / white balance manual 4600 K) using the Camera tab or the
`/api/control` endpoint, then confirm with:

```bash
curl http://localhost:8799/api/health
```

`matches_reference` should be `true`; `mismatches` lists any control that
differs from `camera_reference.json`, and `stream_settings_stale` flags the
"exposure changed but the stream didn't update" condition described above.

### 3. Acquire a run

```bash
python acquire.py --folder <experiment> --name <sample>
```

This drives the light through white → red → green → blue, capturing one
photo per step through the running `camera_server.py`, and writes the run to
`<IMAGING_DATA_DIR>/photos/<experiment>/<sample>/`. Before the first photo it
fixes and reads back the white balance, then queries `/api/health` and
**aborts if the stream is stale** (settings changed after the stream
started, e.g. white balance switched from auto on this run — restart the
server and run again), if there is no fresh frame, or if the controls
cannot be read. A mismatch against `camera_reference.json` is only a
warning (the reference exposure depends on the sample) but is recorded. Useful options (see
`python acquire.py --help` for the full list):

| Option | Effect |
|---|---|
| `--mode {full,white}` | `full` (default): white + red/green/blue, ~3-wavelength OD. `white`: a single white shot only, giving per-camera-channel (R/G/B) OD — quicker, but only valid for named runs |
| `--white-intensity PCT` | Fix the white-shot LED intensity (white-only mode only) so multiple samples are photographed under identical illumination, for photo comparison or absolute-transmittance comparison |
| `--fixed-intensity` | Disable automatic monochromatic-LED intensity adjustment and hold it at `--intensity` (for cross-sample absolute comparison) |
| `--calibrate` | Detect the vial geometry once from a white reference and save it as a locked frame |
| `--locked` | Skip per-run geometry auto-detection and reuse the locked frame (for comparing swapped-in samples without shifts in field of view) |
| `--intensity`, `--restore` | Initial monochromatic LED intensity (default 1 %) and the light state to leave the light in when done (default: off) |

### 4. Run directory format

Each run directory contains:

| File | Contents |
|---|---|
| `ref_white.jpg` | White-light reference photo (geometry detection; also the photometry image itself in white-only mode) |
| `shot_red.jpg`, `shot_green.jpg`, `shot_blue.jpg` | Monochromatic captures (full mode only) |
| `profiles.csv` | Per-row optical density, one column per wavelength/channel (`od_625`/`od_525`/`od_465` in full mode, `od_R`/`od_G`/`od_B` in white-only mode), each followed by `od_<label>_cap`, the uncorrected profile computed with the cap pedestal alone (identical when `pedestal_mode` is `cap`) |
| `spectral.png` | OD-vs-depth figure generated by `acquire.py` at capture time |
| `summary.json` | Geometry (`meniscus_y`, `liquid_bottom_y`, `body_x0`/`x1`, `fill_fraction`), per-wavelength intensity/channel/warning plus `pedestal_rule`, `i0` and `analysis_warning`, `sediment_front_y`, `white_balance_fixed` (commanded), `camera_settings` (read back before the first photo), `camera_settings_after` / `camera_settings_changed` (read back after the last), `camera_device`, `camera_matches_reference`/`camera_mismatches`, and measured headspace brightness used to cross-check actual light intensity. Written before the figure, so a plotting failure never loses it; never contains NaN/inf |

### 5. Make the publication panel

```bash
python make_panel.py examples/zif8_vial_5s --out panels
```

Outputs `<run_dir name>_panel.png` (300 dpi), `_panel.pdf`, and a draft
`_caption.md`, all under `--out` (default `./panels`) — `make_panel.py` never
writes into the run directory itself. Options (`python make_panel.py --help`):

| Option | Effect |
|---|---|
| `--y {transmittance,od}` | Right-panel x-axis: transmittance `T = 10^-OD` (default) or OD directly |
| `--with-white` | Also draw the white (G-channel) curve in the depth panel; omitted by default |
| `--mm-per-px MM` | Show the depth axis in millimetres (5 mm steps) instead of pixel rows |

The panel shows four cropped photographs (white, 625, 525, 465 nm, cap bottom
to vial bottom, all cropped to the same rectangle) beside a depth-resolved
transmittance/OD curve that shares the same vertical (pixel-row) scale as the
photographs, so visual features in the photos line up with inflections in the
curve. The white photograph is not plotted as a curve by default: its green
channel is saturated over a clear supernatant (the sensor clips before the
signal can be used quantitatively), so it is shown only as a visual
reference; `--with-white` draws it anyway (as a plain solid line). A
clip table for all four channels — per row, the fraction of pixels at
sRGB >= 250 in the inner 60 % of the body columns, summarised over the
liquid rows — is always printed to stdout regardless of that flag.

The draft caption is built from the run's `summary.json` (LED intensities,
the camera settings and device read back at capture time) and from those
measured clip fractions. Anything the run did not record is written as
"unknown" — e.g. the included example predates the camera read-back, so its
caption says the exposure is unknown rather than quoting the reference.

## Correspondence with paper Figure 6

| Figure 6 panel | Produced by |
|---|---|
| (a) appearance of the imaging system | The stand ([`cad/imaging-stand/`](../../cad/imaging-stand/)) and camera/light hardware described in [System configuration](#system-configuration) above |
| (b) a photograph taken with it | `examples/zif8_vial_5s/ref_white.jpg`, captured by `camera_server.py` via `acquire.py` |
| (c) sample images under white/red/green/blue illumination | The four cropped photo columns produced by `make_panel.py` from that same run |
| (d) depth profile of the transmitted-light intensity | The depth-resolved transmittance/OD curve in `make_panel.py`, computed from `profiles.csv` (written by `acquire.py`'s `od_profile()`) |

One example run is included, [`examples/zif8_vial_5s/`](examples/zif8_vial_5s/)
— the run behind those Figure 6 panels. It is a ZIF-8 suspension (white
sediment below a clear supernatant) synthesised by the platform in a
screw-top vial without stirring, with the second precursor solution
("solution 2") dispensed over 5 s, and photographed on 2026-07-14 at 13:25,
twelve days after synthesis.

It is included as a **worked example of the pipeline, not as a validated
measurement**, and it is not a dispensing-time comparison: it is a single
run at a single dispensing time. Its white reference and its 465 nm channel
are both sensor-clipped over the clear supernatant, and it predates the
reference camera exposure, so absolute transmittance is not comparable
across runs. [`examples/README.md`](examples/README.md) lists these caveats
in full — read it before reusing the numbers.

Figure 8 of the paper (nine vials, three per dispensing time) was **not**
taken with this imaging stand and is not reproduced by anything in this
directory.

## Analysis method

`acquire.py`'s `od_profile()` computes, for each wavelength/channel, a
depth-resolved optical density `OD(y) = -log10(I(y)/I0)`:

- Pixel values come from the secondary colour channel, averaged over the
  inner 60 % of the vial body's columns (avoiding light piped along the
  glass wall), then sRGB-linearised (inverse gamma) before any arithmetic.
- `I0` is the peak transmitted-light level just below the meniscus (past the
  refractive dip the curved liquid surface produces).
- A pedestal (lens flare + dark current) is normally estimated from the dark
  region above the cap; if attenuation is deep enough that the signal pins to
  stray light instead of continuing to fall, a flat low-value window further
  down is used as the floor instead and the curve is truncated there (a
  lower bound on OD only). Which rule chose the pedestal is recorded
  (`pedestal_rule`: `cap`, `flat_window` or `min_then_rise`); the
  minimum-then-rise rule is a heuristic, so it adds an `analysis_warning`,
  and the uncorrected cap-pedestal profile is saved next to the corrected
  one for comparison.
- A uniformly dark image, or a geometry that does not fit the image, gives
  an empty profile with a warning instead of aborting the run.
- The sedimentation front is the shallowest depth where the 625 nm OD rises
  above a fixed threshold for a sustained run of rows.

## Limitations / notes

- The 625 / 525 / 465 nm wavelengths are the LED's nominal values, not
  independently measured (±10-15 nm); verify with a spectrometer or
  diffraction grating before using them quantitatively rather than for
  relative/time-series comparison.
- The geometry detector (`geometry.py`) and the fixed-frame comparison mode
  assume a single vial on a fixed optical axis; it is not a general-purpose
  multi-sample imaging pipeline.
- Camera behaviour (the white-balance hidden state, the live-exposure
  pitfall, per-channel saturation under single-colour LEDs) is specific to
  the Logitech C920/C920n and was established by measurement on that
  camera; a different webcam model may not share these quirks — or may have
  different ones.
- This is a photographic transmission measurement, not a calibrated
  spectrophotometer: optical density values are useful for relative and
  time-series comparison under matched conditions, not as absolute
  extinction coefficients.

## License

MIT, as the rest of `src/` — see [`LICENSE`](../../LICENSE).
