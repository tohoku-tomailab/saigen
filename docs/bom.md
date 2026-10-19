# Bill of Materials (BOM)

Hardware used in the reference implementation described in the paper. Equivalent
instruments can be substituted; the control drivers in `src/devices/` are written
per-device and can be adapted.

## Instruments

| Item | Model | Qty | Notes |
|---|---|---|---|
| Hot-plate stirrer | IKA RET control-visc | 2 | RT–340 °C, 50–1700 rpm, RS-232/USB. Only stirring and temperature control are used; see the note below before buying this model |
| Robot arm | Dobot Magician | 2 | One per solution to prevent cross-contamination |
| Electric pipette | Sartorius Picus 2 (500–10,000 μL) | 2 | Dispensing speed electronically controlled in 9 steps, about 1–11 mL/s (nominal) |
| Electronic balance | Sartorius BCE822i-1SJP | 1 | Capacity 820 g, readability 0.01 g |
| Web camera (process monitoring) | Logitech C920n | 1 | Fixed above the system |
| Web camera (appearance imaging) | Logitech C920n | 1 | Part of the imaging system (`src/imaging/`) |
| USB digital microscope | Sanwa Supply 400-CAM106 (a Vitiny UM22) | 1 | Close-up photos of the product (`capture_microscope`) and LED on/off/brightness (`microscope_led`). One USB cable: UVC camera + CP210x serial for the control MCU. Optional |
| Digital multimeter (optional) | Keithley 2002 (8.5 digits) + a GPIB interface on the control PC (e.g. NI GPIB-USB-HS) | 1 | DC/AC voltage, DC/AC current, 2- and 4-wire resistance (`measure_dmm`). GPIB only; needs a VISA library (NI-VISA / NI-488.2). Not used for the paper's ZIF-8 experiments |
| Lock-in amplifier (optional) | NF Corporation LI5640 + the same GPIB interface | 1 | Read-only (`measure_lockin`): DATA1 / DATA2 as displayed and the reference frequency; sensitivity, time constant and display are set on the front panel. GPIB only in SAIGEN (the instrument's RS-232 is not supported). The model is discontinued. Not used for the paper's ZIF-8 experiments |
| Digital oscilloscope (optional) | Agilent DSO1014A (100 MHz, 4 channels; the other DSO1000A models work the same) | 1 | Read-only automatic measurements (`measure_scope`): Vpp, mean, RMS, frequency, period, rise/fall time, pulse widths, duty cycle, overshoot. USB (USBTMC) through NI-VISA. Set-up on the front panel. Not used for the paper's ZIF-8 experiments |
| Robot arm, pick-and-place (optional) | Hiwonder MaxArm (ESP32 / MicroPython, 3 bus servos, suction nozzle kit) | 1 | Optional robot 4 for suction pick-and-place, no pipette. USB via CH340; runs the repository's `saigen-maxarm` firmware (`src/devices/maxarm/`). Not used for the paper's ZIF-8 experiments. Add a physical switch on its 12 V servo supply: the arm has no hardware emergency stop |
| Light | NEEWER RGB62 | 1 | White/red/green/blue illumination, controlled over Bluetooth LE (`src/imaging/`) |
| IoT sensor board | Raspberry Pi Zero WH + Waveshare Environment Sensor HAT | 1 | Temperature, humidity, illuminance, UV, VOC, 3-axis acceleration and angular velocity (magnetometer not read; see `src/monitoring/README.md`) |
| Control PC | — | 1 | Connects to all devices via Wi-Fi or USB |

**Note on the hot-plate stirrer.** The RET control-visc has a built-in weighing
function, but it is of no use for this platform: IKA specifies a weighing range of
10–5000 g with an accuracy of ±(0.3 % + 2) g, so a 5 g dispense would carry an
uncertainty of about ±2 g. All mass measurements in SAIGEN are therefore
made with the electronic balance, and the driver in `src/devices/ika/` only uses the
stirring and temperature commands (IKA NAMUR command set over RS-232/USB). Any IKA
plate with that interface is sufficient; the **IKA Plate (RCT digital)** (RT–310 °C,
50–1500 rpm, RS-232 and USB, same NAMUR commands) is the cheaper choice we would
recommend for a new build. Japanese list prices on ika.com in September 2026:
RET control-visc JPY 298,000, IKA Plate (RCT digital) JPY 184,000. We have not yet
run the driver against an RCT digital ourselves; the commands it sends
(`OUT_SP_2`, `OUT_SP_4`, `START_2/4`, `STOP_2/4`, `IN_PV_x`) are documented for both
models, but confirm on your unit with `--mock` first and then a short real test.

## Fabrication

| Item | Model | Notes |
|---|---|---|
| 3D printer | Bambu Lab X1E | Used to print all fixtures in `cad/` |
| CAD software | Autodesk Fusion 360 | Source `.f3d` / `.f3z` files provided in `cad/` |

## Software / AI services

| Component | Used in |
|---|---|
| Gemini 3.5 Flash (Google) | `src/agent/` — control-code and experimental-flow generation |
| whisper-large-v3-turbo (OpenAI) | `src/voice/` — speech-to-text for voice input |
| YOLOv8 (Ultralytics, AGPL-3.0) | `detection/` — optional object detection for process monitoring |

<!-- TODO: add 3D-printing filament type/settings and any wiring or adapters used. -->
