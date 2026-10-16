# IKA hot-plate stirrer control module

This module provides the classes and functions for controlling an IKA RET control-visc
hot-plate stirrer over serial communication in the experiment-automation system.

## Overview

`ika_controller.py` provides the interface for operating the IKA hot-plate stirrer. Its
main features are:

- Serial connection management
- Temperature control
- Stirring-speed control
- Weighing (RET control-visc only; see the note below)
- Data recording and plotting

With this module the temperature and stirring of an experiment can be controlled
precisely and monitored in real time.

## Usage

### Basic example

```python
from src.devices.ika.ika_controller import IKAController
import time

# Initialise the IKA controller (give the port name)
device = IKAController(port='COM8')

# Reset the device (optional)
device.reset_device()

# Set the temperature and stirring speed and start stirring (40 °C, 100 rpm)
device.start_stirring(40, 100)

# Stir for 5 minutes
time.sleep(300)

# Stop stirring
device.stop_stirring()

# Close the connection
device.close()
```

### Temperature control and monitoring

```python
# Run for 10 minutes at 40 °C and 100 rpm (with a live plot)
temperatures, times = device.set_temperature(40, 100, 10)

# Run for 10 minutes at 40 °C and 100 rpm (without a plot)
device.stirring_at_temperature(40, 100, 10)
```

### Weighing

```python
# Measure the weight
weight = device.measure_weight()
print(f"Measured weight: {weight}")
```

### Control with experiment parameters

```python
from src.devices.ika.ika_controller import IKAController, run_temperature_experiment

# Initialise the IKA controller
device = IKAController(port='COM17')

# Define the experiment parameters
params = {
    "temperature": 45,        # set temperature (°C)
    "stabilize_speed": 200,   # stirring speed during the stabilisation phase (rpm)
    "stabilize_minute": 5,    # duration of the stabilisation phase (minutes)
    "stirring_speed": 150,    # stirring speed during the stirring phase (rpm)
    "stirring_minute": 15     # duration of the stirring phase (minutes)
}

# Run the experiment
temperatures, times = run_temperature_experiment(device, params)

# Convert the data to a pandas DataFrame and save it as CSV
import pandas as pd
data = pd.DataFrame({
    "Time (minutes)": times,
    "Temperature (°C)": temperatures
})
data.to_csv("experiment_data.csv", index=False)
```

## Main functions

### Connection management

- `__init__(port, baudrate, timeout)` - initialise the controller and open the serial connection
- `is_connected()` - check the connection state
- `send_command(command)` - send a command
- `close()` - close the serial port

### Device control

- `reset_device()` - reset the device
- `start_pc_mode(X)` - start PC mode (X: mode number)
- `stop_pc_mode(X)` - stop PC mode (X: mode number)
- `read_data(X)` - read data from the device (X: read-mode number)

### Stirring and temperature control

- `set_stirring_speed(speed)` - set the stirring speed
- `start_stirring(temperature, speed)` - set the temperature and stirring speed and start stirring
- `stop_stirring()` - stop stirring and temperature control
- `set_temperature(temperature, speed, wait_minute)` - set the temperature and stirring speed and run for the given time (with a live plot)
- `stirring_at_temperature(temperature, speed, wait_minute)` - set the temperature and stirring speed and run for the given time (without a plot)

### Weighing

- `check_weight_function_ready()` - check whether the weighing function is ready
- `wait_for_weight_function_ready(max_wait_time, check_interval)` - wait until the weighing function is ready
- `measure_weight()` - measure the weight with the device's built-in weighing function

### Experiment utilities

- `run_temperature_experiment(device, params)` - run a temperature experiment

## IKA command reference

The main IKA commands are:

- `RESET` - reset the device
- `START_X` - start PC mode X (X: mode number)
- `STOP_X` - stop PC mode X (X: mode number)
- `IN_PV_X` - read the current value of mode X (X: mode number)
- `OUT_SP_X Y` - set the setpoint of mode X to Y (X: mode number, Y: setpoint)
- `STATUS_X` - get the status of mode X (X: mode number)

### Mode numbers

- `2` - temperature control
- `4` - stirring-speed control
- `90` - weighing

## Data recording and plotting

The `set_temperature()` method plots the temperature data live during the experiment and
returns the data when it finishes. The data can be saved to a CSV file or used for further
analysis.

```python
# Save the data to CSV
import pandas as pd
import os
from datetime import datetime

# Use the current date and time as the folder name
current_time = datetime.now().strftime("%Y%m%d_%H%M")
folder_name = f"data_{current_time}"
os.makedirs(folder_name, exist_ok=True)

data = pd.DataFrame({
    "Time (minutes)": times,
    "Temperature (°C)": temperatures
})

file_name = "temperature_data.csv"
data.to_csv(os.path.join(folder_name, file_name), index=False)
```

## Notes

- **The weighing function is not used in the experimental flows.** The weighing function
  built into the RET control-visc has a range of 10–5000 g and an accuracy of
  ±(0.3 % + 2) g (IKA's published figures), which is far too coarse for judging dispensed
  amounts of a few grams. All mass measurements are made with the electronic balance
  (`src/devices/scale/`). `measure_weight()` and related methods only work on the RET
  control-visc and are kept for compatibility only. The stirring and temperature commands
  (`OUT_SP_2`, `OUT_SP_4`, `START_x`, `STOP_x`, `IN_PV_x`) are IKA's common NAMUR commands,
  so they should also work on cheaper models with RS-232/USB (e.g. the IKA Plate (RCT
  digital)). Operation on the RCT digital has not been verified yet, however.
- The serial port name (e.g. COM8) differs between machines. Check it in Device Manager or
  the equivalent.
- Temperature control is affected by the surroundings, so reaching the set temperature can
  take some time.
- For long experiments, back up the data regularly.
- The stirring speed can be set from 0 to 1000 rpm, but the appropriate value depends on
  the viscosity of the liquid and the vessel.

## Dependencies

- `serial` - serial communication
- `time` - timing
- `matplotlib` - plotting
- `pandas` - data handling (optional)
- `datetime` - date and time handling
- `os` - file operations

## Troubleshooting

- On a connection error, check that the port name is correct and that the device is
  switched on.
- If plotting misbehaves, change `matplotlib.use("TkAgg")` to suit your environment
  (e.g. `Qt5Agg`).
- If the temperature does not stabilise, check the surroundings, the amount of liquid and
  the material of the vessel.
