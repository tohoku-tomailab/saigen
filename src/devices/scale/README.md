# Electronic balance control module

Controller for operating an A&D EK-610i series electronic balance over RS-232C.

## Features

- **High-precision weighing**: weight measurement at 0.01 g precision
- **Averaged weighing**: the mean of several readings
- **Tare**: zero-point reset
- **Context-manager support**: automatic connect and disconnect
- **Error handling**: comprehensive exception handling

## Usage

### Basic example

```python
from src.devices.scale import ScaleController

# Use as a context manager (recommended)
with ScaleController(port="COM3") as scale:
    # Tare
    scale.tare()
    
    # Measure the weight
    weight = scale.query_weight()
    print(f"Weight: {weight:.3f} g")
    
    # Average several readings
    mean_weight = scale.get_stable_weight(samples=5)
    print(f"Mean weight: {mean_weight:.3f} g")
```

### Manual connect and disconnect

```python
scale = ScaleController(port="COM3")

try:
    # Connect
    if scale.connect():
        # Measure
        weight = scale.query_weight()
        print(f"Weight: {weight:.3f} g")
finally:
    # Disconnect
    scale.disconnect()
```

## API reference

### ScaleController

Main class for controlling the electronic balance

#### Constructor

```python
ScaleController(port="COM3", baudrate=2400, bytesize=7, parity=serial.PARITY_EVEN, stopbits=1)
```

`parity` is passed straight to pyserial, so use its constants (`serial.PARITY_EVEN` is `"E"`).

#### Methods

- `connect() -> bool`: connect to the device
- `disconnect()`: disconnect from the device  
- `is_connected() -> bool`: check the connection state
- `query_weight() -> float`: single weight measurement
- `tare(delay=5.0) -> bool`: tare
- `get_stable_weight(samples=3, interval=0.1) -> float`: arithmetic mean of `samples` readings (stability is not checked)

## Communication settings

- **Port**: RS-232C
- **Baud rate**: 2400 bps
- **Data bits**: 7
- **Parity**: even
- **Stop bits**: 1
- **Flow control**: none

## Error handling

The following exceptions may be raised:

- `ConnectionError`: device connection error
- `TimeoutError`: response timeout
- `ValueError`: data parsing error

## Logging

Detailed logs are emitted at DEBUG level and operation logs at INFO level.

```python
import logging
logging.basicConfig(level=logging.INFO)
```
