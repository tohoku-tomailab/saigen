# Electronic balances

Three modules for two instruments. Only the first is used by flows.

| Module | Class | Instrument | Used by |
|---|---|---|---|
| `BCE8221.py` | `SerialBalance` | Sartorius BCE822i (the paper's cell) | `SharedDevices` for the flow actions `measure_weight` and `tare_scale` (`scale_port` in `config.yaml`) |
| `scale_controller.py` | `ScaleController` | A&D EK-610i series | exported by `src.devices.scale`; stand-alone scripts and the self-check in `docs/setup.md` §4. Not used by flows |
| `Ek610i.py` | `Scale` | A&D EK-610i series | older minimal driver, kept for scripts that import it; nothing in this repository uses it |

The flow executor only knows `SerialBalance`. Running flows on an A&D balance needs a
change in `SharedDevices` (see `docs/adding-an-instrument.md`), not a new driver.

## Sartorius BCE822i (`SerialBalance`)

Communication: 9600 bps, 8 data bits, odd parity, 1 stop bit. The balance's print
command `P` returns one line; `T` tares.

```python
from src.devices.scale.BCE8221 import SerialBalance

balance = SerialBalance(port="COM8")     # opens the port in the constructor
try:
    balance.tare()                       # sends T and returns at once
    grams = balance.get_weight()         # float, or None when the reply holds no number
finally:
    balance.close()
```

- `get_weight() -> float | None`: sends `P`, reads one line and parses the fixed-width
  printout (`[marker] [sign] [spaces] value [unit]`, e.g. `-     0.03 g` or
  `N  +  0.00 g`), keeping the sign. Returns `None` and logs a warning when no number
  is found. `SharedDevices.measure_weight` takes the median of several readings and
  skips `None`.
- `tare()`: sends `T`; does not wait for the balance.
- `close()`: closes the port.

## A&D EK-610i (`ScaleController`)

Controller for operating an A&D EK-610i series electronic balance over RS-232C.

### Features

- **High-precision weighing**: weight measurement at 0.01 g precision
- **Averaged weighing**: the mean of several readings
- **Tare**: zero-point reset
- **Context-manager support**: automatic connect and disconnect
- **Error handling**: comprehensive exception handling

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

### Constructor

```python
ScaleController(port="COM3", baudrate=2400, bytesize=7, parity=serial.PARITY_EVEN, stopbits=1)
```

`parity` is passed straight to pyserial, so use its constants (`serial.PARITY_EVEN` is `"E"`).

### Methods

- `connect() -> bool`: connect to the device
- `disconnect()`: disconnect from the device  
- `is_connected() -> bool`: check the connection state
- `query_weight() -> float`: single weight measurement
- `tare(delay=5.0) -> bool`: tare
- `get_stable_weight(samples=3, interval=0.1) -> float`: arithmetic mean of `samples` readings (stability is not checked)

### Communication settings

- **Port**: RS-232C
- **Baud rate**: 2400 bps
- **Data bits**: 7
- **Parity**: even
- **Stop bits**: 1
- **Flow control**: none

### Error handling

The following exceptions may be raised:

- `ConnectionError`: device connection error
- `TimeoutError`: response timeout
- `ValueError`: data parsing error

### Logging

Detailed logs are emitted at DEBUG level and operation logs at INFO level.

```python
import logging
logging.basicConfig(level=logging.INFO)
```

## A&D EK-610i, minimal driver (`Ek610i.Scale`)

Same instrument and the same serial settings as `ScaleController`, with a smaller API
(`Q` for a reading, `R` for tare). Kept for scripts that already import it.

```python
from src.devices.scale.Ek610i import Scale

with Scale(port="COM3") as scale:       # the port is opened on enter, closed on exit
    scale.tare(delay=0.5)               # sends R, then waits delay seconds
    grams = scale.query_weight()        # TimeoutError on no reply, ValueError when the reply holds no number
```

`query_weight()` parses replies of the form `ST,+0012.34 g` (`US,` for an unstable
reading) and keeps the sign; earlier versions dropped it, so negative weights came back
positive (`tests/test_ek610i_sign.py`).
