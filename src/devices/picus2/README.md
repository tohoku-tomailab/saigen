# Picus 2 electronic pipette control module

This module provides the classes and functions for controlling a Picus 2 electronic
pipette over USB serial or Bluetooth in the experiment-automation system.

## Overview

`picus2_controller.py` provides an asynchronous interface for operating the Picus 2
electronic pipette. Its main features are:

- Bluetooth connection management
- Emulation of button presses
- Aspirating and dispensing liquid
- Tip ejection
- Complete expulsion of liquid (blow-out)
- Mixing liquid

The module uses asynchronous communication built on `asyncio` and the `bleak` library.

## Usage

### Basic example

```python
import asyncio
from src.devices.picus2 import Picus2Controller, Buttons, ConnectionType

async def main():
    # Initialise the Picus 2 controller (Bluetooth MAC address; the default
    # connection type is USB, where the address is the serial port name)
    picus = Picus2Controller("XX:XX:XX:XX:XX:XX",
                             connection_type=ConnectionType.BLUETOOTH)
    
    # Connect
    await picus.connect()
    
    try:
        # Enable motor mode
        await picus.set_motor_mode(True)
        
        # Aspirate 5 mL (speed 5)
        await picus.aspirate(amount=5.0, speed=5)
        
        # Dispense 5 mL (speed 5)
        await picus.dispense(amount=5.0, speed=5)
        
        # Expel the remaining liquid completely
        await picus.blow_out()
        
        # Eject the tip
        await picus.eject_tip()
        
    finally:
        # Disable motor mode
        await picus.set_motor_mode(False)
        
        # Disconnect
        await picus.disconnect()

# Run the coroutine
asyncio.run(main())
```

### Button presses

```python
# Press a particular button
await picus.button(Buttons.TRIGGER_BUTTON_POWER)
await picus.button(Buttons.TRIGGER_BUTTON_LEFT)
await picus.button(Buttons.TRIGGER_BUTTON_RIGHT)
```

### Mixing

```python
# Mix 3.0 mL of liquid 5 times at speed 5
await picus.mix(cycles=5, speed=5, amount=3.0)
```

## Main functions

### Connection management

- `connect()` - connect to the Picus 2
- `disconnect()` - disconnect from the Picus 2
- `send_command(command)` - send a command

### Button presses

- `button(button, interval)` - press the given button

### Motor control

- `set_motor_mode(mode)` - set motor mode

### Liquid handling

- `aspirate(amount, speed)` - aspirate liquid
- `dispense(amount, speed)` - dispense liquid
- `blow_out(go_home, speed, delay_ms)` - expel all liquid
- `mix(cycles, speed, amount)` - mix liquid
- `eject_tip()` - eject the tip

### Utilities

- `calculate_operation_time(amount, speed)` - calculate how long a pipette operation takes
- `wait_until_END_received()` - wait until the END message is received
- `debug_print(message)` - print a debug message

## Button constants

The `Buttons` class provides the following constants:

- `TRIGGER_BUTTON_POWER` - power button
- `TRIGGER_BUTTON_LEFT` - left button
- `TRIGGER_BUTTON_MIDDLE` - middle button
- `TRIGGER_BUTTON_RIGHT` - right button
- `TRIGGER_BUTTON_TOP` - top trigger button
- `TRIGGER_BUTTON_PRESET` - preset button
- `TRIGGER_BUTTON_TIPEJECT` - tip-eject button
- `UP` - up button
- `DOWN` - down button

## Notes

### Finding the Bluetooth MAC address

To find the Bluetooth MAC address of the Picus 2:

1. Enable Bluetooth on the pipette
2. Detect it with the computer's Bluetooth scan
3. Read the MAC address from the device information (e.g. "XX:XX:XX:XX:XX:XX")

### Asynchronous programming

This module uses asynchronous programming with Python `asyncio`:

- The device-operation methods are defined with `async def` and must be called with
  `await`; synchronous helpers such as `calculate_operation_time()` and `debug_print()`
  are not
- The main script must also be defined as a coroutine and run with `asyncio.run()`

### Debug mode

To enable debugging:

```python
picus = Picus2Controller("XX:XX:XX:XX:XX:XX",
                         connection_type=ConnectionType.BLUETOOTH)
picus.DEBUG = True  # enable debug output
```

### Operation time

Aspirating, dispensing and similar operations take a certain time. The
`calculate_operation_time()` method returns the estimated duration in seconds, based on
the speed (1–9) and the volume (mL); `aspirate()` and `dispense()` wait for that duration.

### Error handling

When sending a command or a button/trigger fails (`serial.SerialException`, a Bleak
exception, a closed port, etc.), a `Picus2CommandError` (a subclass of `ConnectionError`)
is raised. Previously `button()` swallowed these exceptions and returned normally, so a
failed aspirate/dispense trigger was recorded as "completed" and the volume `LabRobot`
believed it was holding could diverge from reality. On a USB connection, if the check after
opening the port fails, the port is closed before the exception propagates. Import
`Picus2CommandError` from `src.devices.picus2.picus2_controller` (it is also caught by
`except ConnectionError`).

## Dependencies

- `asyncio` - asynchronous processing
- `bleak` - Bluetooth Low Energy communication
- `random` - random values for testing
- `time` - timing

## Troubleshooting

- On a connection error, check that the Picus 2 battery is sufficiently charged and that
  Bluetooth is enabled.
- If you see an error asking you to enable motor mode, call `set_motor_mode(True)` before
  the operation.
- If an operation does not complete, the operation-time estimate may be inaccurate. Adjust
  the wait time to match the actual operation time.
