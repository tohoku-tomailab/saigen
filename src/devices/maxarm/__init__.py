"""
Hiwonder MaxArm (ESP32 + MicroPython) driver: SAIGEN firmware, wire protocol and PC client.

- ``firmware/``          -- the command server installed on the arm (``main.py`` +
  ``saigen_maxarm.py``), replacing Hiwonder's factory ``main.py``
- ``protocol.py``        -- checksummed line codec shared by the client and the tests
- ``client.py``          -- ``MaxArmClient`` (pyserial, imported lazily)
- ``deploy_firmware.py`` -- human-run backup / install / restore tool (raw REPL)

The flow executor never uses this package directly; it goes through
``src.devices.safety.MaxArmRobot``. See ``README.md`` in this directory.
"""
from .client import (
    MaxArmClient,
    MaxArmCommandError,
    MaxArmMoveError,
    MaxArmResetDetected,
    MaxArmStatus,
    MaxArmTimeout,
)
from .protocol import FIRMWARE_NAME, PROTOCOL_VERSION, MaxArmError, MaxArmProtocolError

__all__ = [
    "MaxArmClient",
    "MaxArmStatus",
    "MaxArmError",
    "MaxArmProtocolError",
    "MaxArmCommandError",
    "MaxArmTimeout",
    "MaxArmResetDetected",
    "MaxArmMoveError",
    "FIRMWARE_NAME",
    "PROTOCOL_VERSION",
]
