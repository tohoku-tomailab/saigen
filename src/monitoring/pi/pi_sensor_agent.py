"""Sensor agent for the Raspberry Pi Zero WH + Waveshare Environment Sensor HAT.

Reads the on-board I2C sensors and pushes a JSON line per sample over TCP to the
control PC running the dashboard (see ``src/monitoring/dashboard/launch_sensor_dashboard.py``).

Sensors read (I2C addresses on the default bus, ``/dev/i2c-1``):

    - BME280   0x76  temperature, humidity
    - TSL25911 0x29  visible+IR light, raw channel-0 count (``lux_raw``, NOT lux)
    - ICM20948 0x68  3-axis acceleration, 3-axis angular velocity
    - LTR390   0x53  UV index (UVI)
    - SGP40    0x59  VOC (raw signal, not converted to an index)

Note: the ICM20948 also contains an AK09916 magnetometer accessible through its
auxiliary I2C bus, but this agent does NOT read it. Only the accelerometer and
gyroscope of the ICM20948 are used.

Note on the UV channel: recordings made before 2026-09-08 were taken with the
LTR390 left in ALS mode, in which the UVS data registers never update, so their
``uvi`` column is 0 throughout. From this version the sensor is initialised in
UVS mode and ``uvi`` carries a UV index (float). The CSV column name is
unchanged, so old files stay parseable — only the semantics changed.

Note on humidity: before 2026-09-23 the BME280 humidity was divided by 1024
twice, so older recordings hold 0-0.098 instead of 0-100 %RH (multiply by 1024
to recover it, to a resolution of 1 %RH). See ``compensate_humidity``.

Note on light: the TSL25911 value was sent as ``lux`` until 2026-09-23, but it
has always been the raw channel-0 (visible + IR) ADC count at the power-on gain
(1x) and integration time (100 ms), not lux. It is now sent as ``lux_raw``; the
dashboard maps the old key of older agents onto the new one.

Failed reads: a sensor whose initialisation or read fails reports ``null`` for
its values (never 0), and every message carries an ``ok`` mapping with one
boolean per sensor. Initialisation of a missing or failed sensor is retried
with exponential backoff (1 s doubling up to 60 s) while the others keep
reporting. The SGP40 reply is CRC-checked and a corrupted word is dropped; the
SGP40 is compensated with the BME280's measured humidity and temperature.

Dependencies are kept minimal on purpose (the Pi Zero has limited resources and
this script should run with nothing beyond the Python standard library plus
``smbus``/``smbus2``): no PyYAML, no third-party sensor drivers.

Configuration (environment variables, all optional except the host):

    SENSOR_AGENT_PC_HOST      Required. Hostname or IP address of the control PC.
                               There is no default — an unset value is treated as
                               a configuration error (see below), so a stale
                               address is never silently used.
    SENSOR_AGENT_PC_PORT      TCP port of the dashboard's sensor listener. Default: 50001.
    SENSOR_AGENT_INTERVAL_S   Delay between samples, in seconds. Default: 0.1.
    SENSOR_AGENT_TOKEN        Optional shared secret. When set, the agent sends
                               ``{"type": "auth", "token": ...}`` as the first
                               line of every connection; it must equal
                               SENSOR_DASHBOARD_PI_TOKEN on the PC.

Command-line arguments ``--host``, ``--port`` and ``--interval`` override the
corresponding environment variables. If neither an env var nor ``--host`` is
given, the agent prints an error and exits with status 1 instead of guessing an
address. It also exits with status 1 when the I2C bus cannot be opened or no
sensor answers at all, so systemd restarts it and the failure is visible.

Usage:

    SENSOR_AGENT_PC_HOST=192.0.2.10 python3 pi_sensor_agent.py
    python3 pi_sensor_agent.py --host 192.0.2.10 --port 50001

Runs as a systemd service in normal operation; see ``sensor-agent.service`` and
``sensor-agent.env.example`` in this directory.
"""
import argparse
import json
import logging
import math
import os
import socket
import struct
import sys
import time

logger = logging.getLogger("pi_sensor_agent")

try:
    import smbus
except ImportError:
    try:
        import smbus2 as smbus
    except ImportError:
        # Allowed on the control PC, where the tests import the pure functions;
        # the agent itself refuses to start without it (see start_agent()).
        smbus = None


def compensate_temperature(adc_t, dig_T1, dig_T2, dig_T3):
    """Bosch BME280 integer temperature compensation (datasheet 4.2.3).

    Returns ``(t_fine, temp)`` where ``temp`` is in 0.01 degC (5123 = 51.23 degC)
    and ``t_fine`` is the fine-resolution value the humidity formula needs.
    Pure integer math, no I/O, so it can be unit-tested without ``smbus``.
    """
    v1 = ((((adc_t >> 3) - (dig_T1 << 1))) * (dig_T2)) >> 11
    v2 = (((((adc_t >> 4) - (dig_T1)) * ((adc_t >> 4) - (dig_T1))) >> 12) * (dig_T3)) >> 14
    t_fine = v1 + v2
    temp = (t_fine * 5 + 128) >> 8
    return t_fine, temp


def compensate_humidity(adc_h, t_fine, dig_H1, dig_H2, dig_H3, dig_H4, dig_H5, dig_H6):
    """Bosch BME280 integer humidity compensation (datasheet 4.2.3), in %RH.

    The reference ``bme280_compensate_H_int32`` returns ``v_x1_u32r >> 12``, an
    unsigned Q22.10 value (47445 = 47445/1024 = 46.333 %RH), so %RH is that
    value divided by 1024. The clamp 419430400 = 100 * 2**22 is 100 %RH before
    the final shift. Before 2026-09-23 this code shifted by 22 (already whole
    %RH) and then also divided by 1024, recording 0-0.098 instead of 0-100.
    """
    vh = t_fine - 76800
    vh = (((((adc_h << 14) - (dig_H4 << 20) - (dig_H5 * vh)) + 16384) >> 15) *
          (((((((vh * dig_H6) >> 10) * (((vh * dig_H3) >> 11) + 32768)) >> 10) + 2097152) * dig_H2 + 8192) >> 14))
    vh = vh - (((((vh >> 15) * (vh >> 15)) >> 7) * dig_H1) >> 4)
    vh = 0 if vh < 0 else vh
    vh = 419430400 if vh > 419430400 else vh
    return (vh >> 12) / 1024.0


# --- SGP40 helpers (pure, testable without a bus) ---------------------------

SGP40_DEFAULT_RH = 50.0     # %RH, Sensirion's default compensation value
SGP40_DEFAULT_T = 25.0      # degC


def _finite(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def sgp40_crc8(data):
    """Sensirion CRC-8: polynomial 0x31, init 0xFF, no reflection, no final XOR.

    ``sgp40_crc8([0xBE, 0xEF]) == 0x92`` (the datasheet's example).
    """
    crc = 0xFF
    for byte in data:
        crc ^= byte & 0xFF
        for _ in range(8):
            crc = ((crc << 1) ^ 0x31) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def _word_with_crc(word):
    hi, lo = (word >> 8) & 0xFF, word & 0xFF
    return [hi, lo, sgp40_crc8([hi, lo])]


def sgp40_measure_args(humidity=None, temperature=None):
    """Argument bytes that follow the 0x26 byte of the 0x260F "measure raw" command.

    Compensation words per the SGP40 datasheet: ``RH_ticks = RH * 65535 / 100``
    and ``T_ticks = (T + 45) * 65535 / 175``, each followed by its CRC. A value
    of ``None`` (sensor unavailable) uses the defaults 50 %RH / 25 degC, which
    encode to 0x8000/0xA2 and 0x6666/0x93. Inputs are clamped to the ranges the
    sensor accepts (0-100 %RH, -45-130 degC).
    Returns the 7-byte list passed to ``write_i2c_block_data(addr, 0x26, ...)``.
    """
    rh = SGP40_DEFAULT_RH if not _finite(humidity) else min(100.0, max(0.0, float(humidity)))
    t = SGP40_DEFAULT_T if not _finite(temperature) else min(130.0, max(-45.0, float(temperature)))
    rh_ticks = int(round(rh * 65535 / 100.0))
    t_ticks = int(round((t + 45.0) * 65535 / 175.0))
    return [0x0F] + _word_with_crc(rh_ticks) + _word_with_crc(t_ticks)


def sgp40_parse_reply(reply):
    """Return the raw signal from a 3-byte SGP40 reply, or None if the CRC fails."""
    if reply is None or len(reply) < 3:
        return None
    if sgp40_crc8(reply[0:2]) != (reply[2] & 0xFF):
        return None
    return ((reply[0] & 0xFF) << 8) | (reply[1] & 0xFF)


# --- Sensor base: initialisation retry with backoff --------------------------

class _Sensor:
    """Common init/retry logic.

    Subclasses implement ``_init_hw()`` (raising on failure). ``ensure_ready()``
    re-runs it after a failure, but no more often than the current backoff,
    which starts at ``BACKOFF_MIN`` s and doubles up to ``BACKOFF_MAX`` s.
    """

    NAME = "sensor"
    BACKOFF_MIN = 1.0
    BACKOFF_MAX = 60.0

    def __init__(self, bus, address, clock=time.monotonic):
        self.bus = bus
        self.address = address
        self._clock = clock
        self.ready = False
        self._backoff = self.BACKOFF_MIN
        self._next_try = 0.0
        self._failures = 0
        self.ensure_ready()

    def _init_hw(self):  # pragma: no cover - overridden
        raise NotImplementedError

    def ensure_ready(self):
        """Initialise if needed (respecting the backoff). Returns whether the sensor is usable."""
        if self.ready:
            return True
        now = self._clock()
        if now < self._next_try:
            return False
        try:
            self._init_hw()
        except Exception as e:
            self._failures += 1
            logger.warning("%s at 0x%02X: initialisation failed (%s); retrying in %.0f s",
                           self.NAME, self.address, e, self._backoff)
            self._next_try = now + self._backoff
            self._backoff = min(self._backoff * 2, self.BACKOFF_MAX)
            return False
        if self._failures:
            logger.info("%s at 0x%02X: initialised after %d failed attempt(s)",
                        self.NAME, self.address, self._failures)
        self.ready = True
        self._failures = 0
        self._backoff = self.BACKOFF_MIN
        return True

    def _read_failed(self, e):
        logger.debug("%s at 0x%02X: read failed: %s", self.NAME, self.address, e)


class BME280(_Sensor):
    """Bosch BME280 temperature/humidity sensor."""

    NAME = "BME280"

    def __init__(self, bus, address=0x76, **kw):
        self.calib = []
        self.chip_id = 0
        self.model = "Unknown"
        super().__init__(bus, address, **kw)

    def _init_hw(self):
        # 1. Reset the sensor
        self.bus.write_byte_data(self.address, 0xE0, 0xB6)
        time.sleep(0.1)

        # 2. Check the chip ID
        self.chip_id = self.bus.read_byte_data(self.address, 0xD0)
        if self.chip_id == 0x60:
            self.model = "BME280 (T/H)"
        elif self.chip_id == 0x58:
            self.model = "BMP280 (T only)"
        else:
            raise RuntimeError(f"unexpected chip id 0x{self.chip_id:02X}")

        # 3. Read the calibration data
        c1 = self.bus.read_i2c_block_data(self.address, 0x88, 24)
        c2 = [self.bus.read_byte_data(self.address, 0xA1)]
        c3 = self.bus.read_i2c_block_data(self.address, 0xE1, 7)
        self.calib = c1 + c2 + c3

    def load_calibration(self):
        """Kept for compatibility; initialisation goes through ensure_ready()."""
        return self.ensure_ready()

    def read_data(self):
        """Return ``(temperature_degC, humidity_percent_rh)``; None for what was not read."""
        if not self.ensure_ready():
            return None, None
        try:
            # Write order matters for enabling humidity: 0xF2 must be written before 0xF4
            self.bus.write_byte_data(self.address, 0xF2, 0x01) # Hum x1
            self.bus.write_byte_data(self.address, 0xF4, 0x25) # Temp x1, Press x1, Forced mode

            # Wait for the measurement to finish (up to 50ms)
            for _ in range(10):
                status = self.bus.read_byte_data(self.address, 0xF3)
                if not (status & 0x08): break
                time.sleep(0.01)

            data = self.bus.read_i2c_block_data(self.address, 0xF7, 8)

            # --- Temperature ---
            adc_t = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
            dig_T1 = (self.calib[1] << 8) | self.calib[0]
            dig_T2 = struct.unpack("<h", bytes(self.calib[2:4]))[0]
            dig_T3 = struct.unpack("<h", bytes(self.calib[4:6]))[0]

            t_fine, temp = compensate_temperature(adc_t, dig_T1, dig_T2, dig_T3)

            # --- Humidity ---
            humi = None
            if self.chip_id == 0x60:
                adc_h = (data[6] << 8) | data[7]
                if adc_h != 0x8000: # 0x8000 means "not measured"
                    dig_H1 = self.calib[24]
                    dig_H2 = struct.unpack("<h", bytes(self.calib[25:27]))[0]
                    dig_H3 = self.calib[27]
                    dig_H4 = (self.calib[28] << 4) | (self.calib[29] & 0x0F)
                    dig_H5 = (self.calib[30] << 4) | (self.calib[29] >> 4)
                    dig_H6 = struct.unpack("<b", bytes([self.calib[31]]))[0]

                    humi = compensate_humidity(
                        adc_h, t_fine, dig_H1, dig_H2, dig_H3, dig_H4, dig_H5, dig_H6
                    )

            return temp / 100.0, humi
        except Exception as e:
            self._read_failed(e)
            return None, None


class TSL25911(_Sensor):
    """AMS TSL25911 ambient light sensor.

    Reports the raw channel-0 (visible + IR) count, NOT lux. The agent leaves
    the CONTROL register at its power-on value (gain 1x, integration 100 ms).
    Converting to lux needs channel 1 as well and the gain/integration-time
    scaling; see the TODO in the README.
    """

    NAME = "TSL25911"

    def __init__(self, bus, address=0x29, **kw):
        super().__init__(bus, address, **kw)

    def _init_hw(self):
        # COMMAND bit | ENABLE register (0xA0) = PON | AEN
        self.bus.write_byte_data(self.address, 0xA0, 0x03)

    def read_raw_ch0(self):
        """Return the channel-0 count, or None on failure."""
        if not self.ensure_ready():
            return None
        try:
            l, h = self.bus.read_byte_data(self.address, 0xB4), self.bus.read_byte_data(self.address, 0xB5)
            return (h << 8) | l
        except Exception as e:
            self._read_failed(e)
            return None

    # Old name, kept for scripts that call it; the value is NOT lux.
    read_lux = read_raw_ch0


class ICM20948(_Sensor):
    """TDK InvenSense ICM20948 IMU. Only accelerometer + gyroscope are read here
    (the embedded AK09916 magnetometer is NOT read by this agent)."""

    NAME = "ICM20948"

    def __init__(self, bus, address=0x68, **kw):
        super().__init__(bus, address, **kw)

    def _init_hw(self):
        self.bus.write_byte_data(self.address, 0x06, 0x01)

    def read_accel_gyro(self):
        """Return ``(accel_g, gyro_dps)`` 3-tuples, or ``(None, None)`` on failure."""
        if not self.ensure_ready():
            return None, None
        try:
            d = self.bus.read_i2c_block_data(self.address, 0x2D, 12)
            ax, ay, az = struct.unpack(">hhh", bytes(d[0:6]))
            gx, gy, gz = struct.unpack(">hhh", bytes(d[6:12]))
            return (ax/16384.0, ay/16384.0, az/16384.0), (gx/131.0, gy/131.0, gz/131.0)
        except Exception as e:
            self._read_failed(e)
            return None, None


class LTR390(_Sensor):
    """Lite-On LTR390 UV sensor, configured in UVS mode and reporting a UV index.

    Register setup (see the LTR390-UV-01 datasheet):

        MAIN_CTRL (0x00) = 0x0A   sensor enabled + UVS mode
                                  (0x02 would be enabled + ALS mode, in which
                                  the UVS data registers stay at 0)
        MEAS_RATE (0x04) = 0x05   20-bit resolution (400 ms conversion) with a
                                  1000 ms measurement rate
        GAIN      (0x05) = 0x01   gain 3

    UVS data is a 20-bit little-endian value in 0x10..0x12. The datasheet's
    sensitivity is 2300 counts per UVI at gain 18 / 20-bit, so at gain 3 the
    divisor is 2300 * 3/18.

    The conversion time (400 ms at 20-bit, with a 1000 ms measurement rate) is
    much longer than this agent's 100 ms sampling loop, so consecutive reads
    return the same conversion result; that is the sensor's real update rate,
    not a stall. A failed read returns None, never 0 (which would look like
    darkness).
    """

    NAME = "LTR390"
    _SENSITIVITY = 2300.0 * (3.0 / 18.0)   # counts per UVI at gain 3, 20-bit

    def __init__(self, bus, address=0x53, **kw):
        super().__init__(bus, address, **kw)

    def _init_hw(self):
        self.bus.write_byte_data(self.address, 0x00, 0x0A)  # enable + UVS mode
        self.bus.write_byte_data(self.address, 0x04, 0x05)  # 20-bit, 1000 ms rate
        self.bus.write_byte_data(self.address, 0x05, 0x01)  # gain 3
        time.sleep(0.5)                                     # first conversion

    def read_uv(self):
        """Return the UV index as a float (0.0 in darkness), or None on failure."""
        if not self.ensure_ready():
            return None
        try:
            d = self.bus.read_i2c_block_data(self.address, 0x10, 3)
            raw = d[0] | (d[1] << 8) | (d[2] << 16)
            return round(raw / self._SENSITIVITY, 3)
        except Exception as e:
            self._read_failed(e)
            return None


class SGP40(_Sensor):
    """Sensirion SGP40 VOC sensor. Reports the raw signal (not the VOC index)."""

    NAME = "SGP40"

    def __init__(self, bus, address=0x59, **kw):
        super().__init__(bus, address, **kw)

    def _init_hw(self):
        # No configuration needed; the first measurement doubles as the probe.
        pass

    def read_voc(self, humidity=None, temperature=None):
        """Measure the raw signal, compensated with the given %RH / degC.

        Returns None if the I2C transfer fails or the reply's CRC does not
        match (a corrupted word is dropped, not reported). The raw signal is
        a 16-bit word, so no range check beyond the CRC is needed.
        """
        if not self.ensure_ready():
            return None
        try:
            # "Measure Raw Signal" command 0x260F + RH/T compensation words.
            self.bus.write_i2c_block_data(self.address, 0x26, sgp40_measure_args(humidity, temperature))
            time.sleep(0.03) # measurement delay

            # Read 3 bytes (Data H, Data L, CRC)
            d = self.bus.read_i2c_block_data(self.address, 0x59, 3)
        except Exception as e:
            self._read_failed(e)
            return None
        value = sgp40_parse_reply(d)
        if value is None:
            logger.debug("SGP40: CRC mismatch in reply %r; sample dropped", d)
        return value


def build_message(hostname, model, temp, humi, acc, gyr, lux_raw, uvi, voc):
    """Assemble one JSON-serialisable sample. ``None`` values stay ``null``.

    ``ok`` tells the dashboard which sensors produced a value this sample, so a
    missing reading is never mistaken for a real 0.
    """
    return {
        "type": "sensor_data",
        "temp": temp, "humi": humi,
        "accel": list(acc) if acc is not None else None,
        "gyro": list(gyr) if gyr is not None else None,
        "lux_raw": lux_raw, "uv": uvi, "voc": voc,
        "ok": {
            "bme280": temp is not None,
            "tsl25911": lux_raw is not None,
            "icm20948": acc is not None,
            "ltr390": uvi is not None,
            "sgp40": voc is not None,
        },
        "model": model,
        "hostname": hostname,
    }


def auth_line(token):
    """First line sent on each connection when a shared secret is configured."""
    return (json.dumps({"type": "auth", "token": token}) + "\n").encode()


def parse_args():
    parser = argparse.ArgumentParser(description="Push I2C environment-sensor readings to the dashboard PC over TCP.")
    parser.add_argument("--host", default=None, help="Dashboard PC hostname or IP (overrides SENSOR_AGENT_PC_HOST)")
    parser.add_argument("--port", type=int, default=None, help="Dashboard TCP port (overrides SENSOR_AGENT_PC_PORT, default 50001)")
    parser.add_argument("--interval", type=float, default=None, help="Seconds between samples (overrides SENSOR_AGENT_INTERVAL_S, default 0.1)")
    return parser.parse_args()


def resolve_config(args):
    host = args.host or os.environ.get("SENSOR_AGENT_PC_HOST")
    if not host:
        print("[!] Error: no dashboard PC host configured. Set SENSOR_AGENT_PC_HOST or pass --host.")
        sys.exit(1)
    port = args.port if args.port is not None else int(os.environ.get("SENSOR_AGENT_PC_PORT", "50001"))
    interval = args.interval if args.interval is not None else float(os.environ.get("SENSOR_AGENT_INTERVAL_S", "0.1"))
    return host, port, interval


def start_agent(host, port, interval, token=None):
    """Run the sampling loop forever. Returns a non-zero exit status on a fatal error."""
    import platform
    my_name = platform.node()  # local hostname, sent with each sample
    if smbus is None:
        logger.error("smbus or smbus2 library not found. Install with: sudo apt install python3-smbus")
        return 1
    try:
        bus = smbus.SMBus(1)
    except Exception as e:
        logger.error("Could not open I2C bus 1 (%s). Is I2C enabled (raspi-config)?", e)
        return 1

    light = TSL25911(bus)
    bme = BME280(bus)
    imu = ICM20948(bus)
    uv = LTR390(bus)
    voc = SGP40(bus)
    # The SGP40 has no initialisation transaction to probe, so it is left out.
    if not any(s.ready for s in (light, bme, imu, uv)):
        # Nothing answered: the HAT is missing or on another bus. Exit non-zero
        # so systemd (Restart=always) retries and `systemctl status` shows it.
        logger.error("No sensor on the HAT answered during initialisation; exiting")
        return 1

    while True:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(5)
                s.connect((host, port))
                s.settimeout(None)
                logger.info("Connected to %s:%d", host, port)
                if token:
                    s.sendall(auth_line(token))
                while True:
                    t, h = bme.read_data()
                    acc, gyr = imu.read_accel_gyro()
                    lx = light.read_raw_ch0()
                    uvv = uv.read_uv()
                    v = voc.read_voc(humidity=h, temperature=t)

                    data = build_message(my_name, bme.model, t, h, acc, gyr, lx, uvv, v)
                    s.sendall((json.dumps(data) + "\n").encode())
                    time.sleep(interval)

        except Exception as e:
            logger.warning("Connection to %s:%d lost or failed (%s); retrying in 5 s", host, port, e)
            time.sleep(5)


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")
    _host, _port, _interval = resolve_config(parse_args())
    sys.exit(start_agent(_host, _port, _interval, os.environ.get("SENSOR_AGENT_TOKEN") or None))
