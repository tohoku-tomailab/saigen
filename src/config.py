"""Loader for the environment-dependent settings of the control core.

``config.yaml`` in the repository root is the single source of truth for the
values that change between installations: COM ports, camera indices and the
Dobot workspace limits.

Resolution order (first file that exists wins, merged over ``DEFAULTS``):

    1. ``<repo>/config.yaml``          - your own copy, gitignored
    2. ``<repo>/config.example.yaml``  - tracked template, same values as DEFAULTS
    3. ``DEFAULTS`` below              - used only when neither file exists

A file that exists but cannot be used (PyYAML missing, unreadable, invalid YAML,
top level not a mapping) raises :class:`ConfigError`. It never falls back to
``DEFAULTS``: those carry the spec-envelope workspace (+-300 mm) and default COM
ports, so a silent fallback would widen the workspace limits the lab measured.

Missing keys are filled in from ``DEFAULTS``, so a partial config.yaml that only
overrides a couple of ports keeps working after an upgrade.

Sensor-dashboard ports and the video-recording camera index are NOT duplicated
here: they belong to src/monitoring and are read from
``src/monitoring/config.yaml`` (falling back to ``config.example.yaml`` there),
exactly as launch_sensor_dashboard.py does.
"""
import copy
import logging
import os

logger = logging.getLogger(__name__)

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG_PATH = os.path.join(REPO_ROOT, "config.yaml")
EXAMPLE_CONFIG_PATH = os.path.join(REPO_ROOT, "config.example.yaml")

MONITORING_DIR = os.path.join(REPO_ROOT, "src", "monitoring")

# Fallback used when no YAML file can be read. Keep in sync with
# config.example.yaml (tests/test_config.py checks for drift).
DEFAULTS = {
    "robots": {
        1: {"dobot_port": "COM3", "picus2_address": "COM4"},
        2: {"dobot_port": "COM5", "picus2_address": "COM6"},
        3: {"dobot_port": "COM7"},
        # Hiwonder MaxArm (suction pick-and-place). Placeholder port: a flow that
        # uses robot 4 refuses to start a real run until maxarm_port is set.
        # The workspace is a conservative starting box inside the arm's reach,
        # in the MaxArm's own frame (mm, origin on the base yaw axis); measure
        # your cell and tighten it before any real run.
        4: {
            "type": "maxarm",
            "maxarm_port": "",
            "workspace": {
                "x_min": -250.0, "x_max": 250.0,
                "y_min": -280.0, "y_max": -60.0,
                "z_min": 40.0, "z_max": 225.0,
                "r_min": 60.0, "r_max": 260.0,
            },
            "move_ms": 1500,
        },
    },
    "shared_devices": {
        "scale_port": "COM8",
        "camera_index": 0,
        "microscope_index": 2,
        "microscope_port": "",
        "microscope_resolution": "3840x2160",
        # Keithley 2002 DMM: VISA resource, e.g. GPIB0::16::INSTR. Empty: a flow
        # with measure_dmm refuses to start a real run until it is set.
        "dmm_resource": "",
        "dmm_timeout_ms": 10000,
        "wait_after_dmm": 0.0,
        # NF LI5640 lock-in amplifier (read-only): VISA resource, e.g.
        # GPIB0::<address>::INSTR. Empty: a flow with measure_lockin refuses to
        # start a real run until it is set.
        "lockin_resource": "",
        "lockin_timeout_ms": 5000,
        "wait_after_lockin": 0.0,
        # LI5640 internal oscillator (OSC OUT), written only by set_lockin_oscillator.
        # 0 = writing disabled. Set the largest amplitude (Vrms) the load on OSC OUT
        # tolerates for an unlimited time; the frequency range likewise.
        "lockin_osc_max_vrms": 0.0,
        "lockin_osc_min_hz": 0.0005,
        "lockin_osc_max_hz": 105000.0,
        # Agilent 1000 series oscilloscope (DSO1014A etc., read-only): VISA
        # resource USB0::0x0957::<product id>::<serial>::INSTR. Empty: a flow
        # with measure_scope refuses to start a real run until it is set.
        "scope_resource": "",
        "scope_timeout_ms": 5000,
        "wait_after_scope": 0.0,
    },
    "workspace": {
        "x_min": -300.0, "x_max": 300.0,
        "y_min": -300.0, "y_max": 300.0,
        "z_min": -130.0, "z_max": 150.0,
        "joint1_min": -135.0, "joint1_max": 135.0,
    },
}

# Same defaults launch_sensor_dashboard.py falls back to.
MONITORING_DEFAULTS = {
    "sensor_dashboard": {"web_port": 8000, "tcp_port": 50001},
    "video": {"camera_index": 1},
}

_config_cache = None
_monitoring_cache = None


def _deep_merge(base: dict, override: dict) -> dict:
    """Return a new dict: ``base`` recursively overridden by ``override``."""
    merged = copy.deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = _deep_merge(merged[key], value)
        else:
            merged[key] = value
    return merged


class ConfigError(RuntimeError):
    """An existing config file cannot be used.

    Deliberately not an ImportError/OSError subclass, so callers that catch
    those (e.g. ``default_workspace_validator``) cannot mistake it for
    "config module unavailable" and fall back to wider built-in limits.
    """


def _load_yaml_file(path: str) -> dict:
    """Read one YAML mapping; raise ConfigError if the file is unusable."""
    try:
        import yaml
    except ImportError as e:
        raise ConfigError(
            f"PyYAML is not installed, so {path} cannot be read "
            f"(run: pip install -r requirements.txt). Refusing to fall back to "
            f"built-in defaults."
        ) from e
    try:
        with open(path, encoding="utf-8") as f:
            loaded = yaml.safe_load(f)
    except (OSError, UnicodeDecodeError, yaml.YAMLError) as e:
        raise ConfigError(f"Could not read {path}: {e}") from e
    if loaded is None:
        loaded = {}
    if not isinstance(loaded, dict):
        raise ConfigError(
            f"{path}: the top level must be a mapping, got {type(loaded).__name__}"
        )
    return loaded


def load_config(force_reload: bool = False) -> dict:
    """Return config.yaml (or config.example.yaml) merged over DEFAULTS."""
    global _config_cache
    if _config_cache is not None and not force_reload:
        return _config_cache

    config = copy.deepcopy(DEFAULTS)
    for path in (CONFIG_PATH, EXAMPLE_CONFIG_PATH):
        if not os.path.exists(path):
            continue
        config = _deep_merge(config, _load_yaml_file(path))
        break
    else:
        logger.warning(
            "Neither %s nor %s found; using built-in defaults",
            CONFIG_PATH, EXAMPLE_CONFIG_PATH,
        )

    _config_cache = _detach_retyped_robots(config, _file_robots())
    return _config_cache


def _file_robots() -> dict:
    """The ``robots`` mapping as the file gave it (read again, unmerged); {} without a file"""
    for path in (CONFIG_PATH, EXAMPLE_CONFIG_PATH):
        if os.path.exists(path):
            return dict(_load_yaml_file(path).get("robots") or {})
    return {}


def _detach_retyped_robots(merged: dict, file_robots: dict) -> dict:
    """A robot whose ``type`` in the file differs from the DEFAULTS entry of the same id
    inherits nothing from that entry: its merged entry is replaced by the raw one (2026-10-30,
    flow plan F8). Otherwise a ``type: mg400`` written at id 4 would carry the MaxArm's
    workspace, and a Dobot at id 4 the MaxArm's box."""
    defaults = DEFAULTS.get("robots") or {}
    for rid, raw in file_robots.items():
        raw = dict(raw or {})
        default_type = (defaults.get(rid) or {}).get("type", DEFAULT_ROBOT_TYPE)
        # an entry without a type keeps inheriting (robot 4 without "type" is still the MaxArm)
        if "type" in raw and raw["type"] != default_type and rid in merged.get("robots", {}):
            merged["robots"][rid] = copy.deepcopy(raw)
    return merged


def load_monitoring_config(force_reload: bool = False) -> dict:
    """Return src/monitoring/config.yaml merged over MONITORING_DEFAULTS."""
    global _monitoring_cache
    if _monitoring_cache is not None and not force_reload:
        return _monitoring_cache

    config = copy.deepcopy(MONITORING_DEFAULTS)
    for name in ("config.yaml", "config.example.yaml"):
        path = os.path.join(MONITORING_DIR, name)
        if not os.path.exists(path):
            continue
        config = _deep_merge(config, _load_yaml_file(path))
        break

    _monitoring_cache = config
    return config


#: Robot types a ``robots.<id>.type`` entry may name. Robots without a type are
#: Dobot Magicians (the only type before the MaxArm was added), so existing
#: config.yaml files keep working unchanged.
ROBOT_TYPES = ("dobot", "maxarm", "mg400")
DEFAULT_ROBOT_TYPE = "dobot"


def get_robot_ports() -> dict:
    """robot_id (int) -> connection settings for that robot.

    Every entry carries ``type`` (``"dobot"`` when the file does not say);
    MaxArm entries also carry ``maxarm_port``, an optional ``workspace``
    override and driver options such as ``move_ms``.
    """
    robots = {}
    for rid, ports in (load_config().get("robots") or {}).items():
        entry = copy.deepcopy(dict(ports or {}))
        entry.setdefault("type", DEFAULT_ROBOT_TYPE)
        robots[int(rid)] = entry
    return robots


def get_robot_type(robot_id: int) -> str:
    """``"dobot"``, ``"maxarm"`` or ``"mg400"`` for ``robot_id`` (``"dobot"`` if not configured).

    Raises:
        ConfigError: the configured type is not one of :data:`ROBOT_TYPES`
    """
    rtype = get_robot_ports().get(int(robot_id), {}).get("type", DEFAULT_ROBOT_TYPE)
    if rtype not in ROBOT_TYPES:
        raise ConfigError(
            f"robots.{robot_id}.type = {rtype!r} is not a known robot type {ROBOT_TYPES}"
        )
    return rtype


#: Keys a MaxArm's ``robots.<id>.workspace`` must set. Its frame differs from
#: the Dobot's, so none of these may be inherited from the global workspace.
MAXARM_REQUIRED_WORKSPACE_KEYS = ("x_min", "x_max", "y_min", "y_max", "z_min", "z_max")

#: Keys a MaxArm workspace may set in addition (the base-protecting radius).
MAXARM_OPTIONAL_WORKSPACE_KEYS = ("r_min", "r_max")


def own_workspace(robot_id, entry, label: str, type_name: str) -> dict:
    """The self-contained workspace block of one ``robots.<id>`` entry, checked.

    A robot in its own frame (the MaxArm, the MG400) takes its limits only from its
    own ``workspace`` block, never merged over the global (Dobot) box. The block must
    set every key of :data:`MAXARM_REQUIRED_WORKSPACE_KEYS`; ``r_min`` / ``r_max``
    are optional. A key present with no value (YAML ``z_min:``) is an error, never
    "unlimited".

    Raises:
        ConfigError: the block is missing, not a mapping, incomplete or has a null value
    """
    override = (entry or {}).get("workspace")
    where = f"robots.{robot_id}.workspace"
    if override is None or override == {}:
        raise ConfigError(
            f"Robot {robot_id} is a {label} (type: {type_name}) but {where} is not set. "
            f"A {label} needs its own complete workspace "
            f"({', '.join(MAXARM_REQUIRED_WORKSPACE_KEYS)}; r_min / r_max optional) in its "
            f"own frame; it never uses the global Dobot limits."
        )
    if not isinstance(override, dict):
        raise ConfigError(f"{where} must be a mapping, got {type(override).__name__}")
    missing = [k for k in MAXARM_REQUIRED_WORKSPACE_KEYS if k not in override]
    if missing:
        raise ConfigError(
            f"{where} ({label}, robot {robot_id}) is missing {', '.join(missing)}. "
            f"All of {', '.join(MAXARM_REQUIRED_WORKSPACE_KEYS)} are required."
        )
    nulls = [k for k, v in override.items() if v is None]
    if nulls:
        raise ConfigError(
            f"{where} ({label}, robot {robot_id}): {', '.join(nulls)} has no value (null). "
            f"Give a number or delete the optional key; a null is never read as unlimited."
        )
    return dict(override)


def maxarm_workspace(robot_id, entry) -> dict:
    """The MaxArm workspace block of one ``robots.<id>`` entry, checked (:func:`own_workspace`)"""
    return own_workspace(robot_id, entry, "MaxArm", "maxarm")


#: Keys a ``type: mg400`` entry must set itself (checked in the entry as the file wrote it:
#: nothing is inherited from DEFAULTS, which holds no MG400). ``mg400_address`` may be
#: empty in a tracked file; a real run refuses until it is set. The values themselves are
#: checked by ``src.devices.safety.mg400_robot`` (speed_ratio 1..10, max_move_mm 1..50,
#: home_position [x, y, z], joint_limits four [min, max] inside the published ranges).
MG400_REQUIRED_KEYS = ("mg400_address", "workspace", "home_position", "joint_limits",
                       "speed_ratio", "max_move_mm")


def mg400_config(robot_id, entry) -> dict:
    """One ``type: mg400`` entry, checked for its required keys and its own workspace.

    Raises:
        ConfigError: a required key is missing or null, or the workspace block is unusable
    """
    entry = dict(entry or {})
    where = f"robots.{robot_id}"
    missing = [k for k in MG400_REQUIRED_KEYS if k not in entry]
    if missing:
        raise ConfigError(
            f"{where} is an MG400 (type: mg400) but is missing {', '.join(missing)}. "
            f"All of {', '.join(MG400_REQUIRED_KEYS)} must be written in that entry "
            f"(nothing is inherited; see examples/mg400/config.mg400.yaml)."
        )
    nulls = [k for k in MG400_REQUIRED_KEYS if entry[k] is None and k != "mg400_address"]
    if nulls:
        raise ConfigError(f"{where} (MG400): {', '.join(nulls)} has no value (null).")
    if entry["mg400_address"] is None:
        entry["mg400_address"] = ""
    if not isinstance(entry["mg400_address"], str):
        raise ConfigError(f"{where}.mg400_address must be a string (the arm's IP address), "
                          f"got {type(entry['mg400_address']).__name__}")
    entry["workspace"] = own_workspace(robot_id, entry, "MG400", "mg400")
    # the numeric options with the wrapper's own validators, so that --validate-only and the
    # preflight refuse them before any device is opened (not only the robot's constructor)
    from src.devices.safety.mg400_robot import (parse_home_position, parse_joint_limits,  # noqa: E402 - lazy
                                                validate_max_move_mm, validate_speed_ratio)
    try:
        parse_home_position(entry["home_position"])
        parse_joint_limits(entry["joint_limits"])
        validate_speed_ratio(entry["speed_ratio"])
        validate_max_move_mm(entry["max_move_mm"])
    except ValueError as e:
        raise ConfigError(f"{where} (MG400): {e}") from e
    return entry


def get_robot_workspace(robot_id: int) -> dict:
    """Workspace limits for one robot.

    - MaxArm (``type: maxarm``): only its own ``robots.<id>.workspace`` block,
      checked by :func:`maxarm_workspace` (never merged over the global box).
    - MG400 (``type: mg400``): likewise its own block, through :func:`mg400_config`.
    - Dobot: the global ``workspace`` merged with ``robots.<id>.workspace``
      (the robot's own keys win); without its own section, the global limits.

    Raises:
        ConfigError: the robot's workspace is not a mapping, or a MaxArm's is
            missing, incomplete or has a null value
    """
    entry = get_robot_ports().get(int(robot_id), {})
    if entry.get("type") == "maxarm":
        return maxarm_workspace(robot_id, entry)
    if entry.get("type") == "mg400":
        return mg400_config(robot_id, entry)["workspace"]
    merged = dict(load_config()["workspace"])
    override = entry.get("workspace") or {}
    if not isinstance(override, dict):
        raise ConfigError(
            f"robots.{robot_id}.workspace must be a mapping, got {type(override).__name__}"
        )
    merged.update(override)
    return _reject_null_limits(merged, f"robots.{robot_id}.workspace (merged over workspace)")


def get_shared_devices() -> dict:
    """Settings for the shared devices (scale_port, camera_index, microscope_*, dmm_*, lockin_*, scope_*)."""
    return dict(load_config()["shared_devices"])


def get_workspace() -> dict:
    """Global (Dobot) workspace limits (keys match the WorkspaceValidator arguments).

    Robots with their own ``robots.<id>.workspace`` use :func:`get_robot_workspace`.

    Raises:
        ConfigError: the section is not a mapping or a key has no value (YAML
            ``r_min:`` / ``r_min: null``). A null is never read as "unlimited"
            (the same rule as :func:`maxarm_workspace`).
    """
    return _reject_null_limits(load_config()["workspace"], "workspace")


def _reject_null_limits(ws, where: str) -> dict:
    """Return ``ws`` as a dict; ConfigError if it is not a mapping or has a null value."""
    if not isinstance(ws, dict):
        raise ConfigError(f"{where} must be a mapping, got {type(ws).__name__}")
    nulls = [k for k, v in ws.items() if v is None]
    if nulls:
        raise ConfigError(
            f"{where}: {', '.join(map(str, nulls))} has no value (null). "
            f"Give a number or delete the key; a null is never read as unlimited."
        )
    return dict(ws)


def get_video_camera_index() -> int:
    """OpenCV index of the camera used for video recording (src/monitoring)."""
    return int(load_monitoring_config()["video"]["camera_index"])


def get_dashboard_ports() -> dict:
    """Sensor-dashboard ports (web_port, tcp_port) from src/monitoring."""
    return {k: int(v) for k, v in load_monitoring_config()["sensor_dashboard"].items()}
