"""USB digital microscope (UVC): driver defaults, SharedDevices wiring, mock,
schema/executor routing and a mock run end to end. No hardware needed."""
import asyncio
import json
import os

import numpy as np
import pytest

from src import config as lab_config
from src.devices.microscope import MicroscopeController
from src.devices.safety.mock_robot import MockSharedDevices
from src.devices.safety.shared_devices import SharedDevices
from src.flow import run_flow
from src.flow.executor import SHARED_DEVICE_ACTIONS, execute_shared_device_step, execute_step
from src.flow.experiment_logger import ExperimentLogger
from src.flow.schema import ExperimentWorkflow

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EXAMPLE_FLOW = os.path.join(REPO_ROOT, "examples", "microscope", "product_closeup.json")


# ----------------------------------------------------------------------
# Driver
# ----------------------------------------------------------------------
class FakeCapture:
    """cv2.VideoCapture の代わり: 読み出し回数を数え、指定フレームを返す。"""

    def __init__(self, frame):
        self.frame = frame
        self.reads = 0
        self.props = {}
        self.released = False

    def read(self):
        self.reads += 1
        return True, self.frame

    def set(self, prop, value):
        self.props[prop] = value
        return True

    def get(self, prop):
        return self.props.get(prop, 0)

    def release(self):
        self.released = True


def _driver(frame, **kwargs) -> MicroscopeController:
    scope = object.__new__(MicroscopeController)
    scope.camera_index = 2
    scope.resolution = (1280, 720)
    scope.camera = FakeCapture(frame)
    scope.is_connected = True
    scope.save_directory = "microscope_images"
    scope.warmup_frames = kwargs.get("warmup_frames", MicroscopeController.DEFAULT_WARMUP_FRAMES)
    scope.exposure = kwargs.get("exposure")
    scope.white_balance = kwargs.get("white_balance")
    scope.backend_name = None
    return scope


def _textured_frame():
    rng = np.random.default_rng(0)
    return rng.integers(0, 255, size=(8, 8, 3), dtype=np.uint8)


def test_driver_defaults_differ_from_webcam(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    scope = MicroscopeController()
    assert scope.save_directory == "microscope_images"
    assert (tmp_path / "microscope_images").is_dir()
    assert scope.warmup_frames == MicroscopeController.DEFAULT_WARMUP_FRAMES
    assert scope.exposure is None
    assert scope.camera_index == 2          # placeholder; real value comes from config.yaml
    assert scope.resolution == (3840, 2160)  # 4K by default (sharpest and fastest on the 400-CAM106)
    assert MicroscopeController(resolution=(1280, 720)).resolution == (1280, 720)


def test_capture_discards_warmup_frames_then_returns_one():
    scope = _driver(_textured_frame(), warmup_frames=7)
    frame = scope.capture_image()
    assert frame is not None
    assert scope.camera.reads == 8, "warmup_frames 枚読み捨てた後に 1 枚取得する"


def test_capture_warns_on_uniform_frame(caplog):
    flat = np.full((8, 8, 3), 120, dtype=np.uint8)
    scope = _driver(flat, warmup_frames=0)
    with caplog.at_level("WARNING"):
        frame = scope.capture_image()
    assert frame is not None, "単色でも画像は返す（記録は残す）"
    assert "単色" in caplog.text


def test_is_uniform():
    assert MicroscopeController.is_uniform(np.zeros((4, 4, 3), dtype=np.uint8))
    assert not MicroscopeController.is_uniform(_textured_frame())
    assert MicroscopeController.is_uniform(None)
    # 実機で観察した「緑一色」フレーム: B/G/R は違うが空間的には平坦 -> 単色
    flat_green = np.empty((6, 6, 3), dtype=np.uint8)
    flat_green[..., 0], flat_green[..., 1], flat_green[..., 2] = 95, 255, 123
    assert MicroscopeController.is_uniform(flat_green)
    # 緩やかな周辺減光だけのフレームも単色扱い、はっきりした模様は単色ではない
    gradient = np.tile(np.linspace(117, 123, 16, dtype=np.uint8)[None, :, None], (16, 1, 3))
    assert MicroscopeController.is_uniform(gradient)
    stripes = np.tile(np.array([0, 255] * 8, dtype=np.uint8)[None, :, None], (16, 1, 3))
    assert not MicroscopeController.is_uniform(stripes)


def test_capture_requires_connection():
    scope = _driver(_textured_frame())
    scope.is_connected = False
    assert scope.capture_image() is None


def test_manual_exposure_is_applied_on_connect(monkeypatch):
    import cv2
    from src.devices.microscope import microscope_controller as mc
    scope = _driver(_textured_frame(), exposure=-7)
    scope.is_connected = False
    monkeypatch.setattr(mc.platform, "system", lambda: "Linux")          # parent-class open path
    monkeypatch.setattr(MicroscopeController.__mro__[1], "connect", lambda self: True)
    assert scope.connect() is True
    assert scope.camera.props[cv2.CAP_PROP_EXPOSURE] == -7
    assert scope.camera.props[cv2.CAP_PROP_AUTO_EXPOSURE] == 0.25


# ----------------------------------------------------------------------
# SharedDevices / MockSharedDevices
# ----------------------------------------------------------------------
class FakeMicroscope:
    instances = []

    def __init__(self, camera_index, resolution=None):
        self.camera_index = camera_index
        self.connected = False
        self.saved = []
        FakeMicroscope.instances.append(self)

    def connect(self):
        self.connected = True
        return True

    def capture_and_save(self, file_path=None):
        path = file_path or "microscope_images/auto.jpg"
        self.saved.append(path)
        return path

    def disconnect(self):
        self.connected = False


def test_shared_devices_initialize_microscope_from_config(monkeypatch):
    import src.devices.microscope as pkg
    monkeypatch.setattr(pkg, "MicroscopeController", FakeMicroscope)
    FakeMicroscope.instances.clear()

    shared = SharedDevices(use_microscope=True, microscope_index=7)
    assert asyncio.run(shared.initialize()) is True
    assert FakeMicroscope.instances[0].camera_index == 7
    assert FakeMicroscope.instances[0].connected

    path = asyncio.run(shared.capture_microscope("x.jpg"))
    assert path == "x.jpg"
    assert asyncio.run(shared.capture_microscope(None)) == "microscope_images/auto.jpg"

    asyncio.run(shared.cleanup())
    assert not FakeMicroscope.instances[0].connected


def test_shared_devices_microscope_not_opened_unless_requested(monkeypatch):
    import src.devices.microscope as pkg
    monkeypatch.setattr(pkg, "MicroscopeController", FakeMicroscope)
    FakeMicroscope.instances.clear()

    shared = SharedDevices()          # no flags at all
    assert asyncio.run(shared.initialize()) is True
    assert FakeMicroscope.instances == []
    with pytest.raises(RuntimeError):
        asyncio.run(shared.capture_microscope())


def test_mock_shared_devices_capture_microscope_returns_path():
    mock = MockSharedDevices(use_microscope=True)
    p1 = asyncio.run(mock.capture_microscope())
    p2 = asyncio.run(mock.capture_microscope("given.jpg"))
    assert p1.startswith("microscope_images/mock_microscope_001")
    assert p2 == "given.jpg"


# ----------------------------------------------------------------------
# Schema / executor / planner
# ----------------------------------------------------------------------
def test_schema_accepts_capture_microscope_without_robot_id():
    wf = ExperimentWorkflow(name="m", description="d",
                            steps=[{"action": "capture_microscope"}])
    assert wf.steps[0].file_path == ""


def test_schema_rejects_misspelled_action():
    with pytest.raises(Exception):
        ExperimentWorkflow(name="m", description="d",
                           steps=[{"action": "capture_microscop"}])


def test_capture_microscope_is_a_shared_device_action():
    assert "capture_microscope" in SHARED_DEVICE_ACTIONS


def test_plan_resources_flags_microscope():
    plan = run_flow.plan_resources(
        [{"action": "capture_microscope"}, {"action": "capture_and_save"}])
    assert plan.robot_ids == [] and plan.picus2_robots == set()
    assert (plan.needs_scale, plan.needs_camera, plan.needs_microscope, plan.needs_dmm) == (
        False, True, True, False)


def test_executor_routes_capture_microscope_to_shared_devices():
    mock = MockSharedDevices(use_microscope=True)
    result = asyncio.run(execute_step({"action": "capture_microscope", "file_path": ""},
                                      robots={}, shared_devices=mock))
    assert result["image_path"].startswith("microscope_images/")
    result = asyncio.run(execute_shared_device_step(
        {"action": "capture_microscope", "file_path": "a.jpg"}, mock))
    assert result == {"image_path": "a.jpg"}


def test_logger_tags_microscope_images(tmp_path):
    src = tmp_path / "flow.json"
    src.write_text("{}", encoding="utf-8")
    exp = ExperimentLogger("t", "d", str(src), base_dir=str(tmp_path / "logs"))
    p = exp.next_image_path(3, tag="microscope")
    assert os.path.basename(p).startswith("step003_microscope_")
    assert os.path.basename(exp.next_image_path(3)).startswith("step003_")
    assert "_microscope_" not in os.path.basename(exp.next_image_path(3))


# ----------------------------------------------------------------------
# Config / CLI
# ----------------------------------------------------------------------
def test_config_defaults_have_microscope_index():
    assert "microscope_index" in lab_config.DEFAULTS["shared_devices"]


def test_resolve_ports_microscope_priority(monkeypatch):
    parser = run_flow.build_parser()
    monkeypatch.delenv("MICROSCOPE_INDEX", raising=False)
    args = parser.parse_args(["x.json"])
    _, shared = run_flow.resolve_ports(args)
    assert shared["microscope_index"] == lab_config.get_shared_devices()["microscope_index"]

    monkeypatch.setenv("MICROSCOPE_INDEX", "5")
    _, shared = run_flow.resolve_ports(parser.parse_args(["x.json"]))
    assert shared["microscope_index"] == 5

    _, shared = run_flow.resolve_ports(parser.parse_args(["x.json", "--microscope-index", "9"]))
    assert shared["microscope_index"] == 9


# ----------------------------------------------------------------------
# End to end (mock)
# ----------------------------------------------------------------------
def _latest_run_dir(logs_dir):
    runs = [os.path.join(r, d) for r, ds, _ in os.walk(logs_dir) for d in ds
            if os.path.exists(os.path.join(r, d, "metadata.json"))]
    assert runs
    return max(runs, key=os.path.getmtime)


def test_example_flow_validates():
    assert run_flow.main([EXAMPLE_FLOW, "--validate-only"]) == 0


def test_mock_run_saves_microscope_image_into_run_folder(tmp_path, monkeypatch):
    flow = tmp_path / "micro.json"
    flow.write_text(json.dumps({
        "name": "microscope mock",
        "description": "one webcam photo and one microscope photo",
        "steps": [
            {"action": "capture_and_save", "file_path": ""},
            {"action": "capture_microscope", "file_path": ""},
            {"action": "go_home", "robot_id": 1},
        ],
    }), encoding="utf-8")
    monkeypatch.setattr(run_flow, "LOGS_DIR", str(tmp_path / "logs"))

    assert run_flow.main([str(flow), "--mock"]) == 0

    run_dir = _latest_run_dir(str(tmp_path / "logs"))
    import csv
    with open(os.path.join(run_dir, "measurements.csv"), encoding="utf-8-sig", newline="") as f:
        rows = {r["action"]: r for r in csv.DictReader(f)}
    micro = rows["capture_microscope"]["image_path"]
    assert micro.startswith(os.path.join(run_dir, "images"))
    assert "_microscope_" in os.path.basename(micro)
    assert "_microscope_" not in os.path.basename(rows["capture_and_save"]["image_path"])
    meta = json.load(open(os.path.join(run_dir, "metadata.json"), encoding="utf-8"))
    assert meta["images_captured"] == 2
