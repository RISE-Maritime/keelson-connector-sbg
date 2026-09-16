"""Tests for EKF-status gating, heading, accuracies and yaw rate.

Sample lines are sbgBasicLogger console output (-p --status-format=decimal)
captured from the Ellipse-N on case, 2026-09-16.
"""

import importlib.util
import math
import pathlib
import sys
from importlib.machinery import SourceFileLoader
from types import SimpleNamespace
from unittest.mock import Mock

import keelson
import pytest
from keelson.payloads.Decomposed3DVector_pb2 import Decomposed3DVector
from keelson.payloads.Primitives_pb2 import TimestampedFloat

BIN = pathlib.Path(__file__).resolve().parent.parent / "bin"
sys.path.insert(0, str(BIN))

import sbg_parser  # noqa: E402

_loader = SourceFileLoader("sbg_main", str(BIN / "main"))
_spec = importlib.util.spec_from_loader(_loader.name, _loader)
sbg_main = importlib.util.module_from_spec(_spec)
_loader.exec_module(sbg_main)

# EKF in VERTICAL_GYRO mode: attitude valid, no heading, free-drifting nav.
EULER_NO_HEADING = (
    "euler       : 273         0.115756    -0.168374   -22.622351  0.230137"
    "    0.229791    180.000000  nan         nan         nan"
)
NAV_INVALID = (
    "nav         : 273         107.923157  127.717094  1573.713623 500.000000"
    "  500.000000  500.000000  83.068740   98.779851   -500048.138004924.387817"
    "  208.270309  1000.000000 48.138004"
)
IMU = (
    "imuData     : 1023        -0.020853   -0.017224   -9.734270   1.010215"
    "    -1.750381   0.899941    42.911011"
)

# Same lines with the EKF in NAV_POSITION mode: status 0xCF4 sets attitude,
# heading, velocity and position valid plus GNSS1 velocity/position used.
STATUS_FULL_NAV = 4 | 0xF0 | (1 << 10) | (1 << 11)
EULER_HEADING = (
    f"euler       : {STATUS_FULL_NAV}        0.115756    -0.168374   -22.622351  0.230137"
    "    0.229791    0.450000    -21.822351  -0.800000   51.200000"
)
NAV_VALID = (
    f"nav         : {STATUS_FULL_NAV}        0.010000    0.020000    -0.003000   0.050000"
    "    0.050000    0.080000    38.473986   -8.869593   8.514999    0.300000"
    "    0.400000    0.600000    48.138004"
)


@pytest.fixture
def bus():
    """Mock session recording (key, envelope) per put."""
    published = []
    session = Mock()
    session.put = Mock(side_effect=lambda key, value: published.append((key, value)))
    args = SimpleNamespace(
        realm="rise", entity_id="case", source_id="sbg/0", frame_id=None
    )
    sbg_main.last_attitude_rad = None
    sbg_main.last_ekf_validity.clear()
    yield session, args, published
    sbg_main.last_attitude_rad = None
    sbg_main.last_ekf_validity.clear()


def floats(published):
    """Map '<subject>/<source_id>' to the TimestampedFloat value (last wins)."""
    values = {}
    for key, envelope in published:
        _, _, payload_bytes = keelson.uncover(envelope)
        payload = TimestampedFloat()
        try:
            payload.ParseFromString(payload_bytes)
        except Exception:
            continue
        values[key.split("/pubsub/", 1)[1]] = payload.value
    return values


# ---------------------------------------------------------------- parser


def test_status_helpers_on_vertical_gyro_status():
    assert sbg_parser.ekf_solution_mode_name(273) == "VERTICAL_GYRO"
    assert not sbg_parser.heading_valid(273)
    assert not sbg_parser.position_valid(273)
    assert not sbg_parser.velocity_valid(273)


def test_status_helpers_on_full_navigation():
    assert sbg_parser.ekf_solution_mode_name(STATUS_FULL_NAV) == "NAV_POSITION"
    assert sbg_parser.heading_valid(STATUS_FULL_NAV)
    assert sbg_parser.position_valid(STATUS_FULL_NAV)


def test_parse_nav_line_keeps_status():
    assert sbg_parser.parse_nav_line(NAV_INVALID).status == 273


def test_yaw_rate_level_equals_gyro_z():
    assert sbg_parser.yaw_rate(1.0, 3.0, 0.0, 0.0) == pytest.approx(3.0)


def test_yaw_rate_rolled_90_uses_gyro_y():
    assert sbg_parser.yaw_rate(2.0, 3.0, math.pi / 2, 0.0) == pytest.approx(2.0)


def test_yaw_rate_undefined_at_pitch_90():
    assert sbg_parser.yaw_rate(1.0, 1.0, 0.0, math.pi / 2) is None


# ---------------------------------------------------------------- handlers


def test_euler_without_heading_reference_publishes_no_heading(bus):
    session, args, published = bus
    sbg_main.process_euler(session, args, EULER_NO_HEADING)
    values = floats(published)

    assert values["yaw_deg/sbg/ins/0"] == pytest.approx(-22.622351)
    assert values["heading_accuracy_deg/sbg/ins/0"] == pytest.approx(180.0)
    assert values["roll_accuracy_deg/sbg/ins/0"] == pytest.approx(0.230137)
    assert "heading_true_north_deg/sbg/ins/0" not in values
    assert "heading_magnetic_deg/sbg/ins/0" not in values
    assert "magnetic_variation_deg/sbg/ins/0" not in values  # nan


def test_euler_with_valid_heading_publishes_0_360_heading(bus):
    session, args, published = bus
    sbg_main.process_euler(session, args, EULER_HEADING)
    values = floats(published)

    assert values["heading_true_north_deg/sbg/ins/0"] == pytest.approx(337.377649)
    assert values["heading_magnetic_deg/sbg/ins/0"] == pytest.approx(338.177649)
    assert values["magnetic_variation_deg/sbg/ins/0"] == pytest.approx(-0.8)
    assert values["heading_accuracy_deg/sbg/ins/0"] == pytest.approx(0.45)


def test_euler_logs_heading_validity_once_per_change(bus, caplog):
    session, args, _ = bus
    with caplog.at_level("WARNING", logger="ellipse_n"):
        for line in (EULER_NO_HEADING, EULER_NO_HEADING, EULER_HEADING):
            sbg_main.process_euler(session, args, line)
    messages = [r.getMessage() for r in caplog.records]
    assert len(messages) == 2
    assert "mode=VERTICAL_GYRO" in messages[0] and "heading_valid=NO" in messages[0]
    assert "mode=NAV_POSITION" in messages[1] and "heading_valid=yes" in messages[1]


def test_nav_invalid_publishes_nothing(bus):
    session, args, published = bus
    sbg_main.process_nav(session, args, NAV_INVALID)
    assert published == []


def test_nav_valid_publishes_position_accuracy(bus):
    session, args, published = bus
    sbg_main.process_nav(session, args, NAV_VALID)
    keys = {key.split("/pubsub/", 1)[1] for key, _ in published}
    assert keys == {
        "location_fix/sbg/ins/0",
        "location_fix_accuracy_horizontal_m/sbg/ins/0",
        "location_fix_accuracy_vertical_m/sbg/ins/0",
        "ned_velocity_mps/sbg/ins/0",
    }
    values = floats(published)
    assert values["location_fix_accuracy_horizontal_m/sbg/ins/0"] == pytest.approx(0.5)
    assert values["location_fix_accuracy_vertical_m/sbg/ins/0"] == pytest.approx(0.6)


def test_imu_angular_velocity_converted_from_deg_to_rad(bus):
    session, args, published = bus
    sbg_main.process_imu_data(session, args, IMU)
    envelope = next(v for k, v in published if "/angular_velocity_radps/" in k)
    _, _, payload_bytes = keelson.uncover(envelope)
    vector = Decomposed3DVector()
    vector.ParseFromString(payload_bytes)
    assert vector.vector.x == pytest.approx(math.radians(1.010215))
    assert vector.vector.z == pytest.approx(math.radians(0.899941))


def test_imu_yaw_rate_needs_attitude_first(bus):
    session, args, published = bus
    sbg_main.process_imu_data(session, args, IMU)
    assert "yaw_rate_degps/sbg/ins/0" not in floats(published)

    sbg_main.process_euler(session, args, EULER_NO_HEADING)
    sbg_main.process_imu_data(session, args, IMU)
    roll, pitch = math.radians(0.115756), math.radians(-0.168374)
    expected = (-1.750381 * math.sin(roll) + 0.899941 * math.cos(roll)) / math.cos(
        pitch
    )
    assert floats(published)["yaw_rate_degps/sbg/ins/0"] == pytest.approx(expected)
