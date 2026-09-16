"""Tests for EKF-status gating, heading, accuracies, yaw rate, GNSS quality and status.

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


def test_euler_without_heading_reference_still_publishes_heading(bus):
    """Heading is published even when invalid, with its accuracy beside it."""
    session, args, published = bus
    sbg_main.process_euler(session, args, EULER_NO_HEADING)
    values = floats(published)

    assert values["yaw_deg/sbg/ins/0"] == pytest.approx(-22.622351)
    assert values["heading_true_north_deg/sbg/ins/0"] == pytest.approx(337.377649)
    assert values["heading_accuracy_deg/sbg/ins/0"] == pytest.approx(180.0)
    assert values["roll_accuracy_deg/sbg/ins/0"] == pytest.approx(0.230137)
    assert "heading_magnetic_deg/sbg/ins/0" not in values  # nan
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


# ---------------------------------------------------------------- GNSS, status, mag

# Real lines, 2026-09-16: DGPS (position type 3), 16 satellites, no base station.
GNSS_POS = (
    "gnss1Pos    : 37056       273         319562200   38.473986   -8.869593"
    "   8.514999    48.138004   0.389000    0.389000    0.534000    255"
    "         16          65535       65535       0           429496"
)
STATUS = "status      : 127         97          167         8459        255"
MAG = (
    "mag         : 511         0.418055    -0.789704   -0.773458   -0.022151"
    "   -0.006232   -9.741590"
)


def test_gnss_pos_publishes_quality_and_accuracy(bus):
    from keelson.payloads.LocationFixQuality_pb2 import LocationFixQuality

    session, args, published = bus
    args.enable_rtcm_input = False
    sbg_main.process_gnss_pos(session, args, GNSS_POS)

    values = floats(published)
    assert values["location_fix_accuracy_horizontal_m/sbg/gnss/0"] == pytest.approx(
        math.hypot(0.389, 0.389)
    )
    assert values["location_fix_accuracy_vertical_m/sbg/gnss/0"] == pytest.approx(0.534)

    envelope = next(v for k, v in published if "/location_fix_quality/sbg/gnss/0" in k)
    _, _, payload_bytes = keelson.uncover(envelope)
    quality = LocationFixQuality()
    quality.ParseFromString(payload_bytes)
    assert quality.fix_type == LocationFixQuality.FIX_3D
    assert quality.pos_type == LocationFixQuality.POS_TYPE_PSRDIFF
    assert quality.rtk_status == LocationFixQuality.RTK_STATUS_DIFFERENTIAL


def test_gnss_pos_no_correction_warning_without_rtcm(bus, caplog):
    session, args, _ = bus
    args.enable_rtcm_input = False
    with caplog.at_level("WARNING", logger="ellipse_n"):
        sbg_main.process_gnss_pos(session, args, GNSS_POS)
    assert not [r for r in caplog.records if "corrections" in r.getMessage()]


def test_gnss_pos_correction_warning_with_rtcm(bus, caplog):
    session, args, _ = bus
    args.enable_rtcm_input = True
    with caplog.at_level("WARNING", logger="ellipse_n"):
        sbg_main.process_gnss_pos(session, args, GNSS_POS)
    assert [r for r in caplog.records if "corrections" in r.getMessage()]


def test_parse_status_line():
    status = sbg_parser.parse_status_line(STATUS)
    assert (status.general, status.aiding, status.cpu_pct) == (127, 8459, 255)


def test_status_logs_once_per_change(bus, caplog):
    session, args, published = bus
    sbg_main.last_device_status = None
    lost_gnss = STATUS.replace("8459", "8448")  # gnss1 pos/vel/utc no longer received
    with caplog.at_level("WARNING", logger="ellipse_n"):
        for line in (STATUS, STATUS, lost_gnss):
            sbg_main.process_status(session, args, line)
    sbg_main.last_device_status = None
    messages = [r.getMessage() for r in caplog.records]
    assert len(messages) == 2
    assert "all OK" in messages[0]
    assert (
        "aiding received=gnss1_pos, gnss1_vel, gnss1_utc, mag, air_data" in messages[0]
    )
    assert "aiding received=mag, air_data" in messages[1]
    assert published == []


def test_mag_is_not_published(bus):
    session, args, published = bus
    sbg_main.process_mag(session, args, MAG)
    assert published == []


def test_gnss_quality_works_with_fix_type_only_proto(bus, monkeypatch):
    """keelson 0.5.0's LocationFixQuality has only timestamp and fix_type."""
    from google.protobuf import descriptor_pb2, descriptor_pool, timestamp_pb2
    from google.protobuf import message_factory

    file_proto = descriptor_pb2.FileDescriptorProto(
        name="old_lfq.proto",
        package="oldkeelson",
        syntax="proto3",
        dependency=["google/protobuf/timestamp.proto"],
    )
    msg = file_proto.message_type.add(name="LocationFixQuality")
    enum = msg.enum_type.add(name="FixType")
    for number, name in enumerate(["UNKNOWN", "INVALID", "FIX_NO", "FIX_2D", "FIX_3D"]):
        enum.value.add(name=name, number=number)
    msg.field.add(
        name="timestamp",
        number=1,
        type=11,
        label=1,
        type_name=".google.protobuf.Timestamp",
    )
    msg.field.add(
        name="fix_type",
        number=2,
        type=14,
        label=1,
        type_name=".oldkeelson.LocationFixQuality.FixType",
    )
    pool = descriptor_pool.DescriptorPool()
    pool.AddSerializedFile(timestamp_pb2.DESCRIPTOR.serialized_pb)
    pool.Add(file_proto)
    old_cls = message_factory.GetMessageClass(
        pool.FindMessageTypeByName("oldkeelson.LocationFixQuality")
    )
    old_cls.FIX_3D = 4
    old_cls.FIX_NO = 2
    monkeypatch.setattr(sbg_main, "LocationFixQuality", old_cls)

    session, args, published = bus
    sbg_main.put_gnss_quality(session, args, 0, 3, 1)
    _, _, payload_bytes = keelson.uncover(published[0][1])
    decoded = old_cls()
    decoded.ParseFromString(payload_bytes)
    assert decoded.fix_type == 4
