import math
from types import SimpleNamespace

import pytest

from web_bridge.serialization import json_bytes, quaternion_to_yaw, serialize_map, serialize_scan


def stamp():
    return SimpleNamespace(sec=3, nanosec=4)


def vector(x=0.0, y=0.0, z=0.0):
    return SimpleNamespace(x=x, y=y, z=z)


def test_scan_non_finite_values_become_null():
    message = SimpleNamespace(header=SimpleNamespace(stamp=stamp(), frame_id="laser"),
                              angle_min=-1.0, angle_max=1.0, angle_increment=0.1,
                              time_increment=0.0, scan_time=0.1, range_min=0.1,
                              range_max=10.0, ranges=[1.0, math.inf, math.nan], intensities=[])
    payload = serialize_scan(message, 10)
    assert payload["ranges"] == [1.0, None, None]
    assert payload["invalid_range_count"] == 2
    json_bytes(payload)


def test_scan_limit_is_enforced():
    message = SimpleNamespace(header=SimpleNamespace(stamp=stamp(), frame_id="laser"),
                              angle_min=0.0, angle_max=1.0, angle_increment=0.1,
                              time_increment=0.0, scan_time=0.1, range_min=0.1,
                              range_max=10.0, ranges=[1.0, 2.0], intensities=[])
    with pytest.raises(ValueError, match="sample limit"):
        serialize_scan(message, 1)


def test_map_metadata_and_cells_are_preserved():
    origin = SimpleNamespace(position=vector(1.0, 2.0), orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))
    info = SimpleNamespace(width=2, height=1, resolution=0.05, origin=origin)
    message = SimpleNamespace(header=SimpleNamespace(stamp=stamp(), frame_id="map"), info=info, data=[-1, 100])
    payload = serialize_map(message, 10)
    assert payload["stamp_ns"] == 3000000004
    assert payload["data"] == [-1, 100]
    assert payload["origin"]["position"]["x"] == 1.0


def test_map_dimension_mismatch_is_rejected():
    info = SimpleNamespace(width=2, height=2, resolution=0.05,
                           origin=SimpleNamespace(position=vector(), orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)))
    message = SimpleNamespace(header=SimpleNamespace(stamp=stamp(), frame_id="map"), info=info, data=[0])
    with pytest.raises(ValueError, match="dimensions"):
        serialize_map(message, 10)


def test_quaternion_to_yaw():
    assert quaternion_to_yaw(0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4)) == pytest.approx(math.pi / 2)
    with pytest.raises(ValueError):
        quaternion_to_yaw(0.0, 0.0, 0.0, 0.0)
