import json
import math
from typing import Any


def finite(value: float) -> float:
    if not math.isfinite(value):
        raise ValueError("value must be finite")
    return float(value)


def stamp_ns(stamp: Any) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    for value in (x, y, z, w):
        finite(value)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0:
        raise ValueError("quaternion must not be zero")
    x, y, z, w = (value / norm for value in (x, y, z, w))
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def json_bytes(payload: Any) -> bytes:
    return json.dumps(payload, ensure_ascii=False, separators=(",", ":"), allow_nan=False).encode("utf-8")


def _number(value: float | int) -> float | int:
    return finite(value) if isinstance(value, float) else value


def serialize_map(message: Any, max_cells: int) -> dict[str, Any]:
    width, height = int(message.info.width), int(message.info.height)
    if width < 0 or height < 0 or width * height != len(message.data):
        raise ValueError("map dimensions do not match data")
    if width * height > max_cells:
        raise ValueError("map exceeds configured cell limit")
    origin = message.info.origin
    return {
        "stamp_ns": stamp_ns(message.header.stamp),
        "frame_id": message.header.frame_id,
        "width": width,
        "height": height,
        "resolution": finite(message.info.resolution),
        "origin": {
            "position": {"x": finite(origin.position.x), "y": finite(origin.position.y), "z": finite(origin.position.z)},
            "orientation": {"x": finite(origin.orientation.x), "y": finite(origin.orientation.y), "z": finite(origin.orientation.z), "w": finite(origin.orientation.w)},
        },
        "data": [int(value) for value in message.data],
    }


def serialize_scan(message: Any, max_samples: int) -> dict[str, Any]:
    if len(message.ranges) > max_samples:
        raise ValueError("scan exceeds configured sample limit")

    def clean(values: Any) -> list[float | None]:
        result = []
        for value in values:
            result.append(float(value) if math.isfinite(value) else None)
        return result

    ranges = clean(message.ranges)
    intensities = clean(message.intensities) if message.intensities else []
    return {
        "stamp_ns": stamp_ns(message.header.stamp),
        "frame_id": message.header.frame_id,
        "angle_min": finite(message.angle_min),
        "angle_max": finite(message.angle_max),
        "angle_increment": finite(message.angle_increment),
        "time_increment": finite(message.time_increment),
        "scan_time": finite(message.scan_time),
        "range_min": finite(message.range_min),
        "range_max": finite(message.range_max),
        "ranges": ranges,
        "intensities": intensities,
        "invalid_range_count": ranges.count(None),
    }


def serialize_pose(transform: Any, target_frame: str, source_frame: str, source_stamp_ns: int) -> dict[str, Any]:
    t, q = transform.transform.translation, transform.transform.rotation
    return {
        "stamp_ns": source_stamp_ns,
        "target_frame": target_frame,
        "source_frame": source_frame,
        "position": {"x": finite(t.x), "y": finite(t.y), "z": finite(t.z)},
        "orientation": {"x": finite(q.x), "y": finite(q.y), "z": finite(q.z), "w": finite(q.w)},
        "yaw": quaternion_to_yaw(q.x, q.y, q.z, q.w),
    }
