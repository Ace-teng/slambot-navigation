from dataclasses import dataclass, field
from typing import Any, Optional


@dataclass(frozen=True)
class DataSnapshot:
    payload: dict[str, Any]
    received_at: float
    source_stamp_ns: Optional[int] = None
    frame_id: str = ""


@dataclass(frozen=True)
class MapSnapshot:
    payload: dict[str, Any]
    revision: int
    received_at: float
    source_stamp_ns: Optional[int] = None
    frame_id: str = ""


@dataclass
class GatewayConfig:
    map_topic: str = "map"
    scan_topic: str = "scan"
    odom_topic: str = "odom"
    tf_topic: str = "tf"
    tf_static_topic: str = "tf_static"
    map_frame: str = "map"
    base_frame: str = "base_footprint"
    map_stale_sec: float = 0.0
    scan_stale_sec: float = 2.0
    odom_stale_sec: float = 2.0
    pose_stale_sec: float = 1.0
    max_map_cells: int = 16_000_000
    max_scan_samples: int = 10_000
    scan_rate_hz: float = 5.0
    pose_rate_hz: float = 10.0
    max_clients: int = 8
    client_queue_depth: int = 16
    host: str = "127.0.0.1"
    port: int = 8080


@dataclass
class GatewayStatus:
    errors: list[str] = field(default_factory=list)
    dropped_maps: int = 0
    dropped_scans: int = 0
    dropped_events: int = 0
