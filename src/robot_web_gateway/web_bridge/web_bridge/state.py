import threading
import time
from dataclasses import replace
from typing import Optional

from .models import DataSnapshot, GatewayConfig, GatewayStatus, MapSnapshot


class GatewayState:
    def __init__(self, config: GatewayConfig, clock=time.monotonic):
        self.config = config
        self._clock = clock
        self._lock = threading.RLock()
        self._map: Optional[MapSnapshot] = None
        self._scan: Optional[DataSnapshot] = None
        self._pose: Optional[DataSnapshot] = None
        self._odom: Optional[DataSnapshot] = None
        self._listeners = []
        self.status = GatewayStatus()

    def add_listener(self, listener) -> None:
        with self._lock:
            self._listeners.append(listener)

    def _notify(self, event: dict) -> None:
        for listener in tuple(self._listeners):
            try:
                listener(event)
            except Exception:
                self.add_error("listener failed")

    def set_map(self, payload: dict, source_stamp_ns: int, frame_id: str) -> MapSnapshot:
        with self._lock:
            revision = (self._map.revision + 1) if self._map else 1
            self._map = MapSnapshot(payload, revision, self._clock(), source_stamp_ns, frame_id)
            snapshot = self._map
        self._notify({"type": "map_revision", "revision": snapshot.revision,
                      "stamp_ns": source_stamp_ns, "frame_id": frame_id})
        return snapshot

    def set_data(self, kind: str, payload: dict, source_stamp_ns=None, frame_id="") -> None:
        snapshot = DataSnapshot(payload, self._clock(), source_stamp_ns, frame_id)
        with self._lock:
            setattr(self, "_" + kind, snapshot)
        if kind in ("pose", "scan"):
            self._notify({"type": kind, "payload": payload})

    def get(self, kind: str):
        with self._lock:
            return getattr(self, "_" + kind)

    def age(self, snapshot) -> Optional[float]:
        return None if snapshot is None else max(0.0, self._clock() - snapshot.received_at)

    def is_stale(self, kind: str, snapshot) -> bool:
        threshold = getattr(self.config, kind + "_stale_sec")
        return threshold > 0 and (snapshot is None or self.age(snapshot) > threshold)

    def status_payload(self) -> dict:
        with self._lock:
            result = {
                "state": "ok" if self._map and self._pose else "degraded",
                "read_only": True,
                "map_revision": self._map.revision if self._map else 0,
                "sources": {},
                "errors": list(self.status.errors[-10:]),
                "dropped_maps": self.status.dropped_maps,
                "dropped_scans": self.status.dropped_scans,
            }
            for kind in ("map", "scan", "pose", "odom"):
                snapshot = getattr(self, "_" + kind)
                result["sources"][kind] = {
                    "available": snapshot is not None,
                    "age_sec": self.age(snapshot),
                    "stale": self.is_stale(kind, snapshot),
                    "frame_id": snapshot.frame_id if snapshot else "",
                }
            return result

    def add_error(self, error: str) -> None:
        with self._lock:
            self.status.errors.append(error)
