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
        # When a source stamp stops advancing the corresponding key's value below
        # stops being refreshed, which lets us detect a frozen upstream source.
        self._origin_seen = {}  # kind -> monotonic time the source stamp last changed
        self._last_origin_stamp = {}  # kind -> last observed source stamp_ns

    def _note_origin(self, kind: str, source_stamp_ns=None) -> None:
        now = self._clock()
        last = self._last_origin_stamp.get(kind)
        if last is None or source_stamp_ns != last:
            self._origin_seen[kind] = now
        if source_stamp_ns is not None:
            self._last_origin_stamp[kind] = source_stamp_ns

    def origin_age(self, kind: str) -> Optional[float]:
        seen = self._origin_seen.get(kind)
        if seen is None:
            return None
        return max(0.0, self._clock() - seen)

    def source_is_stale(self, kind: str, snapshot) -> bool:
        # Prefer the upstream source freshness (when the stamp stopped advancing)
        # over the local arrival time for deciding staleness.
        threshold = getattr(self.config, kind + "_stale_sec")
        if threshold <= 0:
            return False
        if snapshot is None:
            return True
        origin = self.origin_age(kind)
        return origin is not None and origin > threshold

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
            self._note_origin("map", source_stamp_ns)
        self._notify({"type": "map_revision", "revision": snapshot.revision,
                      "stamp_ns": source_stamp_ns, "frame_id": frame_id})
        return snapshot

    def set_data(self, kind: str, payload: dict, source_stamp_ns=None, frame_id="") -> None:
        snapshot = DataSnapshot(payload, self._clock(), source_stamp_ns, frame_id)
        with self._lock:
            setattr(self, "_" + kind, snapshot)
            self._note_origin(kind, source_stamp_ns)
        if kind in ("pose", "scan"):
            self._notify({"type": kind, "payload": payload})

    def get(self, kind: str):
        with self._lock:
            return getattr(self, "_" + kind)

    def age(self, snapshot) -> Optional[float]:
        return None if snapshot is None else max(0.0, self._clock() - snapshot.received_at)

    def is_stale(self, kind: str, snapshot) -> bool:
        threshold = getattr(self.config, kind + "_stale_sec")
        return threshold > 0 and (snapshot is None or self.source_is_stale(kind, snapshot))

    def status_payload(self) -> dict:
        with self._lock:
            map_ok = self._map is not None and not self.is_stale("map", self._map)
            pose_ok = self._pose is not None and not self.is_stale("pose", self._pose)
            result = {
                "state": "ok" if (map_ok and pose_ok) else "degraded",
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
                    "source_age_sec": self.origin_age(kind),
                    "stale": self.is_stale(kind, snapshot),
                    "frame_id": snapshot.frame_id if snapshot else "",
                }
            return result

    def add_error(self, error: str) -> None:
        with self._lock:
            self.status.errors.append(error)
            if len(self.status.errors) > 200:
                del self.status.errors[:-200]
