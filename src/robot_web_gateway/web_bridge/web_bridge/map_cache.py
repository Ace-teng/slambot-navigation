import queue
import threading
from typing import Optional


class MapCacheWorker:
    """Encode maps off the ROS callback path, keeping only the newest pending map."""

    def __init__(self, state, serializer, max_pending=1):
        self.state = state
        self.serializer = serializer
        self._pending: Optional[object] = None
        self._condition = threading.Condition()
        self._stopped = False
        self.coalesced = 0
        self._thread = threading.Thread(target=self._run, name="map-encoder", daemon=True)
        self._thread.start()

    def submit(self, message) -> None:
        with self._condition:
            if self._pending is not None:
                self.coalesced += 1
                self.state.status.dropped_maps += 1
            self._pending = message
            self._condition.notify()

    def close(self) -> None:
        with self._condition:
            self._stopped = True
            self._condition.notify_all()
        self._thread.join(timeout=2.0)

    def _run(self) -> None:
        while True:
            with self._condition:
                while self._pending is None and not self._stopped:
                    self._condition.wait()
                if self._pending is None and self._stopped:
                    return
                message = self._pending
                self._pending = None
            try:
                payload = self.serializer(message)
                self.state.set_map(payload, payload["stamp_ns"], payload["frame_id"])
            except (TypeError, ValueError, AttributeError) as exc:
                self.state.add_error("map: " + str(exc))
