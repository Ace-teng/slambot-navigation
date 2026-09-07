import time
from types import SimpleNamespace

from web_bridge.map_cache import MapCacheWorker
from web_bridge.models import GatewayConfig
from web_bridge.state import GatewayState


def test_map_cache_commits_revision():
    state = GatewayState(GatewayConfig())
    finished = []

    def serializer(message):
        finished.append(message)
        return {"stamp_ns": 1, "frame_id": "map", "data": message}

    worker = MapCacheWorker(state, serializer)
    worker.submit([0, 100])
    for _ in range(20):
        if state.get("map"):
            break
        time.sleep(0.01)
    worker.close()
    assert state.get("map").revision == 1
    assert finished == [[0, 100]]


def test_map_cache_rejects_bad_serializer():
    state = GatewayState(GatewayConfig())
    worker = MapCacheWorker(state, lambda _: (_ for _ in ()).throw(ValueError("bad map")))
    worker.submit(SimpleNamespace())
    time.sleep(0.02)
    worker.close()
    assert any("bad map" in error for error in state.status.errors)
