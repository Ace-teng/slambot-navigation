from web_bridge.models import GatewayConfig
from web_bridge.state import GatewayState


def test_state_starts_degraded_and_updates_latest_value():
    now = [10.0]
    state = GatewayState(GatewayConfig(), clock=lambda: now[0])
    assert state.status_payload()["state"] == "degraded"
    state.set_data("pose", {"x": 1}, 1, "map")
    assert state.get("pose").payload["x"] == 1
    state.set_map({"data": [0]}, 2, "map")
    assert state.status_payload()["state"] == "ok"
    state.set_data("pose", {"x": 2}, 3, "map")
    assert state.get("pose").payload["x"] == 2


def test_stale_threshold_is_applied_without_sleep():
    now = [10.0]
    state = GatewayState(GatewayConfig(scan_stale_sec=1.0), clock=lambda: now[0])
    state.set_data("scan", {"ranges": []}, 1, "laser")
    now[0] = 12.0
    assert state.status_payload()["sources"]["scan"]["stale"] is True


def test_map_is_not_stale_by_default():
    now = [10.0]
    state = GatewayState(GatewayConfig(), clock=lambda: now[0])
    state.set_map({"data": []}, 1, "map")
    now[0] = 100.0
    assert state.status_payload()["sources"]["map"]["stale"] is False
