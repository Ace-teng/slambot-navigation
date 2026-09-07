from web_bridge.web_server import EventHub


def test_event_hub_keeps_latest_events_with_bounded_queue():
    hub = EventHub(max_clients=1, queue_depth=2)
    client = hub.add()
    hub.publish({"type": "one"})
    hub.publish({"type": "two"})
    hub.publish({"type": "three"})
    assert len(client) == 2
    assert '"two"' in client.popleft()
    assert '"three"' in client.popleft()
    hub.remove(client)
    assert hub.client_count == 0


def test_event_hub_limits_clients():
    hub = EventHub(max_clients=1)
    client = hub.add()
    try:
        try:
            hub.add()
            assert False, "expected client limit"
        except RuntimeError:
            pass
    finally:
        hub.remove(client)
