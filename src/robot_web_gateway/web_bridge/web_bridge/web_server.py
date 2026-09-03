import asyncio
import json
from collections import deque

try:
    from aiohttp import web
except ImportError:  # pragma: no cover - reported when starting the node
    web = None

from .serialization import json_bytes


class EventHub:
    def __init__(self, max_clients=8, queue_depth=16):
        self.max_clients = max_clients
        self.queue_depth = queue_depth
        self._clients = []

    def add(self):
        if len(self._clients) >= self.max_clients:
            raise RuntimeError("maximum websocket clients reached")
        client = deque(maxlen=self.queue_depth)
        self._clients.append(client)
        return client

    def remove(self, client):
        if client in self._clients:
            self._clients.remove(client)

    def publish(self, event):
        encoded = json.dumps(event, ensure_ascii=False, separators=(",", ":"), allow_nan=False)
        for client in tuple(self._clients):
            if len(client) == client.maxlen:
                client.popleft()
            client.append(encoded)

    @property
    def client_count(self):
        return len(self._clients)

    async def wait(self, client):
        while client:
            return client.popleft()
        await asyncio.sleep(0.05)
        return await self.wait(client)


def create_app(state, hub=None):
    if web is None:
        raise RuntimeError("aiohttp is required; install python3-aiohttp")
    hub = hub or EventHub(state.config.max_clients, state.config.client_queue_depth)
    state.add_listener(hub.publish)
    app = web.Application()

    async def health(_request):
        return web.json_response({"status": "ok", "read_only": True, "api_version": "v1"})

    async def status(_request):
        payload = state.status_payload()
        payload["websocket_clients"] = hub.client_count
        return web.json_response(payload)

    async def snapshot(request, kind):
        snapshot = state.get(kind)
        if snapshot is None:
            raise web.HTTPServiceUnavailable(text=json.dumps({"error": kind + "_unavailable"}), content_type="application/json")
        payload = dict(snapshot.payload)
        payload["age_sec"] = state.age(snapshot)
        payload["stale"] = state.is_stale(kind, snapshot)
        if kind == "map":
            payload["revision"] = snapshot.revision
            etag = 'W/"map-%d"' % snapshot.revision
            if request.headers.get("If-None-Match") == etag:
                return web.Response(status=304, headers={"ETag": etag})
            return web.Response(body=json_bytes(payload), content_type="application/json", headers={"ETag": etag})
        return web.json_response(payload)

    async def map_view(request):
        return await snapshot(request, "map")

    async def pose_view(request):
        return await snapshot(request, "pose")

    async def scan_view(request):
        return await snapshot(request, "scan")

    async def stream(request):
        ws = web.WebSocketResponse(heartbeat=30)
        await ws.prepare(request)
        try:
            client = hub.add()
        except RuntimeError as exc:
            await ws.close(code=1013, message=str(exc).encode())
            return ws
        current_map = state.get("map")
        await ws.send_json({
            "type": "hello",
            "read_only": True,
            "map_revision": current_map.revision if current_map else 0,
        })

        async def sender():
            while not ws.closed:
                try:
                    event = await asyncio.wait_for(hub.wait(client), timeout=30)
                    await ws.send_str(event)
                except asyncio.TimeoutError:
                    await ws.ping()

        async def receiver():
            async for message in ws:
                if message.type in (web.WSMsgType.CLOSE, web.WSMsgType.ERROR):
                    return
                await ws.send_json({"type": "error", "error": "read_only"})

        sender_task = asyncio.create_task(sender())
        receiver_task = asyncio.create_task(receiver())
        try:
            await asyncio.wait(
                (sender_task, receiver_task),
                return_when=asyncio.FIRST_COMPLETED,
            )
        finally:
            sender_task.cancel()
            receiver_task.cancel()
            await asyncio.gather(sender_task, receiver_task, return_exceptions=True)
            hub.remove(client)
        return ws

    app.router.add_get("/api/v1/health", health)
    app.router.add_get("/api/v1/status", status)
    app.router.add_get("/api/v1/map", map_view)
    app.router.add_get("/api/v1/pose", pose_view)
    app.router.add_get("/api/v1/scan", scan_view)
    app.router.add_get("/api/v1/ws", stream)
    app["event_hub"] = hub
    return app
