# web_bridge

`web_bridge` is a read-only ROS 2 Humble gateway for browser clients. It subscribes to the configured map, laser scan, odometry, and TF topics and exposes stable HTTP/WebSocket DTOs without publishing robot control commands.

## Run

```bash
ros2 launch web_bridge web_bridge.launch.py \
  params_file:=$(ros2 pkg prefix web_bridge)/share/web_bridge/config/web_bridge.yaml
```

Endpoints:

- `GET /api/v1/health` — process liveness
- `GET /api/v1/status` — source availability and freshness
- `GET /api/v1/map` — latest `OccupancyGrid` snapshot and weak ETag
- `GET /api/v1/pose` — latest TF pose from `map` to `base_footprint`
- `GET /api/v1/scan` — latest filtered `LaserScan` snapshot
- `GET /api/v1/ws` — read-only real-time events

The default bind address is `127.0.0.1`. Use a reverse proxy for TLS and authentication before exposing the service beyond a trusted host. The first release is intentionally read-only: it does not publish `cmd_vel`, call navigation actions, or accept arbitrary ROS topic names.

`/map` uses `-1` for unknown, `0` for free, and `1..100` for occupied probability. Non-finite laser values are represented as JSON `null` without changing their array positions.
