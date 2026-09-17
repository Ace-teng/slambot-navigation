# Autonomous exploration mapping

`autonomous_exploration` is one ROS 2 Python node that moves the robot through
reachable frontiers while the existing `slam_toolbox` node owns mapping. It
does not implement SLAM, depend on Nav2, or use a behavior tree.

## Start

Start the robot and the existing SLAM launch first, then run:

```bash
ros2 launch navigation autonomous_exploration.launch.py
```

For a namespaced robot:

```bash
ros2 launch navigation autonomous_exploration.launch.py \
  namespace:=robot_1 \
  map_frame:=robot_1/map \
  odom_frame:=robot_1/odom \
  base_frame:=robot_1/base_footprint
```

All configured topic names are relative, so the namespace is applied to
`map`, `scan`, `odom`, `controller/cmd_vel`, and the map-save service.
TF frame IDs are not namespaced by ROS automatically, so namespaced robots
must also pass their prefixed map, odometry, and base frame IDs as shown.

## Responsibilities and TF

The expected TF chain is:

```text
map -> odom -> base_footprint -> lidar_frame
```

- `slam_toolbox` publishes `map -> odom` and the occupancy grid.
- The odometry stack publishes `odom -> base_footprint`.
- The robot description publishes the static sensor transform.
- The exploration node only queries TF; it does not publish these transforms.

## Interfaces

Subscriptions:

- `map` (`nav_msgs/msg/OccupancyGrid`), reliable/transient-local QoS.
- `scan` (`sensor_msgs/msg/LaserScan`), sensor-data QoS.
- `odom` (`nav_msgs/msg/Odometry`), sensor-data QoS.
- `goal_pose` (`geometry_msgs/msg/PoseStamped`) only when
  `accept_external_goals` is enabled.

Publications:

- `controller/cmd_vel` (`geometry_msgs/msg/Twist`).
- `~/planned_path` (`nav_msgs/msg/Path`).
- `~/current_goal` (`geometry_msgs/msg/PoseStamped`).
- `~/frontiers` (`visualization_msgs/msg/MarkerArray`).
- `~/status` (`std_msgs/msg/String`).

Control services use `std_srvs/srv/Trigger`:

- `~/start`
- `~/pause`
- `~/resume`
- `~/stop`

At completion the node calls `slam_toolbox/srv/SaveMap`. The service name,
output map name, timeout, and automatic-save behavior are parameters. The node
enters `COMPLETED` only after a successful service response; an unavailable,
failed, or timed-out save enters `ERROR`.

## Workflow

The finite-state machine waits for fresh scan and odometry data, a valid TF,
and a fresh map. It can rotate in place for an initial scan, then repeats:

1. Inflate obstacles using robot radius and safety margin.
2. Flood-fill safely reachable known free space.
3. Cluster free cells bordering unknown space into frontiers.
4. Generate safe frontier goals and score information gain, path length,
   obstacle clearance, heading change, and previous failures.
5. Plan an eight-connected A* path without crossing unknown cells or cutting
   corners, then simplify it with the same collision rules.
6. Track the path with a Pure Pursuit-style controller while the laser safety
   layer slows, stops, and limits turns near obstacles.
7. Replan on path invalidation, excessive path deviation, lack of progress,
   goal timeout, blocked timeout, map geometry change, or planning timeout.

Failed goals are temporarily blacklisted and retain a scoring penalty after
the blacklist expires. Repeated absence of a reachable goal completes mapping
only after the known map area has also remained stable. This includes small or
permanently unreachable residual frontiers.

During detection, planning, pause, stop, error, save, and all stale-data
states, the node explicitly publishes zero velocity. Shutdown also publishes
zero velocity. A base-controller command watchdog is still recommended as an
independent hardware safety layer.

## Configuration

`config/autonomous_exploration.yaml` contains:

- Topic and frame names.
- Control and frontier-analysis rates.
- Robot radius, safety margin, occupancy threshold, and frontier filtering.
- Goal scoring weights and blacklist duration.
- Speed, lookahead, path-deviation, progress, and obstacle-clearance limits.
- Sensor, odometry, TF, map, goal, blocked, stuck, planning, and save timeouts.
- Initial scan, completion policy, external-goal policy, and map-save options.

Robot geometry, velocity limits, stop distances, topic names, TF frames, and
the output map name must be checked on the real robot before unattended use.

## Verification

```bash
source /opt/ros/humble/setup.bash
python3 -m compileall -q \
  src/navigation/navigation \
  src/navigation/launch \
  src/navigation/test
PYTHONPATH=src/navigation:$PYTHONPATH python3 -m pytest -q \
  src/navigation/test/test_exploration_core.py \
  src/navigation/test/test_autonomous_exploration_node.py
colcon build --packages-select navigation
```

These checks do not replace simulation and real-robot validation of TF,
odometry quality, sensor QoS, footprint clearance, motor direction, emergency
stopping, recovery behavior, and map saving.
