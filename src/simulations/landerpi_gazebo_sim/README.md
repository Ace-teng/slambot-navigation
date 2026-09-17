# LanderPi PC simulation

ROS 2 Humble + Gazebo Classic simulation for staged path planning and obstacle
avoidance. It is self-contained and does not require a physical robot.

## Included scenario

- 14 m x 10 m city-road network with several building blocks.
- Industrial tank farm with fences and six cylindrical storage tanks.
- Prebuilt 288 x 208 occupancy map at 0.05 m/cell, loaded immediately.
- AMCL initial localization at the Gazebo spawn pose.
- Three-stage route: south avenue, tank-farm east road, north avenue.
- One small dynamic obstacle per road, with a 1.2 m spawn safety radius and
  cleanup after each route stage.

## Build and run

From the repository root:

```bash
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-navigation2 \
  ros-humble-nav2-bringup ros-humble-slam-toolbox ros-humble-xacro \
  ros-humble-robot-state-publisher ros-humble-teleop-twist-keyboard

./scripts/pc_sim/build.sh
./scripts/pc_sim/start.sh demo
```

Other modes:

```bash
./scripts/pc_sim/start.sh navigation  # complete map, interactive Nav2 goals
./scripts/pc_sim/start.sh mapping     # online SLAM instead of the prebuilt map
```

Gazebo runs headless by default and RViz uses Mesa software rendering for WSLg
stability. Set `LIBGL_ALWAYS_SOFTWARE=0` before starting if native GPU rendering
is known to work.
