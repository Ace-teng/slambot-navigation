# slambot-navigation

本仓库 Fork 自 [Ace-teng/slambot-navigation](https://github.com/Ace-teng/slambot-navigation)。(This repository is forked from Ace-teng/slambot-navigation.)

ROS 2 workspace for SLAM mapping and autonomous navigation of a wheeled robot
(Hiwonder JetRover chassis family: `JetRover_Mecanum` / `JetRover_Tank` /
`JetRover_Acker`) with an Orbbec depth camera, optional 2D lidar, an arm /
gripper and an offline voice module.

## Status

This workspace is **not** a turn-key, unattended real-robot deliverable yet:

- Several launch/install defects have been fixed, but building and running on a
  real robot still requires the vendor assets listed below and an on-hardware
  validation pass.
- The low-level command path now has a timeout watchdog, e-stop services and
  unified speed limits in `controller/odom_publisher_node.py`. The MCU board
  should still implement its own independent lost-command power cutoff.
- `sim:=true` **does not** mean "hardware safe". Use `use_real_hw:=false` to
  prevent the real controller/arm from being launched. Real driving needs an
  e-stop and a controlled area.
- The arm kinematics modules (`forward_kinematics.py` / `inverse_kinematics.py`)
  are explicit placeholders: they raise `NotImplementedError` until the arm's
  D-H / link parameters are supplied. Do not rely on them for motion planning.

## Required but not committed (vendor/hardware assets)

The following are produced by the hardware vendor and are **not** present in
this repository. Copy them from your kit image or vendor SDK before building:

```text
src/driver/kinematics/kinematics/forward_kinematics.py   (real implementation)
src/driver/kinematics/kinematics/inverse_kinematics.py   (real implementation)
src/xf_mic_asr_offline/lib/arm64/                        (offline_record_lib + deps)
src/xf_mic_asr_offline/feedback_voice/                   (voice prompts wav)
src/simulations/jetrover_description/meshes/             (URDF meshes)
```

The CMake / install logic tolerates the missing `lib/` and `feedback_voice/`
trees so a clean `colcon build` does not fail on their absence.

## Environment

| Variable | Meaning |
|---|---|
| `need_compile` | Must equal the string `True` to use installed package shares; otherwise source paths under `/home/ubuntu/ros2_ws` are used |
| `HOST`, `MASTER` | Robot naming; not SSH hostnames |
| `MACHINE_TYPE` | One of `JetRover_Mecanum`, `JetRover_Tank`, `JetRover_Acker` |
| `LIDAR_TYPE` | One of `A1`, `G4`, `LD19`, `LD14P`, ... |
| `ASR_LANGUAGE` | `Chinese` / `English` |

Typical build (on the robot, after sourcing Humble):

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y   # vendor libs are not in rosdep
colcon build --symlink-install
source install/setup.bash
export need_compile=True HOST=/ MASTER=/
export MACHINE_TYPE=JetRover_Mecanum LIDAR_TYPE=A1
```

## Launch entries

- 2D mapping: `ros2 launch slam slam.launch.py sim:=false`
- RTAB-Map 3D mapping: `ros2 launch slam rtabmap_slam.launch.py sim:=false` — starts
  a new map only when `clear_db:=true` is given; otherwise the previous database
  is preserved.
- 2D navigation: `ros2 launch navigation navigation.launch.py sim:=false map:=map_01`
- RTAB-Map localization/navigation: `ros2 launch navigation rtabmap_navigation.launch.py sim:=false`
- Read-only web gateway: see `src/robot_web_gateway/web_bridge/README.md`

## Notes / limitations

- Mapping and localization each publish `map -> odom`; never run both at the
  same time.
- `example` launch files can move the arm on startup (`start:=true`); do not run
  them to "verify an install".
- Voice data is not saved to disk by default (`save_pcm_local=false`).
- udev rules require the runtime user to be a member of `dialout` / `video`.
- Exposing the web gateway or any ROS node beyond a trusted host requires TLS,
  authentication and DDS/SROS2 policy that is out of scope of this repository.
