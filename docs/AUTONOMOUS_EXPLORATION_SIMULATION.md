# 自动探索建图仿真

该仿真把 Gazebo 世界作为机器人事先未知的物理环境。启动时没有加载
`.yaml/.pgm` 占据栅格；`slam_toolbox` 根据 `/scan`、`/odom` 和 TF 在线发布
`/map`，`autonomous_exploration` 根据已知/未知区域边界规划运动并发布
`/cmd_vel`。

## 场景来源

- 项目：ROBOTIS `turtlebot3_simulations`
- 分支：`humble`
- 固定提交：`a35a56c8b04877dc89772b598084d8ce648a9023`
- 场景：`turtlebot3_gazebo/worlds/turtlebot3_house.world`
- 模型：`turtlebot3_gazebo/models/turtlebot3_house`
- 许可：Apache License 2.0，许可文本随模型保存在 `LICENSE`

下载的场景只提供多房间墙体和障碍物；仿真仍使用本项目的 JetRover 模型、
激光雷达、里程计、SLAM 和自动探索节点。场景所引用的 `cafe_table`、
`first_2015_trash_can`、`mailbox` 和 `table_marble` 资源来自 OSRF 官方
`gazebo_models` 仓库固定提交
`8163eb4b5e7e21985c6591d1c0bfb56468c0093f`，并已一并本地化。

为避免 Gazebo Classic 查询已经退役的在线模型库，原世界中对 `sun` 和
`ground_plane` 的引用已替换为等价的内联 SDF 定义，场景障碍布局未改变。
物理求解频率由官方场景的 1000 Hz/150 次迭代调整为 200 Hz/50 次迭代，
以便复杂房屋在普通开发机上保持接近实时；机器人低速运动和地图几何不变。

## 启动

```bash
cd ~/projects/group2/slambot-navigation
source /opt/ros/humble/setup.bash
colcon build --packages-select navigation slam jetrover_description
source install/setup.bash
ros2 launch jetrover_description autonomous_exploration_sim.launch.py
```

无桌面环境时可关闭两个界面：

```bash
ros2 launch jetrover_description autonomous_exploration_sim.launch.py \
  gui:=false rviz:=false
```

默认出生点为 `(3.50, 2.00)`，位于房屋右上大房间内部，并避开墙壁、
书架和桌子。官方 TurtleBot3 示例的 `(-2.00, -0.50)` 位于中央房间外侧
入口，不适合作为本项目室内自动探索的演示起点。地图完成后默认保存为：

```text
~/.ros/maps/jetrover_exploration_map.pgm
~/.ros/maps/jetrover_exploration_map.yaml
```

可用 `map_output:=/绝对路径/名称` 修改保存位置，不要填写文件后缀。

## 运行时数据流

```text
Gazebo 世界
  -> /scan + /odom + odom→base_footprint
  -> slam_toolbox
  -> /map + map→odom
  -> autonomous_exploration
  -> /cmd_vel
  -> Gazebo 中的 JetRover
```

这不是在已有占据栅格上导航，而是在一个已知仿真环境文件、机器人未知地图的条件下，
验证“自主探索 + 在线 SLAM 建图”的完整闭环。

探索节点的超时、恢复和初始扫描均使用 ROS 时钟：真机运行时等同系统时间，
`use_sim_time:=true` 时跟随 Gazebo `/clock`，不会因仿真低于实时速度而误判卡住。
仿真替身使用平面移动插件，因此固定展示轮的切向摩擦设为零；轮子和车体仍保留
法向碰撞，机器人不会穿过场景墙体。
