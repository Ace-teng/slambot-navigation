#!/usr/bin/env python3
"""Step-by-step Nav2 demo with multiple dynamically inserted obstacles."""

import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


class StagedNavigationDemo(Node):
    def __init__(self):
        super().__init__('staged_navigation_demo')

        transient = QoSProfile(depth=1)
        transient.reliability = ReliabilityPolicy.RELIABLE
        transient.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.plan_client = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')
        self.plan_pub = self.create_publisher(Path, '/staged_demo/planned_path', transient)
        self.rviz_plan_pub = self.create_publisher(Path, '/plan', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/waypoints', 10)
        self.status_pub = self.create_publisher(String, '/staged_demo/status', transient)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, transient)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_pose_callback, 10)

        self.targets = [
            ('Step 1 - southern avenue with obstacle insertion', 5.00, -3.50, 0.0),
            ('Step 2 - east industrial road beside the tank farm', 5.60, 3.75, math.pi / 2.0),
            ('Step 3 - northern avenue across the city blocks', -5.50, 3.75, math.pi),
        ]
        self.obstacle_events = {
            0: [
                (5.0, 'south_roadblock_1', -1.60, -3.20, 0.45, 0.35),
            ],
            1: [
                (5.0, 'industrial_roadblock_1', 5.28, -0.45, 0.35, 0.45),
            ],
            2: [
                (5.0, 'north_roadblock_1', 2.20, 3.42, 0.45, 0.35),
            ],
        }
        self.stage_index = 0
        self.phase = 'WAITING'
        self.map_ready = False
        self.ready_since = None
        self.phase_started = time.monotonic()
        self.spawned_obstacles = set()
        self.active_obstacle = None
        self.current_xy = None
        self.last_feedback_log = 0.0
        self.retry_count = 0
        self.timer = self.create_timer(0.25, self.tick)
        self.say('Waiting for /map and Nav2 action servers...')

    def map_callback(self, msg):
        self.map_ready = msg.info.width > 0 and msg.info.height > 0

    def amcl_pose_callback(self, msg):
        self.current_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def pose(self, x, y, yaw):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        return msg

    def say(self, text):
        prefix = f'[{self.stage_index + 1}/{len(self.targets)}] '
        message = prefix + text
        self.get_logger().info(message)
        self.status_pub.publish(String(data=message))

    def tick(self):
        self.publish_markers()

        if self.phase == 'WAITING':
            ready = (
                self.map_ready
                and self.plan_client.wait_for_server(timeout_sec=0.0)
                and self.nav_client.wait_for_server(timeout_sec=0.0)
            )
            if ready:
                if self.ready_since is None:
                    self.ready_since = time.monotonic()
                    self.say('Map received; waiting 3 seconds for Nav2 lifecycle activation.')
                elif time.monotonic() - self.ready_since >= 3.0:
                    self.request_plan()
            else:
                self.ready_since = None
            return

        elapsed = time.monotonic() - self.phase_started
        if self.phase == 'PREVIEW' and elapsed >= 3.0:
            self.start_navigation()
        elif self.phase == 'NAVIGATING':
            for event in self.obstacle_events.get(self.stage_index, []):
                delay, name, x, y, size_x, size_y = event
                if elapsed >= delay and name not in self.spawned_obstacles:
                    self.spawned_obstacles.add(name)
                    if self.current_xy is not None and math.hypot(
                        self.current_xy[0] - x, self.current_xy[1] - y
                    ) < 1.20:
                        self.say(f'Safety skip: {name} is too close to the robot.')
                    else:
                        self.insert_dynamic_obstacle(name, x, y, size_x, size_y)
                    break
        elif self.phase == 'BETWEEN' and elapsed >= 2.0:
            self.stage_index += 1
            self.retry_count = 0
            if self.stage_index >= len(self.targets):
                self.phase = 'DONE'
                self.say('Demo complete: planning, replanning and avoidance all finished.')
            else:
                self.request_plan()
        elif self.phase == 'RETRY' and elapsed >= 3.0:
            self.request_plan()

    def request_plan(self):
        name, x, y, yaw = self.targets[self.stage_index]
        self.phase = 'PLANNING'
        self.phase_started = time.monotonic()
        self.say(f'{name}: computing the global path to ({x:.1f}, {y:.1f}).')
        goal = ComputePathToPose.Goal()
        goal.goal = self.pose(x, y, yaw)
        goal.use_start = False
        future = self.plan_client.send_goal_async(goal)
        future.add_done_callback(self.plan_goal_response)

    def plan_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.fail_or_retry('Global planner rejected the request.')
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(self.plan_result)

    def plan_result(self, future):
        wrapped = future.result()
        path = wrapped.result.path
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED or not path.poses:
            self.fail_or_retry('No valid global path was found.')
            return
        path.header.stamp = self.get_clock().now().to_msg()
        self.plan_pub.publish(path)
        self.rviz_plan_pub.publish(path)
        self.phase = 'PREVIEW'
        self.phase_started = time.monotonic()
        self.say(f'Global path ready ({len(path.poses)} poses). Previewing it for 3 seconds.')

    def start_navigation(self):
        name, x, y, yaw = self.targets[self.stage_index]
        goal = NavigateToPose.Goal()
        goal.pose = self.pose(x, y, yaw)
        self.phase = 'SENDING'
        self.say('Executing the path; local planner is monitoring /scan for obstacles.')
        future = self.nav_client.send_goal_async(goal, feedback_callback=self.nav_feedback)
        future.add_done_callback(self.nav_goal_response)

    def nav_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.fail_or_retry('Nav2 rejected the navigation goal.')
            return
        self.phase = 'NAVIGATING'
        self.phase_started = time.monotonic()
        result_future = handle.get_result_async()
        result_future.add_done_callback(self.nav_result)

    def nav_feedback(self, feedback_msg):
        now = time.monotonic()
        if now - self.last_feedback_log < 1.0:
            return
        self.last_feedback_log = now
        feedback = feedback_msg.feedback
        self.say(
            f'Following path: {feedback.distance_remaining:.2f} m remaining, '
            f'{feedback.number_of_recoveries} recoveries.'
        )

    def nav_result(self, future):
        wrapped = future.result()
        if wrapped.status == GoalStatus.STATUS_SUCCEEDED:
            self.clear_active_obstacle()
            self.phase = 'BETWEEN'
            self.phase_started = time.monotonic()
            self.say('Stage reached successfully. Preparing the next planning step.')
        else:
            self.fail_or_retry(f'Navigation ended with action status {wrapped.status}.')

    def fail_or_retry(self, reason):
        if self.retry_count < 1:
            self.retry_count += 1
            self.phase = 'RETRY'
            self.phase_started = time.monotonic()
            self.say(reason + ' Replanning once in 3 seconds.')
        else:
            self.phase = 'BETWEEN'
            self.phase_started = time.monotonic()
            self.say(reason + ' Skipping to the next stage.')

    def insert_dynamic_obstacle(self, name, x, y, size_x, size_y):
        self.say(f'Obstacle event: inserting one obstacle on this road ({name}); Nav2 must replan around it.')
        if self.active_obstacle and self.delete_client.service_is_ready():
            request = DeleteEntity.Request()
            request.name = self.active_obstacle
            future = self.delete_client.call_async(request)
            future.add_done_callback(
                lambda _, n=name, px=x, py=y, sx=size_x, sy=size_y:
                self.spawn_dynamic_obstacle(n, px, py, sx, sy)
            )
        else:
            self.spawn_dynamic_obstacle(name, x, y, size_x, size_y)

    def spawn_dynamic_obstacle(self, name, x, y, size_x, size_y):
        if not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.say('Gazebo spawn service unavailable; continuing with the static obstacles.')
            return
        request = SpawnEntity.Request()
        request.name = name
        request.robot_namespace = ''
        request.reference_frame = 'world'
        request.initial_pose.position.x = x
        request.initial_pose.position.y = y
        request.initial_pose.position.z = 0.5
        request.xml = f'''<?xml version="1.0"?>
<sdf version="1.6"><model name="{name}"><static>true</static>
<link name="link"><collision name="collision"><geometry><box><size>{size_x} {size_y} 1.0</size></box></geometry></collision>
<visual name="visual"><geometry><box><size>{size_x} {size_y} 1.0</size></box></geometry>
<material><ambient>0.95 0.08 0.08 1</ambient><diffuse>0.95 0.08 0.08 1</diffuse></material></visual>
</link></model></sdf>'''
        future = self.spawn_client.call_async(request)
        future.add_done_callback(lambda result, n=name: self.spawn_result(result, n))

    def spawn_result(self, future, name):
        response = future.result()
        if response.success:
            self.active_obstacle = name
            self.say(f'{name} inserted. Laser/costmap update should bend the path around it.')
        else:
            self.say('Obstacle insertion reported: ' + response.status_message)

    def clear_active_obstacle(self):
        if not self.active_obstacle or not self.delete_client.service_is_ready():
            self.active_obstacle = None
            return
        request = DeleteEntity.Request()
        request.name = self.active_obstacle
        self.delete_client.call_async(request)
        self.say(f'Cleared {self.active_obstacle} after reaching the stage goal.')
        self.active_obstacle = None

    def publish_markers(self):
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        clear = Marker()
        clear.header.frame_id = 'map'
        clear.header.stamp = stamp
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for index, (name, x, y, _) in enumerate(self.targets):
            sphere = Marker()
            sphere.header.frame_id = 'map'
            sphere.header.stamp = stamp
            sphere.ns = 'staged_goals'
            sphere.id = index * 2
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = x
            sphere.pose.position.y = y
            sphere.pose.position.z = 0.18
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.28
            if index < self.stage_index:
                sphere.color.r, sphere.color.g, sphere.color.b = 0.35, 0.35, 0.35
            elif index == self.stage_index:
                sphere.color.r, sphere.color.g, sphere.color.b = 0.05, 1.0, 0.15
            else:
                sphere.color.r, sphere.color.g, sphere.color.b = 0.10, 0.45, 1.0
            sphere.color.a = 1.0
            markers.markers.append(sphere)
        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = StagedNavigationDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
