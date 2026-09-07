#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2023/11/10
import threading
import time

from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.duration import Duration
from control_msgs.action import FollowJointTrajectory
from servo_controller_msgs.msg import ServoPosition

FollowJointTrajectoryResult = FollowJointTrajectory.Result


def _duration_seconds(value):
    # builtin_interfaces/Duration -> float seconds
    return float(value.sec) + float(value.nanosec) * 1e-9


class JointTrajectoryActionController(Node):
    def __init__(self, servo_manager, controller_namespace, controllers):
        super().__init__(controller_namespace)
        self.servo_manager = servo_manager

        self.joint_names = []
        self.joint_to_controller = {}
        for c in controllers:
            self.joint_names.append(c.joint_name)
            self.joint_to_controller[c.joint_name] = c
        self.num_joints = len(self.joint_names)

        ns = controller_namespace + '/joint_trajectory_action_node/constraints'
        self.goal_constraints = [-1] * self.num_joints
        self.trajectory_constraints = [-1] * self.num_joints

        self.feedback_msg = FollowJointTrajectory.Feedback()
        self.feedback_msg.joint_names = list(self.joint_names)
        self.feedback_msg.desired.positions = [0.0] * self.num_joints
        self.feedback_msg.desired.velocities = [0.0] * self.num_joints
        self.feedback_msg.desired.accelerations = [0.0] * self.num_joints
        self.feedback_msg.actual.positions = [0.0] * self.num_joints
        self.feedback_msg.actual.velocities = [0.0] * self.num_joints
        self.feedback_msg.error.positions = [0.0] * self.num_joints
        self.feedback_msg.error.velocities = [0.0] * self.num_joints

        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            controller_namespace + '/follow_joint_trajectory',
            self.follow_trajectory_callback)

    def _joint_pulse(self, joint_name, position_rad):
        controller = self.joint_to_controller[joint_name]
        pulse = controller.pos_rad_to_pulse(float(position_rad))
        pulse = int(round(min(max(pulse, 0.0), 1000.0)))
        return controller.servo_id, pulse

    def follow_trajectory_callback(self, goal_handle):
        # Run the trajectory on a dedicated worker thread so the action server
        # stays responsive to cancellation while the motion is being executed.
        threading.Thread(target=self._execute, args=(goal_handle,), daemon=True).start()
        return FollowJointTrajectory.Result()

    def _execute(self, goal_handle):
        try:
            return self._run_trajectory(goal_handle)
        except Exception as exc:
            self.get_logger().error('trajectory execution failed: %s' % str(exc))
            goal_handle.abort()
            return FollowJointTrajectory.Result()

    def _run_trajectory(self, goal_handle):
        traj = goal_handle.request.trajectory
        points = list(traj.points)
        if len(points) == 0:
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        # Validate that every controlled joint exists in the goal trajectory.
        try:
            for joint in self.joint_names:
                traj.joint_names.index(joint)
        except ValueError:
            self.get_logger().error('trajectory joint_names does not cover the controller joints')
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        # Per-segment duration in seconds from the goal time_from_start stamps.
        base = [_duration_seconds(p.time_from_start) for p in points]
        durations = [base[0]] + [max(0.0, base[i] - base[i - 1]) for i in range(1, len(points))]

        def build_commands(point):
            commands = []
            for joint in self.joint_names:
                idx = traj.joint_names.index(joint)
                servo_id, pulse = self._joint_pulse(joint, point.positions[idx])
                msg = ServoPosition()
                msg.id = servo_id
                msg.position = pulse
                commands.append(msg)
            return commands

        for i, point in enumerate(points):
            if goal_handle.is_cancel_requested:
                self.get_logger().warn('trajectory cancelled')
                goal_handle.canceled()
                return FollowJointTrajectory.Result()
            commands = build_commands(point)
            # Command the servo group for the whole remaining segment and wait
            # until the next point is due. A fresh command is sent at every point
            # so the segment is not skipped.
            duration = max(0.02, durations[i])
            self.servo_manager.set_position(duration, commands)
            time.sleep(duration)

        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result
