#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2023/11/10
from rclpy.node import Node
from ros_robot_controller_msgs.srv import GetBusServoState
from ros_robot_controller_msgs.msg import GetBusServoCmd, ServoPosition, ServosPosition

class ServoState:
    def __init__(self, name=''):
        self.name = name
        self.position = 500

class ServoManager(Node):
    def __init__(self, connected_ids=[]):
        super().__init__('servo_manager')
        self.servos = {}
        self.connected_ids = connected_ids
        for i in connected_ids:
            self.servos[i] = ServoState(connected_ids[i])
        self.servo_position_pub = self.create_publisher(ServosPosition, 'ros_robot_controller/bus_servo/set_position', 1)
        self.client = self.create_client(GetBusServoState, 'ros_robot_controller/bus_servo/get_state')
        self.client.wait_for_service()
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def connect(self):
        # Check that every configured servo answers a read-id query. A missing
        # servo is reported but does not abort startup so the rest of the node
        # can still bring itself up.
        for servo_id in self.servos:
            try:
                if not self.get_servo_id(int(servo_id)):
                    self.get_logger().error('Bus servo ID %s not found.' % servo_id)
            except Exception as exc:
                self.get_logger().error('connect check for servo %s failed: %s' % (servo_id, exc))

    def get_position(self):
        return self.servos

    def sample_positions(self):
        # Read measured positions back from the controller board and refresh the
        # cached values. Command echo (set_position) is only a fallback when the
        # read-back fails or is not yet available.
        if not self.servos:
            return False
        request = GetBusServoState.Request()
        for servo_id in self.servos:
            cmd = GetBusServoCmd()
            cmd.id = int(servo_id)
            cmd.get_position = 1
            request.cmd.append(cmd)
        try:
            response = self.client.call(request)
        except Exception as exc:
            self.get_logger().warn('servo state read-back failed: %s' % str(exc))
            return False
        if response is None or not getattr(response, 'success', False):
            return False
        for servo_id, state in zip(list(self.servos.keys()), response.state):
            position = getattr(state, 'position', None)
            if not position:
                continue
            try:
                self.servos[servo_id].position = int(position[0])
            except (TypeError, ValueError, IndexError):
                continue
        return True

    def get_servo_id(self, servo_id):
        request = GetBusServoState.Request()
        cmd = GetBusServoCmd()
        cmd.id = servo_id
        cmd.get_id = 1
        request.cmd.append(cmd)
        for i in range(0, 20):
            response = self.client.call(request)
            if response is None or not response.success:
                continue
            for state in response.state:
                if state.present_id and servo_id in list(state.present_id):
                    return True
        return False

    def set_position(self, duration, position):
        duration = 0.02 if duration < 0.02 else 30 if duration > 30 else duration
        msg = ServosPosition()
        msg.duration = float(duration)
        for i in position:
            position = int(i.position)
            position = 0 if position < 0 else 1000 if position > 1000 else position
            self.servos[str(i.id)].position = position  # 记录发送的位置
            servo_msg = ServoPosition()
            servo_msg.id = i.id
            servo_msg.position = position
            msg.position.append(servo_msg)
        self.servo_position_pub.publish(msg)
