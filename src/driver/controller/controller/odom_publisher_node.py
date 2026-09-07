#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import math
import time
import rclpy
import signal
import tf2_ros
import threading
from rclpy.node import Node
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
from controller import ackermann, mecanum
from ros_robot_controller_msgs.msg import MotorsState, BusServoState, SetBusServoState 
from geometry_msgs.msg import Pose2D, Pose, Twist, PoseWithCovarianceStamped, TransformStamped

ODOM_POSE_COVARIANCE = list(map(float, 
                        [1e-3, 0, 0, 0, 0, 0, 
                        0, 1e-3, 0, 0, 0, 0,
                        0, 0, 1e6, 0, 0, 0,
                        0, 0, 0, 1e6, 0, 0,
                        0, 0, 0, 0, 1e6, 0,
                        0, 0, 0, 0, 0, 1e3]))

ODOM_POSE_COVARIANCE_STOP = list(map(float, 
                            [1e-9, 0, 0, 0, 0, 0, 
                             0, 1e-3, 1e-9, 0, 0, 0,
                             0, 0, 1e6, 0, 0, 0,
                             0, 0, 0, 1e6, 0, 0,
                             0, 0, 0, 0, 1e6, 0,
                             0, 0, 0, 0, 0, 1e-9]))

ODOM_TWIST_COVARIANCE = list(map(float, 
                        [1e-3, 0, 0, 0, 0, 0, 
                         0, 1e-3, 0, 0, 0, 0,
                         0, 0, 1e6, 0, 0, 0,
                         0, 0, 0, 1e6, 0, 0,
                         0, 0, 0, 0, 1e6, 0,
                         0, 0, 0, 0, 0, 1e3]))

ODOM_TWIST_COVARIANCE_STOP = list(map(float, 
                            [1e-9, 0, 0, 0, 0, 0, 
                              0, 1e-3, 1e-9, 0, 0, 0,
                              0, 0, 1e6, 0, 0, 0,
                              0, 0, 0, 1e6, 0, 0,
                              0, 0, 0, 0, 1e6, 0,
                              0, 0, 0, 0, 0, 1e-9]))

def rpy2qua(roll, pitch, yaw):
    cy = math.cos(yaw*0.5)
    sy = math.sin(yaw*0.5)
    cp = math.cos(pitch*0.5)
    sp = math.sin(pitch*0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    
    q = Pose()
    q.orientation.w = cy * cp * cr + sy * sp * sr
    q.orientation.x = cy * cp * sr - sy * sp * cr
    q.orientation.y = sy * cp * sr + cy * sp * cr
    q.orientation.z = sy * cp * cr - cy * sp * sr
    return q.orientation

def qua2rpy(x, y, z, w):
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(2 * (w * y - x * z))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
  
    return roll, pitch, yaw

class Controller(Node):
    
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)

        self.x = 0.0
        self.y = 0.0
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0
        self.pose_yaw = 0
        self.last_time = None
        self.current_time = None
        signal.signal(signal.SIGINT, self.shutdown)

        # Command watchdog state (monotonic wall clock, not sim time)
        self.last_cmd_at = time.time()
        self.cmd_lost = False
        self.estop = False
        self.last_stop_sent = 0.0

        self.ackermann = ackermann.AckermannChassis(wheelbase=0.216, track_width=0.195, wheel_diameter=0.097)
        self.mecanum = mecanum.MecanumChassis(wheelbase=0.216, track_width=0.195, wheel_diameter=0.097)

        # 声明参数
        self.declare_parameter('pub_odom_topic', True)
        self.declare_parameter('base_frame_id', 'base_footprint')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('linear_correction_factor', 1.00)
        self.declare_parameter('angular_correction_factor', 1.00)
        self.declare_parameter('machine_type', os.environ['MACHINE_TYPE'])
        self.declare_parameter('cmd_timeout_sec', 0.3)
        self.declare_parameter('max_linear_x', 0.2)
        self.declare_parameter('max_linear_y', 0.2)
        self.declare_parameter('max_angular_z', 0.5)
        
        self.pub_odom_topic = self.get_parameter('pub_odom_topic').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        
        self.linear_factor = self.get_parameter('linear_correction_factor').value
        self.angular_factor = self.get_parameter('angular_correction_factor').value
        self.machine_type = self.get_parameter('machine_type').value
        self.cmd_timeout_sec = float(self.get_parameter('cmd_timeout_sec').value)
        self.max_linear_x = float(self.get_parameter('max_linear_x').value)
        self.max_linear_y = float(self.get_parameter('max_linear_y').value)
        self.max_angular_z = float(self.get_parameter('max_angular_z').value)

        self.clock = self.get_clock() 
        if self.pub_odom_topic:
            # self.odom_broadcaster = tf2_ros.TransformBroadcaster(self)  # 定义TF变换广播者
            # self.odom_trans = TransformStamped()
            # self.odom_trans.header.frame_id = self.odom_frame_id
            # self.odom_trans.child_frame_id = self.base_frame_id
            
            self.odom = Odometry()
            self.odom.header.frame_id = self.odom_frame_id
            self.odom.child_frame_id = self.base_frame_id
            
            self.odom.pose.covariance = ODOM_POSE_COVARIANCE
            self.odom.twist.covariance = ODOM_TWIST_COVARIANCE
            
            self.odom_pub = self.create_publisher(Odometry, 'odom_raw', 1)
            self.dt = 1.0/50.0

            threading.Thread(target=self.cal_odom_fun, daemon=True).start()
        self.get_logger().info('\033[1;32m%f %f\033[0m' % (self.linear_factor, self.angular_factor))
        self.motor_pub = self.create_publisher(MotorsState, 'ros_robot_controller/set_motor', 1)
        self.servo_state_pub = self.create_publisher(SetBusServoState, 'ros_robot_controller/bus_servo/set_state', 1)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'set_pose', 1)
        self.create_subscription(Pose2D, 'set_odom', self.set_odom, 1)
        self.create_subscription(Twist, 'controller/cmd_vel', self.cmd_vel_callback, 1)
        self.create_subscription(Twist, 'cmd_vel', self.app_cmd_vel_callback, 1)
        self.create_service(Trigger, 'controller/load_calibrate_param', self.load_calibrate_param)
        self.create_service(Trigger, '~/estop', self.estop_callback)
        self.create_service(Trigger, '~/estop_release', self.estop_release_callback)

        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def get_node_state(self, request, response):
        response.success = True
        return response

    def shutdown(self, signum, frame):
        self.get_logger().info('\033[1;32m%s\033[0m' % 'shutdown')
        self.estop = True
        self._publish_stop()
        rclpy.shutdown()

    def load_calibrate_param(self, request, response):
        self.linear_factor = self.get_parameter('linear_correction_factor').value or 1.00
        self.angular_factor = self.get_parameter('angular_correction_factor').value or 1.00
        self.get_logger().info('\033[1;32m%s\033[0m' % 'load_calibrate_param')

        response.success = True
        return response

    def estop_callback(self, request, response):
        self.get_logger().warn('\033[1;31mE-STOP engaged\033[0m')
        self.estop = True
        self._publish_stop()
        response.success = True
        return response

    def estop_release_callback(self, request, response):
        self.get_logger().warn('\033[1;32mE-STOP released\033[0m')
        self.estop = False
        self.cmd_lost = False
        response.success = True
        return response

    def _stop_motor_data(self):
        if self.machine_type == 'JetRover_Acker':
            _, motor_data = self.ackermann.set_velocity(0.0, 0.0)
        else:
            return self.mecanum.set_velocity(0.0, 0.0, 0.0)
        msg = MotorsState()
        msg.data = motor_data
        return msg

    def _publish_stop(self):
        try:
            self.motor_pub.publish(self._stop_motor_data())
        except Exception as exc:
            self.get_logger().error('stop publish failed: %s' % str(exc))
        self.last_stop_sent = time.time()

    def set_odom(self, msg):
        self.odom = Odometry()
        self.odom.header.frame_id = self.odom_frame_id
        self.odom.child_frame_id = self.base_frame_id

        self.odom.pose.covariance = ODOM_POSE_COVARIANCE
        self.odom.twist.covariance = ODOM_TWIST_COVARIANCE
        self.odom.pose.pose.position.x = msg.x
        self.odom.pose.pose.position.y = msg.y
        self.x = msg.x
        self.y = msg.y
        self.pose_yaw = msg.theta
        self.last_time = time.time()
        self.odom.pose.pose.orientation = rpy2qua(0, 0, self.pose_yaw)

        self.linear_x = 0
        self.linear_y = 0
        self.angular_z = 0

        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = self.odom_frame_id
        pose.header.stamp = self.clock.now().to_msg()
        pose.pose.pose = self.odom.pose.pose
        pose.pose.covariance = ODOM_POSE_COVARIANCE
        self.pose_pub.publish(pose)

    def _clamp(self, msg):
        msg.linear.x = max(-self.max_linear_x, min(self.max_linear_x, msg.linear.x))
        msg.linear.y = max(-self.max_linear_y, min(self.max_linear_y, msg.linear.y))
        msg.angular.z = max(-self.max_angular_z, min(self.max_angular_z, msg.angular.z))
        return msg

    def app_cmd_vel_callback(self, msg):
        self.cmd_vel_callback(self._clamp(msg))

    def cmd_vel_callback(self, msg):
        # Both velocity entry points (Nav2/vel smoother 'cmd_vel' and upstream
        # 'controller/cmd_vel') share the same clamp + freshness bookkeeping.
        self._clamp(msg)
        if self.estop:
            return
        self.last_cmd_at = time.time()
        self.cmd_lost = False
        if self.machine_type == 'JetRover_Mecanum':
            self.linear_x = msg.linear.x
            self.linear_y = msg.linear.y
        else:
            self.linear_x = msg.linear.x
            self.linear_y = msg.linear.y
            if abs(msg.linear.y) > 1e-8:
                self.linear_x = 0.0
            else:
                self.linear_x = msg.linear.x
            self.linear_y = 0.0

        if self.machine_type != 'JetRover_Acker':
            self.angular_z = msg.angular.z
            speeds = self.mecanum.set_velocity(self.linear_x, self.linear_y, self.angular_z)
            self.motor_pub.publish(speeds)
        elif self.machine_type == 'JetRover_Acker':
            if msg.angular.z != 0:
                r = self.linear_x / msg.angular.z
                if r == 0:
                    self.angular_z = 0.0
                else:
                    self.angular_z = msg.angular.z
            else:
                self.angular_z = 0.0
            servo_state = BusServoState()
            servo_state.present_id = [1, 9]
            speeds = self.ackermann.set_velocity(self.linear_x, self.angular_z)
            motors = MotorsState()
            motors.data = speeds[1]
            self.motor_pub.publish(motors)

            if speeds[0] is not None:
                angle_pulse = int(round(min(max(speeds[0], 0.0), 1000.0)))
                servo_state.position = [1, angle_pulse]
                data = SetBusServoState()
                data.state = [servo_state]
                data.duration = 0.02
                self.servo_state_pub.publish(data)

    def cal_odom_fun(self):
        while True:
            self.current_time = time.time()
            if self.last_time is None:
                self.dt = 0.0
            else:
                # 计算时间间隔
                self.dt = self.current_time - self.last_time
            self.odom.header.stamp = self.clock.now().to_msg()

            # Command watchdog: if no fresh command arrives within cmd_timeout_sec
            # (or an e-stop is latched), stop integrating the old speed and keep
            # sending zero motor commands so the last motion command is revoked.
            stale = (self.cmd_timeout_sec > 0.0 and
                     self.current_time - self.last_cmd_at > self.cmd_timeout_sec)
            if self.estop or stale:
                if not self.cmd_lost:
                    self.cmd_lost = True
                    self.get_logger().warn('\033[1;33mcmd_vel lost/e-stop -> STOP\033[0m')
                self.linear_x = 0.0
                self.linear_y = 0.0
                self.angular_z = 0.0
                if self.current_time - self.last_stop_sent > 1.0:
                    self._publish_stop()

            self.x += math.cos(self.pose_yaw)*self.linear_x*self.dt - math.sin(self.pose_yaw)*self.linear_y*self.dt
            self.y += math.sin(self.pose_yaw)*self.linear_x*self.dt + math.cos(self.pose_yaw)*self.linear_y*self.dt

            self.odom.pose.pose.position.x = self.linear_factor*self.x
            self.odom.pose.pose.position.y = self.linear_factor*self.y
            self.odom.pose.pose.position.z = 0.0

            self.pose_yaw += self.angular_factor*self.angular_z*self.dt

            self.odom.pose.pose.orientation = rpy2qua(0.0, 0.0, self.pose_yaw)
            self.odom.twist.twist.linear.x = self.linear_x
            self.odom.twist.twist.linear.y = self.linear_y
            self.odom.twist.twist.angular.z = self.angular_z

            # self.odom_trans.header.stamp = self.clock.now().to_msg()
            # self.odom_trans.transform.translation.x = self.odom.pose.pose.position.x
            # self.odom_trans.transform.translation.y = self.odom.pose.pose.position.y
            # self.odom_trans.transform.translation.z = 0.0
            # self.odom_trans.transform.rotation = self.odom.pose.pose.orientation

            # 如果velocity是零，说明编码器的误差会比较小，认为编码器数据更可靠
            # 如果velocity非零，考虑到运动中编码器可能带来的滑动误差，认为imu的数据更可靠
            if self.linear_x == 0 and self.linear_y == 0 and self.angular_z == 0:
                self.odom.pose.covariance = ODOM_POSE_COVARIANCE_STOP
                self.odom.twist.covariance = ODOM_TWIST_COVARIANCE_STOP
            else:
                self.odom.pose.covariance = ODOM_POSE_COVARIANCE
                self.odom.twist.covariance = ODOM_TWIST_COVARIANCE

            # self.odom_broadcaster.sendTransform(self.odom_trans)
            self.odom_pub.publish(self.odom)
            self.last_time = self.current_time
            time.sleep(0.02)

def main():
    node = Controller('odom_publisher')
    rclpy.spin(node)  # 循环等待ROS2退出

if __name__ == "__main__":
    main()
