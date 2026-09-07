import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
import tf2_ros

from .map_cache import MapCacheWorker
from .serialization import serialize_map, serialize_pose, serialize_scan
from .state import GatewayState


class GatewayNode(Node):
    def __init__(self):
        super().__init__("web_bridge")
        self.declare_parameter("map_topic", "map")
        self.declare_parameter("scan_topic", "scan")
        self.declare_parameter("odom_topic", "odom")
        self.declare_parameter("tf_topic", "tf")
        self.declare_parameter("tf_static_topic", "tf_static")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("max_map_cells", 16000000)
        self.declare_parameter("max_scan_samples", 10000)
        self.declare_parameter("pose_rate_hz", 10.0)
        self.declare_parameter("scan_stale_sec", 2.0)
        self.declare_parameter("odom_stale_sec", 2.0)
        self.declare_parameter("pose_stale_sec", 1.0)
        self.declare_parameter("map_stale_sec", 0.0)
        self.declare_parameter("http_host", "127.0.0.1")
        self.declare_parameter("http_port", 8080)
        from .models import GatewayConfig
        self.config = GatewayConfig(**{name: self.get_parameter(name).value for name in (
            "map_topic", "scan_topic", "odom_topic", "tf_topic", "tf_static_topic",
            "map_frame", "base_frame", "max_map_cells", "max_scan_samples", "pose_rate_hz",
            "scan_stale_sec", "odom_stale_sec", "pose_stale_sec", "map_stale_sec")})
        if not self.config.map_frame or not self.config.base_frame:
            raise ValueError("map_frame and base_frame are required")
        self.state = GatewayState(self.config)
        self.tf_buffer = tf2_ros.Buffer()
        self.map_worker = MapCacheWorker(
            self.state,
            lambda msg: serialize_map(msg, self.config.max_map_cells),
        )
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        scan_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(OccupancyGrid, self.config.map_topic, self.map_worker.submit, map_qos)
        self.create_subscription(LaserScan, self.config.scan_topic, self._scan_callback, scan_qos)
        self.create_subscription(Odometry, self.config.odom_topic, self._odom_callback, scan_qos)
        self.create_subscription(TFMessage, self.config.tf_topic, self._tf_callback, 100)
        static_qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(TFMessage, self.config.tf_static_topic, self._tf_static_callback, static_qos)
        self.create_timer(1.0 / self.config.pose_rate_hz, self._pose_timer)
        self._last_tf_error = None

    def _scan_callback(self, message):
        try:
            payload = serialize_scan(message, self.config.max_scan_samples)
            self.state.set_data("scan", payload, payload["stamp_ns"], payload["frame_id"])
        except (TypeError, ValueError, AttributeError) as exc:
            self.state.status.dropped_scans += 1
            self.state.add_error("scan: " + str(exc))

    def _odom_callback(self, message):
        linear = message.twist.twist.linear
        angular = message.twist.twist.angular
        payload = {
            "linear": {"x": float(linear.x), "y": float(linear.y), "z": float(linear.z)},
            "angular": {"x": float(angular.x), "y": float(angular.y), "z": float(angular.z)},
        }
        source_stamp = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )
        self.state.set_data("odom", payload, source_stamp, message.child_frame_id)

    def _tf_callback(self, message):
        for transform in message.transforms:
            self.tf_buffer.set_transform(transform, "web_bridge")

    def _tf_static_callback(self, message):
        for transform in message.transforms:
            self.tf_buffer.set_transform_static(transform, "web_bridge")

    def _pose_timer(self):
        try:
            transform = self.tf_buffer.lookup_transform(self.config.map_frame, self.config.base_frame, rclpy.time.Time())
            stamp = transform.header.stamp
            source_stamp = int(stamp.sec) * 1000000000 + int(stamp.nanosec)
            payload = serialize_pose(transform, self.config.map_frame, self.config.base_frame, source_stamp)
            self.state.set_data("pose", payload, source_stamp, self.config.map_frame)
            self._last_tf_error = None
        except Exception as exc:  # tf2 raises several exception subclasses across Humble patches
            # Deduplicate identical consecutive errors so a frozen TF does not
            # grow the error buffer at 10 Hz forever.
            message = str(exc)
            if message != self._last_tf_error:
                self.state.add_error("tf: " + message)
                self._last_tf_error = message

    def close(self):
        self.map_worker.close()
