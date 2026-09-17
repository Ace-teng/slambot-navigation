"""Single-node autonomous frontier exploration for the navigation package."""

from concurrent.futures import CancelledError, ThreadPoolExecutor
from enum import Enum
import math
import threading
from typing import Dict, List, Optional, Sequence, Set, Tuple

from geometry_msgs.msg import Point, PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from slam_toolbox.srv import SaveMap
from std_msgs.msg import String
from std_srvs.srv import Trigger
import tf2_ros
from tf2_ros import TransformException
from visualization_msgs.msg import Marker, MarkerArray

from .exploration_core import (
    Cell,
    CompletionMonitor,
    GridMap,
    astar,
    bounded_candidate_batch,
    candidate_goals,
    detect_frontiers,
    inflate_obstacles,
    information_gain,
    initial_scan_velocity,
    known_area,
    normalize_angle,
    obstacle_clearance_risk,
    path_cost,
    path_is_safe,
    reachable_free,
    round_path_corners,
    scan_directional_minima,
    scan_sector_minima,
    simplify_path,
)


class State(Enum):
    """Runtime states for the exploration finite-state machine."""

    IDLE = 'IDLE'
    WAIT_SENSOR = 'WAIT_FOR_SENSOR'
    WAIT_TF = 'WAIT_FOR_TF'
    WAIT_MAP = 'WAIT_FOR_MAP'
    INITIAL_SCAN = 'INITIAL_SCAN'
    DETECT = 'DETECT_FRONTIER'
    SELECT = 'SELECT_GOAL'
    PLAN = 'PLAN_PATH'
    FOLLOW = 'FOLLOW_PATH'
    REPLAN = 'REPLAN'
    RECOVERY = 'RECOVERY'
    VERIFY = 'VERIFY_COMPLETE'
    SAVE = 'SAVE_MAP'
    COMPLETE = 'COMPLETED'
    ERROR = 'ERROR'


def yaw_from_quaternion(quaternion) -> float:
    return math.atan2(
        2.0 * (
            quaternion.w * quaternion.z
            + quaternion.x * quaternion.y
        ),
        1.0 - 2.0 * (
            quaternion.y * quaternion.y
            + quaternion.z * quaternion.z
        ),
    )


class AutonomousExplorationNode(Node):
    """Drive frontier exploration while an existing SLAM node builds a map."""

    def __init__(self) -> None:
        super().__init__('autonomous_exploration')
        self._declare_parameters()
        self._read_parameters()

        self._lock = threading.RLock()
        self._destroying = False
        self.state = State.IDLE
        self.paused = False
        self.stop_requested = False

        self.scan: Optional[LaserScan] = None
        self.odom: Optional[Odometry] = None
        self.grid: Optional[GridMap] = None
        self.odom_frame_warning_issued = False
        self.last_map_at = 0.0
        self.last_scan_at = 0.0
        self.last_odom_at = 0.0
        self.last_tf_at = 0.0

        self.path: List[Cell] = []
        self.path_index = 0
        self.path_signature = None
        self.goal: Optional[Cell] = None
        self.manual_goal_world: Optional[Tuple[float, float]] = None
        self.active_goal_world: Optional[Tuple[float, float]] = None
        self.dfs_heading: Optional[float] = None
        self.dfs_stack_world: List[Tuple[float, float]] = []
        self.goal_started = 0.0
        self.last_progress_at = 0.0
        self.last_progress_pose: Optional[
            Tuple[float, float, float]
        ] = None
        self.blocked_since: Optional[float] = None

        self.blacklist: Dict[Cell, float] = {}
        self.failure_counts: Dict[Cell, int] = {}
        self.completion = CompletionMonitor(
            self.no_frontier_confirm_count,
            self.map_stable_duration,
        )

        self.initial_scan_started = 0.0
        self.initial_scan_completed = False
        self.rotation_accum = 0.0
        self.last_scan_yaw: Optional[float] = None
        self.recovery_started = 0.0
        self.recovery_direction = 1.0
        self.recovery_can_reverse = False
        self.verify_started = 0.0
        self.last_frontier_at = 0.0
        self.last_command = (0.0, 0.0)

        self.plan_pool = ThreadPoolExecutor(
            max_workers=1,
            thread_name_prefix='frontier-plan',
        )
        self.plan_future = None
        self.planning = False
        self.planning_started = 0.0
        self.planning_generation = 0

        self.save_future = None
        self.save_started = 0.0
        self.save_generation = 0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self,
        )

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self._map_cb,
            map_qos,
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self._scan_cb,
            qos_profile_sensor_data,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self._odom_cb,
            qos_profile_sensor_data,
        )
        self.goal_sub = self.create_subscription(
            PoseStamped,
            self.goal_topic,
            self._goal_cb,
            10,
        )

        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.path_pub = self.create_publisher(Path, '~/planned_path', 10)
        self.goal_pub = self.create_publisher(
            PoseStamped,
            '~/current_goal',
            10,
        )
        self.frontier_pub = self.create_publisher(
            MarkerArray,
            '~/frontiers',
            10,
        )
        self.status_pub = self.create_publisher(String, '~/status', 10)

        self.create_service(Trigger, '~/start', self._start)
        self.create_service(Trigger, '~/pause', self._pause)
        self.create_service(Trigger, '~/resume', self._resume)
        self.create_service(Trigger, '~/stop', self._stop)

        self.save_client = None
        if self.map_save_service:
            self.save_client = self.create_client(
                SaveMap,
                self.map_save_service,
            )

        self.timer = self.create_timer(
            1.0 / max(self.control_frequency, 1.0),
            self._control_tick,
        )
        self._set_state(
            State.WAIT_SENSOR if self.auto_start else State.IDLE
        )

    def _declare_parameters(self) -> None:
        defaults = {
            'map_topic': 'map',
            'scan_topic': 'scan',
            'odom_topic': 'odom',
            'cmd_topic': 'controller/cmd_vel',
            'goal_topic': 'goal_pose',
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'control_frequency': 15.0,
            'frontier_frequency': 1.0,
            'occupied_threshold': 65,
            'robot_radius': 0.18,
            'safety_margin': 0.10,
            'min_frontier_size': 5,
            'frontier_goal_offset': 0.30,
            'max_candidates': 12,
            'exploration_strategy': 'dfs',
            'dfs_gain_scale': 0.25,
            'dfs_heading_weight': 35.0,
            'dfs_depth_weight': 12.0,
            'dfs_depth_cap': 5.0,
            'dfs_revisit_radius': 0.80,
            'dfs_revisit_penalty': 60.0,
            'dfs_branch_switch_angle': 1.40,
            'dfs_stack_limit': 64,
            'max_linear_speed': 0.20,
            'min_linear_speed': 0.04,
            'max_angular_speed': 0.50,
            'lookahead': 0.45,
            'lookahead_max': 0.80,
            'lookahead_speed_gain': 1.50,
            'goal_tolerance': 0.20,
            'min_goal_distance': 0.80,
            'path_deviation_tolerance': 0.60,
            'progress_distance': 0.08,
            'progress_angle': 0.15,
            'rotate_in_place_angle': 1.20,
            'corner_rounding_fraction': 0.35,
            'slowdown_distance': 0.60,
            'emergency_stop_distance': 0.28,
            'side_clearance_distance': 0.34,
            'side_hard_stop_distance': 0.20,
            'side_min_turn_scale': 0.25,
            'sensor_timeout': 1.0,
            'odom_timeout': 1.0,
            'tf_timeout': 1.0,
            'map_timeout': 5.0,
            'goal_timeout': 120.0,
            'blocked_timeout': 5.0,
            'stuck_timeout': 8.0,
            'planning_timeout': 6.0,
            'recovery_duration': 3.0,
            'recovery_backup_duration': 0.8,
            'recovery_linear_speed': 0.06,
            'recovery_angular_speed': 0.35,
            'recovery_backup_clearance': 0.35,
            'initial_scan_enabled': True,
            'initial_scan_speed': 0.25,
            'initial_scan_duration': 30.0,
            'information_radius_cells': 8,
            'risk_radius_cells': 4,
            'w_gain': 1.0,
            'w_path': 0.5,
            'w_risk': 0.3,
            'w_turn': 0.1,
            'w_failure': 2.0,
            'failure_blacklist_duration': 60.0,
            'map_stable_duration': 10.0,
            'no_frontier_confirm_count': 3,
            'auto_start': True,
            'accept_external_goals': False,
            'auto_save_map': True,
            'map_save_service': 'slam_toolbox/save_map',
            'map_save_name': 'exploration_map',
            'map_save_timeout': 10.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _read_parameters(self) -> None:
        string_parameters = (
            'map_topic',
            'scan_topic',
            'odom_topic',
            'cmd_topic',
            'goal_topic',
            'map_frame',
            'odom_frame',
            'base_frame',
            'map_save_service',
            'map_save_name',
            'exploration_strategy',
        )
        float_parameters = (
            'control_frequency',
            'frontier_frequency',
            'robot_radius',
            'safety_margin',
            'frontier_goal_offset',
            'dfs_gain_scale',
            'dfs_heading_weight',
            'dfs_depth_weight',
            'dfs_depth_cap',
            'dfs_revisit_radius',
            'dfs_revisit_penalty',
            'dfs_branch_switch_angle',
            'max_linear_speed',
            'min_linear_speed',
            'max_angular_speed',
            'lookahead',
            'lookahead_max',
            'lookahead_speed_gain',
            'goal_tolerance',
            'min_goal_distance',
            'path_deviation_tolerance',
            'progress_distance',
            'progress_angle',
            'rotate_in_place_angle',
            'corner_rounding_fraction',
            'slowdown_distance',
            'emergency_stop_distance',
            'side_clearance_distance',
            'side_hard_stop_distance',
            'side_min_turn_scale',
            'sensor_timeout',
            'odom_timeout',
            'tf_timeout',
            'map_timeout',
            'goal_timeout',
            'blocked_timeout',
            'stuck_timeout',
            'planning_timeout',
            'recovery_duration',
            'recovery_backup_duration',
            'recovery_linear_speed',
            'recovery_angular_speed',
            'recovery_backup_clearance',
            'initial_scan_speed',
            'initial_scan_duration',
            'w_gain',
            'w_path',
            'w_risk',
            'w_turn',
            'w_failure',
            'failure_blacklist_duration',
            'map_stable_duration',
            'map_save_timeout',
        )
        int_parameters = (
            'occupied_threshold',
            'min_frontier_size',
            'max_candidates',
            'dfs_stack_limit',
            'information_radius_cells',
            'risk_radius_cells',
            'no_frontier_confirm_count',
        )
        bool_parameters = (
            'initial_scan_enabled',
            'auto_start',
            'accept_external_goals',
            'auto_save_map',
        )

        for name in string_parameters:
            setattr(self, name, str(self.get_parameter(name).value))
        for name in float_parameters:
            setattr(self, name, float(self.get_parameter(name).value))
        for name in int_parameters:
            setattr(self, name, int(self.get_parameter(name).value))
        for name in bool_parameters:
            setattr(self, name, bool(self.get_parameter(name).value))
        self._validate_parameters()

    def _validate_parameters(self) -> None:
        if not 1 <= self.occupied_threshold <= 100:
            raise ValueError('occupied_threshold must be between 1 and 100')
        if self.robot_radius < 0.0 or self.safety_margin < 0.0:
            raise ValueError(
                'robot radius and safety margin must be nonnegative'
            )
        if self.min_frontier_size < 1 or self.max_candidates < 1:
            raise ValueError(
                'frontier sizes and candidate count must be positive'
            )
        if self.exploration_strategy not in ('utility', 'dfs'):
            raise ValueError(
                'exploration_strategy must be either utility or dfs'
            )
        if (
            self.dfs_heading_weight < 0.0
            or not 0.0 <= self.dfs_gain_scale <= 1.0
            or self.dfs_depth_weight < 0.0
            or self.dfs_depth_cap <= 0.0
            or self.dfs_revisit_radius <= 0.0
            or self.dfs_revisit_penalty < 0.0
            or not 0.0 < self.dfs_branch_switch_angle <= math.pi
            or self.dfs_stack_limit < 1
        ):
            raise ValueError('DFS exploration parameters are invalid')
        if self.max_linear_speed <= 0.0 or self.max_angular_speed <= 0.0:
            raise ValueError('maximum velocities must be positive')
        if self.min_linear_speed < 0.0:
            raise ValueError('min_linear_speed must be nonnegative')
        if self.min_linear_speed > self.max_linear_speed:
            raise ValueError('min_linear_speed exceeds max_linear_speed')
        if not 0.0 < self.emergency_stop_distance < self.slowdown_distance:
            raise ValueError(
                'emergency_stop_distance must be positive and below '
                'slowdown_distance'
            )
        if self.side_clearance_distance <= 0.0:
            raise ValueError('side_clearance_distance must be positive')
        if not (
            0.0 < self.side_hard_stop_distance
            < self.side_clearance_distance
        ):
            raise ValueError(
                'side_hard_stop_distance must be positive and below '
                'side_clearance_distance'
            )
        if not 0.0 < self.side_min_turn_scale <= 1.0:
            raise ValueError('side_min_turn_scale must be in (0, 1]')
        if self.lookahead_max < self.lookahead or self.lookahead <= 0.0:
            raise ValueError(
                'lookahead must be positive and no greater than '
                'lookahead_max'
            )
        if self.lookahead_speed_gain < 0.0:
            raise ValueError('lookahead_speed_gain must be nonnegative')
        if not 0.0 < self.corner_rounding_fraction < 0.5:
            raise ValueError('corner_rounding_fraction must be in (0, 0.5)')
        if not (
            0.0 <= self.recovery_backup_duration
            < self.recovery_duration
        ):
            raise ValueError(
                'recovery_backup_duration must be nonnegative and below '
                'recovery_duration'
            )
        if (
            self.recovery_linear_speed <= 0.0
            or self.recovery_angular_speed <= 0.0
            or self.recovery_backup_clearance <= 0.0
        ):
            raise ValueError('recovery speeds and clearance must be positive')
        if self.min_goal_distance <= self.goal_tolerance:
            raise ValueError(
                'min_goal_distance must exceed goal_tolerance'
            )
        required_positive = {
            'control_frequency': self.control_frequency,
            'frontier_frequency': self.frontier_frequency,
            'goal_timeout': self.goal_timeout,
            'blocked_timeout': self.blocked_timeout,
            'stuck_timeout': self.stuck_timeout,
            'planning_timeout': self.planning_timeout,
            'recovery_duration': self.recovery_duration,
            'map_save_timeout': self.map_save_timeout,
            'progress_angle': self.progress_angle,
        }
        invalid = [
            name
            for name, value in required_positive.items()
            if value <= 0.0
        ]
        if invalid:
            names = ', '.join(invalid)
            raise ValueError(f'parameters must be positive: {names}')
        if self.no_frontier_confirm_count < 1:
            raise ValueError('no_frontier_confirm_count must be positive')
        if self.auto_save_map and (
            not self.map_save_service or not self.map_save_name
        ):
            raise ValueError(
                'automatic map saving requires service and map name'
            )

    def _map_cb(self, message: OccupancyGrid) -> None:
        if not message.data or message.info.width <= 0:
            return
        if message.info.height <= 0 or message.info.resolution <= 0.0:
            return

        new_grid = GridMap(
            message.info.width,
            message.info.height,
            message.info.resolution,
            message.info.origin.position.x,
            message.info.origin.position.y,
            list(message.data),
        )
        now = self._now()
        with self._lock:
            old_grid = self.grid
            old_signature = old_grid.signature if old_grid else None
            self.grid = new_grid
            self.last_map_at = now

            geometry_changed = (
                old_signature is not None
                and old_signature != new_grid.signature
            )
            if geometry_changed:
                # Failure records use grid-cell coordinates and cannot be
                # carried into a map whose origin or dimensions changed.
                self.blacklist.clear()
                self.failure_counts.clear()
            if geometry_changed and self.state in (
                State.SELECT,
                State.PLAN,
                State.FOLLOW,
            ):
                self._invalidate_planning()
                self.path = []
                self.path_index = 0
                self._stop_cmd()
                fixed_goal_world = (
                    self.manual_goal_world
                    if self.manual_goal_world is not None
                    else self.active_goal_world
                )
                if fixed_goal_world is not None:
                    projected_goal = new_grid.world_to_cell(
                        fixed_goal_world
                    )
                    if new_grid.in_bounds(projected_goal):
                        self.goal = projected_goal
                        self.get_logger().info(
                            'map geometry changed; keeping the world-frame '
                            'goal and replanning on the updated map'
                        )
                        self._set_state(State.PLAN)
                        return
                    self.get_logger().warning(
                        'preserved goal is outside the updated map; '
                        'selecting a new frontier'
                    )
                    self.active_goal_world = None
                    self.manual_goal_world = None
                    self.goal = None
                    self._set_state(State.DETECT)
                else:
                    # A frontier-selection job had not committed a goal yet.
                    # Restart detection against the newest map geometry.
                    self.goal = None
                    self._set_state(State.DETECT)

    def _reproject_active_path(
        self,
        old_grid: GridMap,
        new_grid: GridMap,
    ) -> bool:
        """Compatibility hook: preserve the goal and request a fresh path."""
        del old_grid
        if self.active_goal_world is None:
            return False
        projected_goal = new_grid.world_to_cell(self.active_goal_world)
        if not new_grid.in_bounds(projected_goal):
            return False
        self.goal = projected_goal
        self.path = []
        self.path_index = 0
        self.path_signature = None
        return True

    def _scan_cb(self, message: LaserScan) -> None:
        self.scan = message
        self.last_scan_at = self._now()

    def _odom_cb(self, message: Odometry) -> None:
        if (
            message.header.frame_id
            and message.header.frame_id != self.odom_frame
            and not self.odom_frame_warning_issued
        ):
            self.get_logger().warning(
                f'odometry frame {message.header.frame_id} does not match '
                f'configured frame {self.odom_frame}'
            )
            self.odom_frame_warning_issued = True
        self.odom = message
        self.last_odom_at = self._now()

    def _goal_cb(self, message: PoseStamped) -> None:
        if not self.accept_external_goals:
            self.get_logger().warning(
                'external goal ignored; accept_external_goals is false'
            )
            return
        if message.header.frame_id not in ('', self.map_frame):
            self.get_logger().error(
                f'external goal frame {message.header.frame_id} '
                f'does not match {self.map_frame}'
            )
            return
        with self._lock:
            self.manual_goal_world = (
                message.pose.position.x,
                message.pose.position.y,
            )
            self.active_goal_world = None
            self.goal = None
            self.path = []
            self.path_index = 0
            self._invalidate_planning()
            self._stop_cmd()
            self._set_state(State.PLAN)

    def _pose(self) -> Optional[Tuple[float, float, float]]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(),
            )
        except TransformException:
            return None
        self.last_tf_at = self._now()
        return (
            transform.transform.translation.x,
            transform.transform.translation.y,
            yaw_from_quaternion(transform.transform.rotation),
        )

    def _set_state(self, state: State) -> None:
        if self.state != state:
            self.get_logger().info(
                f'exploration state: {state.value}'
            )
        self.state = state
        if hasattr(self, 'status_pub'):
            self._publish_status()

    def _now(self) -> float:
        """Return ROS time so simulation timeouts follow the Gazebo clock."""
        return self.get_clock().now().nanoseconds * 1e-9

    def _publish_status(self) -> None:
        message = String()
        message.data = self.state.value
        self.status_pub.publish(message)

    def _start(self, request, response):
        del request
        with self._lock:
            self._reset_run_state()
            self.paused = False
            self.stop_requested = False
            self._set_state(State.WAIT_SENSOR)
        response.success = True
        response.message = 'exploration started'
        return response

    def _pause(self, request, response):
        del request
        self.paused = True
        self._stop_cmd()
        response.success = True
        response.message = 'exploration paused'
        return response

    def _resume(self, request, response):
        del request
        self.paused = False
        response.success = True
        response.message = 'exploration resumed'
        return response

    def _stop(self, request, response):
        del request
        with self._lock:
            self.stop_requested = True
            self._invalidate_planning()
            self._stop_cmd()
            self._set_state(State.IDLE)
        response.success = True
        response.message = 'exploration stopped'
        return response

    def _reset_run_state(self) -> None:
        self._invalidate_planning()
        self.path = []
        self.path_index = 0
        self.path_signature = None
        self.goal = None
        self.manual_goal_world = None
        self.active_goal_world = None
        self.dfs_heading = None
        self.dfs_stack_world = []
        self.goal_started = 0.0
        self.last_progress_at = 0.0
        self.last_progress_pose = None
        self.blocked_since = None
        self.blacklist.clear()
        self.failure_counts.clear()
        self.completion.reset()
        self.initial_scan_started = 0.0
        self.initial_scan_completed = False
        self.rotation_accum = 0.0
        self.last_scan_yaw = None
        self.recovery_started = 0.0
        self.recovery_direction = 1.0
        self.recovery_can_reverse = False
        self.verify_started = 0.0
        self.save_future = None
        self.save_started = 0.0
        self.save_generation += 1
        self._stop_cmd()

    def _control_tick(self) -> None:
        try:
            self._publish_status()
            if self.paused or self.stop_requested:
                self._stop_cmd()
                return
            if self.state in (
                State.IDLE,
                State.COMPLETE,
                State.ERROR,
            ):
                self._stop_cmd()
                return

            now = self._now()
            if self.state == State.SAVE:
                self._stop_cmd()
                self._save_tick(now)
                return
            if self.state == State.WAIT_SENSOR:
                self._stop_cmd()
                if self._sensors_fresh(now):
                    self._set_state(State.WAIT_TF)
                return

            if not self._sensors_fresh(now):
                self._invalidate_planning()
                self._stop_cmd()
                self._set_state(State.WAIT_SENSOR)
                return

            pose = self._pose()
            if pose is None:
                self._stop_cmd()
                if (
                    self.last_tf_at == 0.0
                    or now - self.last_tf_at >= self.tf_timeout
                ):
                    self._invalidate_planning()
                    self._set_state(State.WAIT_TF)
                return

            if self.state == State.WAIT_TF:
                self._stop_cmd()
                self._set_state(State.WAIT_MAP)
                return

            if not self._map_fresh(now):
                self._invalidate_planning()
                self._stop_cmd()
                self._set_state(State.WAIT_MAP)
                return

            if self.state == State.WAIT_MAP:
                self._stop_cmd()
                should_scan = (
                    self.initial_scan_enabled
                    and not self.initial_scan_completed
                )
                if should_scan:
                    self.initial_scan_started = now
                    self.rotation_accum = 0.0
                    self.last_scan_yaw = pose[2]
                    self._set_state(State.INITIAL_SCAN)
                else:
                    self._set_state(State.DETECT)
                return

            if self.planning:
                self._stop_cmd()
                if now - self.planning_started >= self.planning_timeout:
                    self.get_logger().warning('planning timed out')
                    self._invalidate_planning()
                    self.recovery_started = 0.0
                    self._set_state(State.RECOVERY)
                return

            if self.state == State.INITIAL_SCAN:
                self._initial_scan_tick(pose, now)
            elif self.state == State.DETECT:
                self._stop_cmd()
                interval = 1.0 / max(self.frontier_frequency, 0.1)
                if now - self.last_frontier_at >= interval:
                    self._begin_plan(pose, choose_frontier=True)
            elif self.state == State.SELECT:
                self._stop_cmd()
                self._begin_plan(pose, choose_frontier=True)
            elif self.state == State.PLAN:
                self._stop_cmd()
                self._begin_plan(pose, choose_frontier=False)
            elif self.state == State.FOLLOW:
                self._follow(pose, now)
            elif self.state == State.REPLAN:
                self._stop_cmd()
                self._blacklist_goal()
                self.recovery_started = 0.0
                self._set_state(State.RECOVERY)
            elif self.state == State.RECOVERY:
                self._recovery_tick(now)
            elif self.state == State.VERIFY:
                self._stop_cmd()
                if self.verify_started == 0.0:
                    self.verify_started = now
                interval = 1.0 / max(self.frontier_frequency, 0.1)
                if now - self.verify_started >= interval:
                    self.verify_started = 0.0
                    self._set_state(State.DETECT)
        except Exception as exception:  # pragma: no cover - final safety net
            self.get_logger().error(f'control error: {exception}')
            self._invalidate_planning()
            self._stop_cmd()
            self._set_state(State.ERROR)

    def _sensors_fresh(self, now: float) -> bool:
        return (
            self.scan is not None
            and self.odom is not None
            and self._fresh(self.last_scan_at, self.sensor_timeout, now)
            and self._fresh(self.last_odom_at, self.odom_timeout, now)
        )

    def _map_fresh(self, now: float) -> bool:
        return (
            self.grid is not None
            and self._fresh(self.last_map_at, self.map_timeout, now)
        )

    @staticmethod
    def _fresh(timestamp: float, timeout: float, now: float) -> bool:
        return timestamp > 0.0 and (
            timeout <= 0.0 or now - timestamp <= timeout
        )

    def _initial_scan_tick(
        self,
        pose: Tuple[float, float, float],
        now: float,
    ) -> None:
        previous_yaw = (
            self.last_scan_yaw if self.last_scan_yaw is not None else pose[2]
        )
        self.rotation_accum += abs(normalize_angle(pose[2] - previous_yaw))
        self.last_scan_yaw = pose[2]
        timed_out = (
            now - self.initial_scan_started >= self.initial_scan_duration
        )
        if self.rotation_accum >= 2.0 * math.pi or timed_out:
            self._stop_cmd()
            self.initial_scan_completed = True
            self._set_state(State.DETECT)
            return
        linear, angular = initial_scan_velocity(self.initial_scan_speed)
        sectors = self._scan_sectors()
        if sectors is None:
            self._stop_cmd()
            return
        left_range, _, right_range = sectors
        if (
            left_range < self.side_clearance_distance
            and right_range < self.side_clearance_distance
        ):
            self._stop_cmd()
            return
        if left_range < self.side_clearance_distance:
            angular = -angular
        elif right_range < self.side_clearance_distance:
            angular = abs(angular)
        self._command(linear, angular)

    def _begin_plan(
        self,
        pose: Tuple[float, float, float],
        choose_frontier: bool,
    ) -> None:
        with self._lock:
            if self.planning or self.grid is None:
                return
            grid = self.grid
            start = grid.world_to_cell((pose[0], pose[1]))
            manual_goal_world = self.manual_goal_world
            fixed_goal_world = (
                manual_goal_world
                if manual_goal_world is not None
                else self.active_goal_world
            )
            failure_counts = dict(self.failure_counts)
            blacklist = dict(self.blacklist)
            dfs_heading = self.dfs_heading
            dfs_stack_world = tuple(self.dfs_stack_world)
            signature = grid.signature
            now = self._now()
            blocked = inflate_obstacles(
                grid,
                self.robot_radius + self.safety_margin,
                self.occupied_threshold,
            )
            blocked.discard(start)

            self.planning_generation += 1
            generation = self.planning_generation
            self.planning = True
            self.planning_started = now
            self.last_frontier_at = now
            self._set_state(
                State.SELECT if choose_frontier else State.PLAN
            )

        def work():
            frontiers: List[Set[Cell]] = []
            failed_goals: List[Cell] = []
            choices = []

            if choose_frontier:
                reachable = reachable_free(
                    grid,
                    start,
                    blocked,
                    self.occupied_threshold,
                )
                frontiers = detect_frontiers(
                    grid,
                    start,
                    blocked,
                    self.min_frontier_size,
                    self.occupied_threshold,
                    reachable=reachable,
                )
                frontier_goals = []
                for frontier in frontiers:
                    frontier_goals.append(candidate_goals(
                        grid,
                        frontier,
                        blocked,
                        self.frontier_goal_offset,
                        self.occupied_threshold,
                        self.max_candidates,
                        reachable,
                    ))

                # Treat max_candidates as a global planning budget.  Taking
                # that many goals from every frontier made planning time grow
                # with map size and could permanently trap the controller in
                # timeout recovery.  Round-robin selection still represents
                # every frontier cluster before using additional candidates.
                candidate_batch = bounded_candidate_batch(
                    frontier_goals,
                    self.max_candidates,
                )

                for goal in candidate_batch:
                    goal_distance = math.hypot(
                        goal[0] - start[0],
                        goal[1] - start[1],
                    ) * grid.resolution
                    if goal_distance < self.min_goal_distance:
                        continue
                    if now < blacklist.get(goal, 0.0):
                        continue
                    raw_path = astar(
                        grid,
                        start,
                        goal,
                        blocked,
                        self.occupied_threshold,
                    )
                    if not raw_path:
                        failed_goals.append(goal)
                        continue
                    simplified_path = simplify_path(
                        grid,
                        raw_path,
                        blocked,
                        self.occupied_threshold,
                    )
                    safe_path = round_path_corners(
                        grid,
                        simplified_path,
                        blocked,
                        self.occupied_threshold,
                        self.corner_rounding_fraction,
                    )
                    score = self._score(
                        grid,
                        goal,
                        safe_path,
                        blocked,
                        pose,
                        failure_counts.get(goal, 0),
                        dfs_heading,
                        dfs_stack_world,
                    )
                    choices.append((score, goal, safe_path))
            elif fixed_goal_world is not None:
                fixed_goal = grid.world_to_cell(fixed_goal_world)
                raw_path = astar(
                    grid,
                    start,
                    fixed_goal,
                    blocked,
                    self.occupied_threshold,
                )
                if raw_path:
                    choices.append((
                        0.0,
                        fixed_goal,
                        round_path_corners(
                            grid,
                            simplify_path(
                                grid,
                                raw_path,
                                blocked,
                                self.occupied_threshold,
                            ),
                            blocked,
                            self.occupied_threshold,
                            self.corner_rounding_fraction,
                        ),
                    ))
                else:
                    failed_goals.append(fixed_goal)

            choices.sort(key=lambda item: item[0], reverse=True)
            return {
                'signature': signature,
                'selected': choices[0] if choices else None,
                'frontiers': frontiers,
                'failed_goals': failed_goals,
                'fixed': not choose_frontier,
                'manual': manual_goal_world is not None,
            }

        self.plan_future = self.plan_pool.submit(work)
        self.plan_future.add_done_callback(
            lambda future: self._plan_done(future, generation)
        )

    def _plan_done(self, future, generation: int) -> None:
        try:
            result = future.result()
        except CancelledError:
            return
        except Exception as exception:  # pragma: no cover - worker safety net
            self.get_logger().error(f'planning failed: {exception}')
            result = None

        with self._lock:
            if generation != self.planning_generation or self._destroying:
                return
            self.plan_future = None
            if self.paused or self.stop_requested:
                self.planning = False
                return
            if result is None or self.grid is None:
                self.recovery_started = 0.0
                self._finish_planning(State.RECOVERY)
                return
            if result['signature'] != self.grid.signature:
                self._finish_planning(State.REPLAN)
                return

            now = self._now()
            for failed_goal in result['failed_goals']:
                self.failure_counts[failed_goal] = (
                    self.failure_counts.get(failed_goal, 0) + 1
                )
                self.blacklist[failed_goal] = (
                    now + self.failure_blacklist_duration
                )

            self._publish_frontiers(result['frontiers'], self.grid)
            selected = result['selected']
            if selected is not None:
                _, goal, path = selected
                blocked = inflate_obstacles(
                    self.grid,
                    self.robot_radius + self.safety_margin,
                    self.occupied_threshold,
                )
                blocked.discard(path[0])
                if not path_is_safe(
                    self.grid,
                    path,
                    blocked,
                    self.occupied_threshold,
                ):
                    self._finish_planning(State.REPLAN)
                    return
                self.goal = goal
                self.path = path
                self.path_index = 0
                if not result.get('fixed', False):
                    self.active_goal_world = self.grid.cell_to_world(goal)
                self._commit_dfs_direction(path)
                self.path_signature = self.grid.signature
                self.goal_started = now
                self.last_progress_at = now
                self.last_progress_pose = None
                self.blocked_since = None
                self.completion.observe(
                    now,
                    known_area(self.grid),
                    True,
                )
                self._publish_path()
                self._publish_goal()
                self._finish_planning(State.FOLLOW)
                return

            if result.get('fixed', False):
                if result['manual']:
                    self.manual_goal_world = None
                else:
                    self.active_goal_world = None
                self.goal = None
                self.recovery_started = 0.0
                self._finish_planning(State.RECOVERY)
                return

            completed = self.completion.observe(
                now,
                known_area(self.grid),
                False,
            )
            if completed:
                self._begin_save()
                self.planning = False
                self.planning_started = 0.0
            elif result['manual']:
                self.manual_goal_world = None
                self.recovery_started = 0.0
                self._finish_planning(State.RECOVERY)
            elif result['frontiers']:
                self.recovery_started = 0.0
                self._finish_planning(State.RECOVERY)
            else:
                self.verify_started = now
                self._finish_planning(State.VERIFY)

    def _finish_planning(self, state: State) -> None:
        """Publish the next state before exposing the planner as idle."""
        self._set_state(state)
        self.planning = False
        self.planning_started = 0.0

    def _invalidate_planning(self) -> None:
        self.planning_generation += 1
        self.planning = False
        self.planning_started = 0.0
        if self.plan_future is not None:
            self.plan_future.cancel()
            self.plan_future = None

    def _score(
        self,
        grid: GridMap,
        goal: Cell,
        path: Sequence[Cell],
        blocked: Set[Cell],
        pose: Tuple[float, float, float],
        failure_count: int,
        dfs_heading: Optional[float] = None,
        dfs_stack_world: Sequence[Tuple[float, float]] = (),
    ) -> float:
        goal_x, goal_y = grid.cell_to_world(goal)
        goal_bearing = math.atan2(goal_y - pose[1], goal_x - pose[0])
        heading = abs(normalize_angle(goal_bearing - pose[2]))
        risk = obstacle_clearance_risk(
            path,
            blocked,
            self.risk_radius_cells,
        )
        gain_score = self.w_gain * information_gain(
            grid,
            goal,
            self.information_radius_cells,
        )
        common_penalty = (
            self.w_risk * risk
            + self.w_turn * heading
            + self.w_failure * failure_count
        )
        if self.exploration_strategy == 'dfs':
            reference_heading = (
                pose[2] if dfs_heading is None else dfs_heading
            )
            alignment = math.cos(normalize_angle(
                goal_bearing - reference_heading
            ))
            depth = min(
                path_cost(path) * grid.resolution,
                self.dfs_depth_cap,
            )
            revisit = 0.0
            if dfs_stack_world:
                nearest_visit = min(
                    math.hypot(goal_x - point[0], goal_y - point[1])
                    for point in dfs_stack_world
                )
                revisit = max(
                    0.0,
                    1.0 - nearest_visit / self.dfs_revisit_radius,
                )
            return (
                self.dfs_gain_scale * gain_score
                + self.dfs_heading_weight * alignment
                + self.dfs_depth_weight * depth
                - self.dfs_revisit_penalty * revisit
                - common_penalty
            )
        return (
            gain_score
            - self.w_path * path_cost(path)
            - common_penalty
        )

    def _commit_dfs_direction(self, path: Sequence[Cell]) -> None:
        """Keep following the current branch until a real branch switch."""
        if (
            self.exploration_strategy != 'dfs'
            or self.grid is None
            or len(path) < 2
        ):
            return
        start_x, start_y = self.grid.cell_to_world(path[0])
        goal_x, goal_y = self.grid.cell_to_world(path[-1])
        bearing = math.atan2(goal_y - start_y, goal_x - start_x)
        branch_switch = (
            self.dfs_heading is None
            or abs(normalize_angle(
                bearing - self.dfs_heading
            )) >= self.dfs_branch_switch_angle
        )
        if branch_switch:
            self.dfs_heading = bearing

    def _record_dfs_progress(self, point: Tuple[float, float]) -> None:
        """Push a completed branch point onto the DFS history stack."""
        if self.exploration_strategy != 'dfs':
            return
        if self.dfs_stack_world:
            previous = self.dfs_stack_world[-1]
            distance = math.hypot(
                point[0] - previous[0],
                point[1] - previous[1],
            )
            if distance > 1e-3:
                self.dfs_heading = math.atan2(
                    point[1] - previous[1],
                    point[0] - previous[0],
                )
        self.dfs_stack_world.append(point)
        if len(self.dfs_stack_world) > self.dfs_stack_limit:
            del self.dfs_stack_world[:-self.dfs_stack_limit]

    def _follow(
        self,
        pose: Tuple[float, float, float],
        now: float,
    ) -> None:
        if not self.path or self.goal is None or self.grid is None:
            self._stop_cmd()
            self._set_state(State.REPLAN)
            return
        if self.path_signature != self.grid.signature:
            self._stop_cmd()
            self._set_state(State.REPLAN)
            return
        if now - self.goal_started >= self.goal_timeout:
            self._stop_cmd()
            self._set_state(State.REPLAN)
            return

        blocked = inflate_obstacles(
            self.grid,
            self.robot_radius + self.safety_margin,
            self.occupied_threshold,
        )
        current_cell = self.grid.world_to_cell((pose[0], pose[1]))
        blocked.discard(current_cell)
        remaining_path = self.path[max(0, self.path_index - 1):]
        if not path_is_safe(
            self.grid,
            remaining_path,
            blocked,
            self.occupied_threshold,
        ):
            self._stop_cmd()
            self._set_state(State.REPLAN)
            return

        goal_x, goal_y = self.grid.cell_to_world(self.goal)
        goal_distance = math.hypot(goal_x - pose[0], goal_y - pose[1])
        if goal_distance <= self.goal_tolerance:
            reached_goal = self.goal
            self._record_dfs_progress((goal_x, goal_y))
            self._stop_cmd()
            self.goal = None
            self.path = []
            self.path_index = 0
            self.manual_goal_world = None
            self.active_goal_world = None
            self.failure_counts.pop(reached_goal, None)
            self.blacklist.pop(reached_goal, None)
            self._set_state(State.DETECT)
            return

        closest_index, deviation = self._closest_path_index(pose)
        self.path_index = max(self.path_index, closest_index)
        if deviation > self.path_deviation_tolerance:
            self.get_logger().warning(
                f'path deviation {deviation:.3f} exceeds tolerance'
            )
            self._stop_cmd()
            self._set_state(State.REPLAN)
            return

        if self.last_progress_pose is None:
            self.last_progress_pose = pose
            self.last_progress_at = now
        else:
            linear_progress = math.hypot(
                pose[0] - self.last_progress_pose[0],
                pose[1] - self.last_progress_pose[1],
            )
            angular_progress = abs(normalize_angle(
                pose[2] - self.last_progress_pose[2]
            ))
            if (
                linear_progress >= self.progress_distance
                or angular_progress >= self.progress_angle
            ):
                self.last_progress_pose = pose
                self.last_progress_at = now
            elif (
                (
                    abs(self.last_command[0]) > 1e-3
                    or abs(self.last_command[1]) > 1e-3
                )
                and now - self.last_progress_at >= self.stuck_timeout
            ):
                self.get_logger().warning('robot appears stuck')
                self._stop_cmd()
                self._set_state(State.REPLAN)
                return

        sectors = self._scan_sectors()
        if sectors is None:
            self._handle_blocked(now, 'no valid laser samples')
            return
        left_range, front_range, right_range = sectors
        if front_range < self.emergency_stop_distance:
            self._handle_blocked(now, 'front obstacle')
            return

        target_cell = self._lookahead_cell(pose)
        target_x, target_y = self.grid.cell_to_world(target_cell)
        delta_x = target_x - pose[0]
        delta_y = target_y - pose[1]
        local_x = math.cos(pose[2]) * delta_x + math.sin(pose[2]) * delta_y
        local_y = -math.sin(pose[2]) * delta_x + math.cos(pose[2]) * delta_y
        heading_error = normalize_angle(
            math.atan2(delta_y, delta_x) - pose[2]
        )

        if local_x <= 0.0 or abs(heading_error) >= self.rotate_in_place_angle:
            angular = max(
                -self.max_angular_speed,
                min(self.max_angular_speed, heading_error),
            )
            limited_angular = self._limit_turn_for_side_clearance(
                angular,
                left_range,
                right_range,
            )
            if abs(angular) > 1e-3 and abs(limited_angular) <= 1e-3:
                self._handle_blocked(now, 'side clearance prevents turn')
                return
            self.blocked_since = None
            self._command(0.0, limited_angular)
            return

        target_distance = max(math.hypot(delta_x, delta_y), 0.05)
        curvature = 2.0 * local_y / (target_distance * target_distance)
        speed = min(
            self.max_linear_speed,
            max(self.min_linear_speed, goal_distance * 0.5),
        )
        speed /= 1.0 + abs(curvature)
        if front_range < self.slowdown_distance:
            scale = (
                (front_range - self.emergency_stop_distance)
                / max(
                    0.01,
                    self.slowdown_distance - self.emergency_stop_distance,
                )
            )
            speed *= max(0.0, min(1.0, scale))

        angular = max(
            -self.max_angular_speed,
            min(self.max_angular_speed, curvature * speed),
        )
        limited_angular = self._limit_turn_for_side_clearance(
            angular,
            left_range,
            right_range,
        )
        if abs(angular) > 1e-3 and abs(limited_angular) <= 1e-3:
            self._handle_blocked(now, 'side clearance prevents turn')
            return
        self.blocked_since = None
        self._command(speed, limited_angular)

    def _closest_path_index(
        self,
        pose: Tuple[float, float, float],
    ) -> Tuple[int, float]:
        start_index = max(0, self.path_index - 1)
        if start_index >= len(self.path) - 1:
            path_x, path_y = self.grid.cell_to_world(self.path[-1])
            return len(self.path) - 1, math.hypot(
                path_x - pose[0], path_y - pose[1]
            )

        best_distance = math.inf
        best_index = start_index
        for index in range(start_index, len(self.path) - 1):
            start_x, start_y = self.grid.cell_to_world(self.path[index])
            end_x, end_y = self.grid.cell_to_world(self.path[index + 1])
            segment_x = end_x - start_x
            segment_y = end_y - start_y
            segment_length_sq = segment_x ** 2 + segment_y ** 2
            if segment_length_sq <= 1e-12:
                projection = 0.0
            else:
                projection = (
                    (pose[0] - start_x) * segment_x
                    + (pose[1] - start_y) * segment_y
                ) / segment_length_sq
                projection = max(0.0, min(1.0, projection))
            nearest_x = start_x + projection * segment_x
            nearest_y = start_y + projection * segment_y
            distance = math.hypot(
                nearest_x - pose[0], nearest_y - pose[1]
            )
            if distance < best_distance:
                best_distance = distance
                best_index = index + (1 if projection >= 0.80 else 0)
        return best_index, best_distance

    def _lookahead_cell(
        self,
        pose: Tuple[float, float, float],
    ) -> Cell:
        adaptive_lookahead = min(
            self.lookahead_max,
            self.lookahead
            + self.lookahead_speed_gain * abs(self.last_command[0]),
        )
        target = self.path[-1]
        for index in range(self.path_index, len(self.path)):
            path_x, path_y = self.grid.cell_to_world(self.path[index])
            target_distance = math.hypot(
                path_x - pose[0],
                path_y - pose[1],
            )
            if target_distance >= adaptive_lookahead:
                target = self.path[index]
                break
        return target

    def _scan_sectors(self) -> Optional[Tuple[float, float, float]]:
        if self.scan is None:
            return None
        return scan_sector_minima(
            self.scan.ranges,
            self.scan.angle_min,
            self.scan.angle_increment,
            self.scan.range_min,
            self.scan.range_max,
        )

    def _scan_directions(
        self,
    ) -> Optional[Tuple[float, float, float, float]]:
        if self.scan is None:
            return None
        return scan_directional_minima(
            self.scan.ranges,
            self.scan.angle_min,
            self.scan.angle_increment,
            self.scan.range_min,
            self.scan.range_max,
        )

    def _handle_blocked(self, now: float, reason: str) -> None:
        self._stop_cmd()
        if self.blocked_since is None:
            self.blocked_since = now
            self.get_logger().warning(f'motion blocked: {reason}')
            return
        if now - self.blocked_since >= self.blocked_timeout:
            self.get_logger().warning('blocked timeout; requesting replan')
            self.blocked_since = None
            self._set_state(State.REPLAN)

    def _limit_turn_for_side_clearance(
        self,
        angular: float,
        left_range: float,
        right_range: float,
    ) -> float:
        clearance = left_range if angular > 0.0 else right_range
        if abs(angular) <= 1e-9 or clearance >= self.side_clearance_distance:
            return angular
        if clearance <= self.side_hard_stop_distance:
            return 0.0
        span = (
            self.side_clearance_distance - self.side_hard_stop_distance
        )
        ratio = (
            clearance - self.side_hard_stop_distance
        ) / max(span, 1e-6)
        scale = max(
            self.side_min_turn_scale,
            min(1.0, ratio),
        )
        return angular * scale

    def _recovery_tick(self, now: float) -> None:
        directions = self._scan_directions()
        if directions is None:
            self._stop_cmd()
            return
        left_range, _, right_range, rear_range = directions

        if self.recovery_started == 0.0:
            self.recovery_started = now
            self.recovery_direction = (
                1.0 if left_range >= right_range else -1.0
            )
            self.recovery_can_reverse = (
                rear_range >= self.recovery_backup_clearance
            )

        elapsed = now - self.recovery_started
        if elapsed >= self.recovery_duration:
            self._stop_cmd()
            self.recovery_started = 0.0
            self.recovery_can_reverse = False
            self.blocked_since = None
            self._set_state(State.DETECT)
            return

        if (
            elapsed < self.recovery_backup_duration
            and self.recovery_can_reverse
        ):
            if rear_range < self.recovery_backup_clearance:
                self.recovery_can_reverse = False
            else:
                self._command(-self.recovery_linear_speed, 0.0)
                return

        if left_range <= self.side_hard_stop_distance and (
            right_range <= self.side_hard_stop_distance
        ):
            self._stop_cmd()
            return
        self._command(
            0.0,
            self.recovery_direction * min(
                self.recovery_angular_speed,
                self.max_angular_speed,
            ),
        )

    def _blacklist_goal(self) -> None:
        if self.goal is None:
            self.path = []
            self.path_index = 0
            return
        self.failure_counts[self.goal] = (
            self.failure_counts.get(self.goal, 0) + 1
        )
        self.blacklist[self.goal] = (
            self._now() + self.failure_blacklist_duration
        )
        self.goal = None
        self.active_goal_world = None
        self.path = []
        self.path_index = 0
        self.path_signature = None

    def _begin_save(self) -> None:
        self._invalidate_planning()
        self._stop_cmd()
        self.save_future = None
        self.save_started = self._now()
        self.save_generation += 1
        self._set_state(State.SAVE)

    def _save_tick(self, now: float) -> None:
        if not self.auto_save_map:
            self.get_logger().info('map saving disabled; exploration complete')
            self._set_state(State.COMPLETE)
            return
        if self.save_client is None:
            self.get_logger().error('map save service is not configured')
            self._set_state(State.ERROR)
            return
        if self.save_future is not None:
            if now - self.save_started >= self.map_save_timeout:
                self.get_logger().error('map save request timed out')
                self.save_generation += 1
                self.save_future = None
                self._set_state(State.ERROR)
            return
        if not self.save_client.service_is_ready():
            if now - self.save_started >= self.map_save_timeout:
                self.get_logger().error('map save service is unavailable')
                self._set_state(State.ERROR)
            return

        request = SaveMap.Request()
        request.name.data = self.map_save_name
        generation = self.save_generation
        self.save_future = self.save_client.call_async(request)
        self.save_future.add_done_callback(
            lambda future: self._save_done(future, generation)
        )

    def _save_done(self, future, generation: int) -> None:
        if generation != self.save_generation or self._destroying:
            return
        try:
            response = future.result()
        except Exception as exception:  # pragma: no cover - middleware error
            self.get_logger().error(f'map save failed: {exception}')
            self._set_state(State.ERROR)
            return
        self.save_future = None
        if int(response.result) == int(SaveMap.Response.RESULT_SUCCESS):
            self.get_logger().info(f'map saved as {self.map_save_name}')
            self._set_state(State.COMPLETE)
        else:
            self.get_logger().error(
                f'map save service returned result {response.result}'
            )
            self._set_state(State.ERROR)

    def _command(self, linear: float, angular: float) -> None:
        message = Twist()
        message.linear.x = float(linear)
        message.angular.z = float(angular)
        self.last_command = (message.linear.x, message.angular.z)
        if not rclpy.ok(context=self.context):
            return
        try:
            self.cmd_pub.publish(message)
        except Exception as exception:  # pragma: no cover - shutdown race
            if not self._destroying:
                self.get_logger().error(
                    f'failed to publish velocity command: {exception}'
                )

    def _stop_cmd(self) -> None:
        self._command(0.0, 0.0)

    def _publish_path(self) -> None:
        if self.grid is None:
            return
        message = Path()
        message.header.frame_id = self.map_frame
        message.header.stamp = self.get_clock().now().to_msg()
        for cell in self.path:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x, pose.pose.position.y = (
                self.grid.cell_to_world(cell)
            )
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)
        self.path_pub.publish(message)

    def _publish_goal(self) -> None:
        if self.grid is None or self.goal is None:
            return
        message = PoseStamped()
        message.header.frame_id = self.map_frame
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.position.x, message.pose.position.y = (
            self.grid.cell_to_world(self.goal)
        )
        message.pose.orientation.w = 1.0
        self.goal_pub.publish(message)

    def _publish_frontiers(
        self,
        frontiers: Sequence[Set[Cell]],
        grid: GridMap,
    ) -> None:
        timestamp = self.get_clock().now().to_msg()
        markers = MarkerArray()
        clear = Marker()
        clear.header.frame_id = self.map_frame
        clear.header.stamp = timestamp
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for index, frontier in enumerate(frontiers):
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = timestamp
            marker.ns = 'frontiers'
            marker.id = index
            marker.type = Marker.POINTS
            marker.action = Marker.ADD
            marker.scale.x = grid.resolution
            marker.scale.y = grid.resolution
            marker.color.r = 0.1
            marker.color.g = 0.6
            marker.color.b = 1.0
            marker.color.a = 0.9
            for cell in frontier:
                world_x, world_y = grid.cell_to_world(cell)
                marker.points.append(
                    Point(x=world_x, y=world_y, z=0.02)
                )
            markers.markers.append(marker)
        self.frontier_pub.publish(markers)

    def destroy_node(self) -> None:
        self._destroying = True
        self._invalidate_planning()
        self.save_generation += 1
        self._stop_cmd()
        self.plan_pool.shutdown(wait=False, cancel_futures=True)
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AutonomousExplorationNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass
