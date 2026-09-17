import pytest

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
from rclpy.task import Future
from slam_toolbox.srv import SaveMap
from sensor_msgs.msg import LaserScan

from navigation.autonomous_exploration_node import (
    AutonomousExplorationNode,
    State,
)
from navigation.exploration_core import GridMap


@pytest.fixture
def exploration_node():
    if not rclpy.ok():
        rclpy.init()
    node = AutonomousExplorationNode()
    yield node
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def test_default_state_and_sensor_qos(exploration_node):
    assert exploration_node.state == State.WAIT_SENSOR
    assert exploration_node.exploration_strategy == 'dfs'
    assert (
        exploration_node.min_goal_distance
        > exploration_node.goal_tolerance
    )
    assert exploration_node.progress_angle > 0.0
    assert (
        exploration_node.scan_sub.qos_profile.reliability
        == ReliabilityPolicy.BEST_EFFORT
    )
    assert (
        exploration_node.map_sub.qos_profile.durability
        == DurabilityPolicy.TRANSIENT_LOCAL
    )


def test_external_goal_is_disabled_by_default(exploration_node):
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.pose.position.x = 1.0
    goal.pose.position.y = 2.0
    exploration_node._goal_cb(goal)
    assert exploration_node.manual_goal_world is None


def test_blocked_timeout_requests_replan(exploration_node):
    exploration_node.state = State.FOLLOW
    exploration_node._handle_blocked(10.0, 'test obstacle')
    assert exploration_node.state == State.FOLLOW
    exploration_node._handle_blocked(
        10.0 + exploration_node.blocked_timeout,
        'test obstacle',
    )
    assert exploration_node.state == State.REPLAN


def test_side_clearance_scales_turn_before_hard_stop(exploration_node):
    scaled = exploration_node._limit_turn_for_side_clearance(
        0.4,
        0.27,
        5.0,
    )
    stopped = exploration_node._limit_turn_for_side_clearance(
        0.4,
        0.18,
        5.0,
    )

    assert 0.0 < scaled < 0.4
    assert stopped == 0.0


def test_recovery_backs_up_then_turns(exploration_node):
    scan = LaserScan()
    scan.angle_min = -3.141592653589793
    scan.angle_increment = 2.0 * 3.141592653589793 / 360.0
    scan.range_min = 0.05
    scan.range_max = 10.0
    scan.ranges = [5.0] * 360
    exploration_node.scan = scan
    exploration_node.state = State.RECOVERY

    exploration_node._recovery_tick(10.0)
    assert exploration_node.last_command[0] < 0.0
    assert exploration_node.last_command[1] == 0.0

    exploration_node._recovery_tick(
        10.0 + exploration_node.recovery_backup_duration + 0.1
    )
    assert exploration_node.last_command[0] == 0.0
    assert exploration_node.last_command[1] != 0.0


def test_map_growth_keeps_world_goal_and_requests_new_path(
    exploration_node,
):
    old_grid = GridMap(4, 4, 1.0, 0.0, 0.0, [0] * 16)
    exploration_node.grid = old_grid
    exploration_node.state = State.FOLLOW
    exploration_node.path = [(1, 1), (2, 1)]
    exploration_node.goal = (2, 1)
    exploration_node.active_goal_world = (2.5, 1.5)
    exploration_node.path_signature = old_grid.signature

    message = OccupancyGrid()
    message.info.width = 6
    message.info.height = 4
    message.info.resolution = 1.0
    message.info.origin.position.x = -1.0
    message.info.origin.position.y = 0.0
    message.data = [0] * 24

    exploration_node._map_cb(message)

    assert exploration_node.state == State.PLAN
    assert exploration_node.path == []
    assert exploration_node.goal == (3, 1)
    assert exploration_node.active_goal_world == (2.5, 1.5)


def test_replanned_fixed_goal_does_not_shift_world_target(
    exploration_node,
):
    grid = GridMap(5, 5, 1.0, -1.0, 0.0, [0] * 25)
    exploration_node.grid = grid
    exploration_node.active_goal_world = (2.5, 1.5)
    exploration_node.planning = True
    exploration_node.planning_generation = 9
    future = Future()
    future.set_result({
        'signature': grid.signature,
        'selected': (0.0, (3, 1), [(1, 1), (3, 1)]),
        'frontiers': [],
        'failed_goals': [],
        'fixed': True,
        'manual': False,
    })

    exploration_node._plan_done(future, 9)

    assert exploration_node.state == State.FOLLOW
    assert exploration_node.goal == (3, 1)
    assert exploration_node.active_goal_world == (2.5, 1.5)


def test_path_deviation_uses_segments_not_only_sparse_points(
    exploration_node,
):
    exploration_node.grid = GridMap(
        6, 2, 1.0, 0.0, 0.0, [0] * 12
    )
    exploration_node.path = [(0, 0), (5, 0)]
    exploration_node.path_index = 0

    index, deviation = exploration_node._closest_path_index(
        (3.0, 0.7, 0.0)
    )

    assert index == 0
    assert deviation == pytest.approx(0.2)


def test_dfs_scoring_prefers_deeper_aligned_frontier(exploration_node):
    grid = GridMap(8, 8, 1.0, 0.0, 0.0, [0] * 64)
    pose = (0.5, 0.5, 0.0)

    deep_score = exploration_node._score(
        grid,
        (5, 0),
        [(0, 0), (5, 0)],
        set(),
        pose,
        0,
        0.0,
        (),
    )
    side_score = exploration_node._score(
        grid,
        (0, 2),
        [(0, 0), (0, 2)],
        set(),
        pose,
        0,
        0.0,
        (),
    )

    assert deep_score > side_score


def test_dfs_scoring_penalizes_recent_branch_points(exploration_node):
    grid = GridMap(8, 8, 1.0, 0.0, 0.0, [0] * 64)
    pose = (0.5, 0.5, 0.0)
    path = [(0, 0), (3, 0)]

    fresh_score = exploration_node._score(
        grid, (3, 0), path, set(), pose, 0, 0.0, ()
    )
    revisited_score = exploration_node._score(
        grid, (3, 0), path, set(), pose, 0, 0.0, ((3.5, 0.5),)
    )

    assert fresh_score > revisited_score


def test_plan_transition_sets_state_before_exposing_idle_planner(
    exploration_node,
):
    occupancy = GridMap(3, 3, 1.0, 0.0, 0.0, [0] * 9)
    exploration_node.grid = occupancy
    exploration_node.planning = True
    exploration_node.planning_generation = 7
    observed = []
    original_set_state = exploration_node._set_state

    def observe_transition(state):
        observed.append((state, exploration_node.planning))
        original_set_state(state)

    exploration_node._set_state = observe_transition
    future = Future()
    future.set_result({
        'signature': occupancy.signature,
        'selected': None,
        'frontiers': [],
        'failed_goals': [],
        'manual': False,
    })

    exploration_node._plan_done(future, 7)

    assert observed[-1] == (State.VERIFY, True)
    assert exploration_node.planning is False


def test_successful_map_save_completes(exploration_node):
    exploration_node.state = State.SAVE
    exploration_node.save_generation = 3
    response = SaveMap.Response()
    response.result = SaveMap.Response.RESULT_SUCCESS
    future = Future()
    future.set_result(response)

    exploration_node._save_done(future, 3)

    assert exploration_node.state == State.COMPLETE


def test_failed_map_save_enters_error(exploration_node):
    exploration_node.state = State.SAVE
    exploration_node.save_generation = 4
    response = SaveMap.Response()
    response.result = SaveMap.Response.RESULT_NO_MAP_RECEIEVD
    future = Future()
    future.set_result(response)

    exploration_node._save_done(future, 4)

    assert exploration_node.state == State.ERROR
