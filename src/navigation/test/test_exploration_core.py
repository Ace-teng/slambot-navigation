import math

from navigation.exploration_core import (
    CompletionMonitor,
    GridMap,
    astar,
    bounded_candidate_batch,
    candidate_goals,
    detect_frontiers,
    inflate_obstacles,
    initial_scan_velocity,
    line_is_safe,
    path_is_safe,
    reachable_free,
    reproject_cells,
    round_path_corners,
    scan_directional_minima,
    scan_sector_minima,
    simplify_path,
)


def grid(rows, resolution=0.1):
    return GridMap(
        len(rows[0]),
        len(rows),
        resolution,
        -1.0,
        -1.0,
        [value for row in rows for value in row],
    )


def test_coordinate_conversion_roundtrip():
    occupancy = GridMap(10, 10, 0.5, -2.0, 1.0, [0] * 100)
    assert occupancy.world_to_cell((-0.1, 1.1)) == (3, 0)
    assert occupancy.cell_to_world((3, 0)) == (-0.25, 1.25)


def test_frontier_is_reachable_and_clustered():
    occupancy = grid([
        [-1, -1, -1, -1, -1],
        [-1, 0, 0, 0, -1],
        [-1, 0, 0, 0, -1],
        [-1, -1, -1, -1, -1],
    ])
    frontiers = detect_frontiers(occupancy, (2, 1), set(), min_size=2)
    assert frontiers
    assert sum(map(len, frontiers)) >= 4


def test_inflation_blocks_near_obstacle():
    occupancy = grid([
        [0, 0, 0],
        [0, 100, 0],
        [0, 0, 0],
    ])
    assert len(inflate_obstacles(occupancy, 0.11)) == 9


def test_astar_rejects_obstacle_corner_cutting():
    occupancy = grid([
        [0, 100],
        [100, 0],
    ])
    assert astar(occupancy, (0, 0), (1, 1), {(0, 1), (1, 0)}) is None


def test_astar_rejects_unknown_corner_cutting():
    occupancy = grid([
        [0, -1],
        [-1, 0],
    ])
    assert astar(occupancy, (0, 0), (1, 1), set()) is None


def test_astar_finds_safe_route():
    occupancy = grid([
        [0, 0, 0],
        [100, 100, 0],
        [0, 0, 0],
    ])
    path = astar(occupancy, (0, 0), (2, 2), set())
    assert path
    assert path[0] == (0, 0)
    assert path[-1] == (2, 2)


def test_reachable_free_does_not_cross_unknown_corner():
    occupancy = grid([
        [0, -1],
        [-1, 0],
    ])
    assert reachable_free(occupancy, (0, 0), set()) == {(0, 0)}


def test_simplification_does_not_cross_blocked_cell():
    occupancy = grid([
        [0, 0, 0],
        [0, 100, 0],
        [0, 0, 0],
    ])
    path = [(0, 0), (0, 1), (0, 2), (1, 2), (2, 2)]
    result = simplify_path(occupancy, path, {(1, 1)})
    assert result[0] == path[0]
    assert result[-1] == path[-1]
    assert not line_is_safe(occupancy, (0, 0), (2, 2), {(1, 1)})


def test_simplification_does_not_cross_unknown_corner():
    occupancy = grid([
        [0, -1, 0],
        [0, 0, 0],
    ])
    path = [(0, 0), (0, 1), (1, 1), (2, 1), (2, 0)]
    result = simplify_path(occupancy, path, set())
    assert result != [(0, 0), (2, 0)]


def test_corner_rounding_keeps_a_safe_shortcut():
    occupancy = grid([[0] * 12 for _ in range(12)])
    path = [(1, 1), (1, 8), (9, 8)]

    rounded = round_path_corners(occupancy, path, set(), fraction=0.35)

    assert rounded[0] == path[0]
    assert rounded[-1] == path[-1]
    assert (1, 8) not in rounded
    assert path_is_safe(occupancy, rounded, set())


def test_corner_rounding_falls_back_when_shortcut_is_blocked():
    occupancy = grid([[0] * 12 for _ in range(12)])
    path = [(1, 1), (1, 8), (9, 8)]
    blocked = {(2, 7), (3, 7), (3, 6)}

    rounded = round_path_corners(
        occupancy,
        path,
        blocked,
        fraction=0.35,
    )

    assert path_is_safe(occupancy, rounded, blocked)


def test_reproject_cells_preserves_world_location():
    source = GridMap(5, 5, 1.0, 0.0, 0.0, [0] * 25)
    target = GridMap(8, 7, 1.0, -2.0, -1.0, [0] * 56)

    projected = reproject_cells(source, target, [(1, 1), (2, 1)])

    assert projected == [(3, 2), (4, 2)]


def test_candidate_goals_are_limited_to_reachable_cells():
    occupancy = grid([
        [-1, -1, -1, -1, -1, -1],
        [-1, 0, 0, 100, 0, -1],
        [-1, 0, 0, 100, 0, -1],
        [-1, -1, -1, -1, -1, -1],
    ])
    reachable = reachable_free(occupancy, (1, 1), {(3, 1), (3, 2)})
    frontier = {(1, 1), (2, 1), (1, 2), (2, 2)}
    goals = candidate_goals(
        occupancy,
        frontier,
        {(3, 1), (3, 2)},
        offset=0.1,
        reachable=reachable,
    )
    assert goals
    assert all(goal in reachable for goal in goals)
    assert (4, 1) not in goals


def test_candidate_goals_cover_a_long_frontier():
    occupancy = grid([
        [0] * 32,
        [0] * 32,
        [-1] * 32,
    ])
    frontier = {(x, 1) for x in range(1, 31)}
    reachable = reachable_free(occupancy, (16, 0), set())

    goals = candidate_goals(
        occupancy,
        frontier,
        set(),
        offset=0.1,
        max_candidates=6,
        reachable=reachable,
    )

    goal_x = [goal[0] for goal in goals]
    assert len(goals) == 6
    assert min(goal_x) <= 3
    assert max(goal_x) >= 28


def test_candidate_batch_is_global_unique_and_round_robin():
    groups = [
        [(1, 0), (2, 0), (3, 0)],
        [(10, 0), (2, 0), (12, 0)],
        [(20, 0)],
    ]

    result = bounded_candidate_batch(groups, 5)

    assert result == [(1, 0), (10, 0), (20, 0), (2, 0), (3, 0)]


def test_frontier_pipeline_produces_a_safe_path():
    occupancy = grid([
        [-1, -1, -1, -1, -1, -1, -1],
        [-1, 0, 0, 0, 0, 0, -1],
        [-1, 0, 0, 0, 0, 0, -1],
        [-1, 0, 0, 100, 0, 0, -1],
        [-1, 0, 0, 0, 0, 0, -1],
        [-1, 0, 0, 0, 0, 0, -1],
        [-1, -1, -1, -1, -1, -1, -1],
    ])
    blocked = inflate_obstacles(occupancy, 0.0)
    start = (2, 2)
    reachable = reachable_free(occupancy, start, blocked)
    frontiers = detect_frontiers(
        occupancy,
        start,
        blocked,
        min_size=2,
    )

    paths = []
    for frontier in frontiers:
        for goal in candidate_goals(
            occupancy,
            frontier,
            blocked,
            offset=0.1,
            reachable=reachable,
        ):
            path = astar(occupancy, start, goal, blocked)
            if path:
                paths.append(simplify_path(occupancy, path, blocked))

    assert paths
    assert all(path_is_safe(occupancy, path, blocked) for path in paths)


def test_completion_requires_repeated_absence_and_stable_area():
    monitor = CompletionMonitor(confirm_count=3, stable_duration=5.0)
    assert not monitor.observe(0.0, 10, False)
    assert not monitor.observe(3.0, 10, False)
    assert monitor.observe(5.0, 10, False)

    assert not monitor.observe(6.0, 11, False)
    assert not monitor.observe(12.0, 11, True)
    assert not monitor.observe(13.0, 11, False)


def test_initial_scan_command_is_rotation_only():
    linear, angular = initial_scan_velocity(-0.25)
    assert linear == 0.0
    assert angular == 0.25


def test_scan_sectors_wrap_zero_to_two_pi_angles():
    count = 360
    ranges = [5.0] * count
    ranges[359] = 0.4
    sectors = scan_sector_minima(
        ranges,
        0.0,
        2.0 * math.pi / count,
        0.05,
        10.0,
    )
    assert sectors is not None
    _, front, _ = sectors
    assert front == 0.4


def test_scan_with_no_finite_samples_is_invalid():
    sectors = scan_sector_minima(
        [float('inf')] * 10,
        -0.5,
        0.1,
        0.05,
        10.0,
    )
    assert sectors is None


def test_directional_scan_reports_rear_clearance():
    count = 360
    ranges = [5.0] * count
    ranges[0] = 0.7
    directions = scan_directional_minima(
        ranges,
        -math.pi,
        2.0 * math.pi / count,
        0.05,
        10.0,
    )

    assert directions is not None
    _, _, _, rear = directions
    assert rear == 0.7
