"""Dependency-free algorithms for autonomous frontier exploration."""

from collections import deque
from dataclasses import dataclass
import heapq
import math
from typing import Dict, Iterable, List, Optional, Sequence, Set, Tuple


Cell = Tuple[int, int]
Point = Tuple[float, float]


@dataclass(frozen=True)
class GridMap:
    """A small immutable view of a ROS occupancy grid."""

    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    data: Sequence[int]

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0 or self.resolution <= 0.0:
            raise ValueError('invalid map geometry')
        if len(self.data) != self.width * self.height:
            raise ValueError('map data size does not match geometry')

    @property
    def signature(self) -> Tuple[int, int, float, float, float]:
        """Return the geometry fields that make cell coordinates compatible."""
        return (
            self.width,
            self.height,
            self.resolution,
            self.origin_x,
            self.origin_y,
        )

    def in_bounds(self, cell: Cell) -> bool:
        return 0 <= cell[0] < self.width and 0 <= cell[1] < self.height

    def value(self, cell: Cell) -> int:
        return int(self.data[cell[1] * self.width + cell[0]])

    def world_to_cell(self, point: Point) -> Cell:
        return (
            math.floor((point[0] - self.origin_x) / self.resolution),
            math.floor((point[1] - self.origin_y) / self.resolution),
        )

    def cell_to_world(self, cell: Cell) -> Point:
        return (
            self.origin_x + (cell[0] + 0.5) * self.resolution,
            self.origin_y + (cell[1] + 0.5) * self.resolution,
        )


def reproject_cells(
    source: GridMap,
    target: GridMap,
    cells: Sequence[Cell],
) -> List[Cell]:
    """Re-express grid cells in a map with different geometry."""
    projected: List[Cell] = []
    for cell in cells:
        target_cell = target.world_to_cell(source.cell_to_world(cell))
        if not target.in_bounds(target_cell):
            return []
        if not projected or projected[-1] != target_cell:
            projected.append(target_cell)
    return projected


@dataclass
class CompletionMonitor:
    """Track repeated no-goal observations while the known map is stable."""

    confirm_count: int
    stable_duration: float
    last_area: Optional[int] = None
    stable_since: Optional[float] = None
    no_goal_count: int = 0

    def reset(self) -> None:
        self.last_area = None
        self.stable_since = None
        self.no_goal_count = 0

    def observe(self, now: float, area: int, goal_available: bool) -> bool:
        """Return true after reachable goals remain absent on a stable map."""
        if self.last_area is None or area != self.last_area:
            self.last_area = area
            self.stable_since = now
            self.no_goal_count = 0

        if goal_available:
            self.no_goal_count = 0
            return False

        self.no_goal_count += 1
        stable_since = (
            self.stable_since if self.stable_since is not None else now
        )
        return (
            self.no_goal_count >= max(1, self.confirm_count)
            and now - stable_since >= max(0.0, self.stable_duration)
        )


def is_free(grid: GridMap, cell: Cell, threshold: int = 65) -> bool:
    """Return whether a cell is known and below the occupied threshold."""
    return grid.in_bounds(cell) and 0 <= grid.value(cell) < threshold


def is_unknown(grid: GridMap, cell: Cell) -> bool:
    return grid.in_bounds(cell) and grid.value(cell) < 0


def neighbors(
    cell: Cell,
    diagonal: bool = True,
) -> Iterable[Tuple[Cell, float]]:
    directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]
    if diagonal:
        directions.extend([(1, 1), (1, -1), (-1, 1), (-1, -1)])
    for dx, dy in directions:
        yield (
            (cell[0] + dx, cell[1] + dy),
            math.sqrt(2.0) if dx and dy else 1.0,
        )


def transition_is_safe(
    grid: GridMap,
    current: Cell,
    target: Cell,
    blocked: Set[Cell],
    threshold: int = 65,
) -> bool:
    """Check a one-cell move, including diagonal corner clearance."""
    dx = target[0] - current[0]
    dy = target[1] - current[1]
    if max(abs(dx), abs(dy)) != 1:
        return False
    if target in blocked or not is_free(grid, target, threshold):
        return False
    if dx and dy:
        side_x = (current[0] + dx, current[1])
        side_y = (current[0], current[1] + dy)
        if (
            side_x in blocked
            or side_y in blocked
            or not is_free(grid, side_x, threshold)
            or not is_free(grid, side_y, threshold)
        ):
            return False
    return True


def inflate_obstacles(
    grid: GridMap,
    radius: float,
    threshold: int = 65,
) -> Set[Cell]:
    """Return occupied cells inflated by the supplied metric radius."""
    radius_cells = math.ceil(max(0.0, radius) / grid.resolution)
    occupied = {
        (x, y)
        for y in range(grid.height)
        for x in range(grid.width)
        if grid.value((x, y)) >= threshold
    }
    inflated: Set[Cell] = set()
    for obstacle_x, obstacle_y in occupied:
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                if dx * dx + dy * dy > radius_cells * radius_cells:
                    continue
                candidate = (obstacle_x + dx, obstacle_y + dy)
                if grid.in_bounds(candidate):
                    inflated.add(candidate)
    return inflated


def reachable_free(
    grid: GridMap,
    start: Cell,
    blocked: Set[Cell],
    threshold: int = 65,
) -> Set[Cell]:
    """Flood-fill safely reachable known free cells."""
    if not is_free(grid, start, threshold) or start in blocked:
        return set()

    seen = {start}
    queue = deque([start])
    while queue:
        current = queue.popleft()
        for target, _ in neighbors(current):
            if target in seen:
                continue
            if transition_is_safe(grid, current, target, blocked, threshold):
                seen.add(target)
                queue.append(target)
    return seen


def detect_frontiers(
    grid: GridMap,
    start: Cell,
    blocked: Set[Cell],
    min_size: int = 5,
    threshold: int = 65,
    connectivity: int = 8,
    reachable: Optional[Set[Cell]] = None,
) -> List[Set[Cell]]:
    """Detect connected free cells bordering unknown space."""
    if reachable is None:
        reachable = reachable_free(grid, start, blocked, threshold)
    frontier_cells = {
        cell
        for cell in reachable
        if any(is_unknown(grid, neighbor) for neighbor, _ in neighbors(cell))
    }

    clusters: List[Set[Cell]] = []
    while frontier_cells:
        root = frontier_cells.pop()
        cluster = {root}
        queue = deque([root])
        while queue:
            current = queue.popleft()
            for target, _ in neighbors(current, connectivity == 8):
                if target in frontier_cells:
                    frontier_cells.remove(target)
                    cluster.add(target)
                    queue.append(target)
        if len(cluster) >= max(1, min_size):
            clusters.append(cluster)
    return clusters


def _nearest_reachable_candidate(
    grid: GridMap,
    desired: Cell,
    frontier_cell: Cell,
    blocked: Set[Cell],
    reachable: Optional[Set[Cell]],
    threshold: int,
    search_radius: int,
) -> Optional[Cell]:
    candidates: List[Tuple[float, Cell]] = []
    for dy in range(-search_radius, search_radius + 1):
        for dx in range(-search_radius, search_radius + 1):
            cell = (desired[0] + dx, desired[1] + dy)
            if cell in blocked or not is_free(grid, cell, threshold):
                continue
            if reachable is not None and cell not in reachable:
                continue
            distance = math.hypot(dx, dy)
            frontier_distance = math.hypot(
                cell[0] - frontier_cell[0],
                cell[1] - frontier_cell[1],
            )
            candidates.append((distance + 0.05 * frontier_distance, cell))
    return min(candidates, default=(0.0, None), key=lambda item: item[0])[1]


def candidate_goals(
    grid: GridMap,
    frontier: Set[Cell],
    blocked: Set[Cell],
    offset: float = 0.3,
    threshold: int = 65,
    max_candidates: int = 12,
    reachable: Optional[Set[Cell]] = None,
) -> List[Cell]:
    """Generate safe goals offset into reachable free space."""
    if not frontier:
        return []

    center_x = sum(cell[0] for cell in frontier) / len(frontier)
    center_y = sum(cell[1] for cell in frontier) / len(frontier)
    offset_cells = max(1, round(offset / grid.resolution))
    # Sample the whole frontier, not only cells nearest its centroid.  A long
    # frontier can pass beside the robot while extending into unexplored
    # space; centroid-only sampling then produces goals already inside the
    # arrival tolerance and the robot never advances.  Farthest-point
    # sampling gives the scorer near, middle, and distant alternatives.
    sample_limit = min(len(frontier), max_candidates * 4)
    # Bound the input to farthest-point sampling.  SLAM can produce a single
    # frontier containing thousands of cells; repeatedly comparing every cell
    # with every selected sample can then consume the whole planning timeout.
    # A deterministic, evenly spaced pre-sample retains coverage along the
    # frontier while keeping planning latency predictable.
    ordered_frontier = sorted(frontier)
    pool_limit = max(64, max_candidates * 8)
    if len(ordered_frontier) > pool_limit:
        last = len(ordered_frontier) - 1
        pool = {
            ordered_frontier[round(index * last / (pool_limit - 1))]
            for index in range(pool_limit)
        }
    else:
        pool = set(ordered_frontier)
    remaining = pool
    first = min(
        remaining,
        key=lambda cell: (
            (cell[0] - center_x) ** 2 + (cell[1] - center_y) ** 2,
            cell,
        ),
    )
    ordered = [first]
    remaining.remove(first)
    while remaining and len(ordered) < sample_limit:
        next_cell = max(
            remaining,
            key=lambda cell: (
                min(
                    (cell[0] - chosen[0]) ** 2
                    + (cell[1] - chosen[1]) ** 2
                    for chosen in ordered
                ),
                cell,
            ),
        )
        ordered.append(next_cell)
        remaining.remove(next_cell)
    goals: Set[Cell] = set()

    for frontier_cell in ordered:
        unknown_neighbors = [
            cell
            for cell, _ in neighbors(frontier_cell)
            if is_unknown(grid, cell)
        ]
        if unknown_neighbors:
            unknown_x = sum(cell[0] for cell in unknown_neighbors) / len(
                unknown_neighbors
            )
            unknown_y = sum(cell[1] for cell in unknown_neighbors) / len(
                unknown_neighbors
            )
            away_x = frontier_cell[0] - unknown_x
            away_y = frontier_cell[1] - unknown_y
        else:
            away_x = frontier_cell[0] - center_x
            away_y = frontier_cell[1] - center_y

        magnitude = math.hypot(away_x, away_y)
        if magnitude < 1e-9:
            desired = frontier_cell
        else:
            desired = (
                round(frontier_cell[0] + away_x / magnitude * offset_cells),
                round(frontier_cell[1] + away_y / magnitude * offset_cells),
            )

        goal = _nearest_reachable_candidate(
            grid,
            desired,
            frontier_cell,
            blocked,
            reachable,
            threshold,
            max(2, offset_cells),
        )
        if goal is not None:
            goals.add(goal)
        if len(goals) >= max_candidates:
            break

    return sorted(
        goals,
        key=lambda cell: (
            (cell[0] - center_x) ** 2 + (cell[1] - center_y) ** 2
        ),
    )[:max_candidates]


def bounded_candidate_batch(
    groups: Sequence[Sequence[Cell]],
    limit: int,
) -> List[Cell]:
    """Select a unique, round-robin candidate batch with a hard limit."""
    selected: List[Cell] = []
    seen: Set[Cell] = set()
    for rank in range(max(0, limit)):
        for group in groups:
            if rank >= len(group):
                continue
            candidate = group[rank]
            if candidate in seen:
                continue
            seen.add(candidate)
            selected.append(candidate)
            if len(selected) >= limit:
                return selected
    return selected


def astar(
    grid: GridMap,
    start: Cell,
    goal: Cell,
    blocked: Set[Cell],
    threshold: int = 65,
) -> Optional[List[Cell]]:
    """Plan through known free space using safe eight-connected A*."""
    if (
        not is_free(grid, start, threshold)
        or not is_free(grid, goal, threshold)
        or start in blocked
        or goal in blocked
    ):
        return None

    queue = [(0.0, 0.0, start)]
    came_from: Dict[Cell, Cell] = {}
    costs = {start: 0.0}

    while queue:
        _, current_cost, current = heapq.heappop(queue)
        if current_cost > costs.get(current, float('inf')) + 1e-9:
            continue
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return list(reversed(path))

        for target, step_cost in neighbors(current):
            if not transition_is_safe(
                grid,
                current,
                target,
                blocked,
                threshold,
            ):
                continue
            new_cost = costs[current] + step_cost
            if new_cost >= costs.get(target, float('inf')):
                continue
            costs[target] = new_cost
            came_from[target] = current
            heuristic = math.hypot(
                goal[0] - target[0],
                goal[1] - target[1],
            )
            heapq.heappush(
                queue,
                (new_cost + heuristic, new_cost, target),
            )
    return None


def _grid_line(start: Cell, end: Cell) -> List[Cell]:
    """Return a dense integer line suitable for transition validation."""
    steps = max(abs(end[0] - start[0]), abs(end[1] - start[1]))
    if steps == 0:
        return [start]
    cells: List[Cell] = []
    for index in range(steps + 1):
        ratio = index / steps
        cell = (
            round(start[0] + (end[0] - start[0]) * ratio),
            round(start[1] + (end[1] - start[1]) * ratio),
        )
        if not cells or cell != cells[-1]:
            cells.append(cell)
    return cells


def line_is_safe(
    grid: GridMap,
    start: Cell,
    end: Cell,
    blocked: Set[Cell],
    threshold: int = 65,
) -> bool:
    """Validate every cell and diagonal transition on a straight segment."""
    cells = _grid_line(start, end)
    if any(
        cell in blocked or not is_free(grid, cell, threshold)
        for cell in cells
    ):
        return False
    return all(
        transition_is_safe(grid, first, second, blocked, threshold)
        for first, second in zip(cells, cells[1:])
    )


def simplify_path(
    grid: GridMap,
    path: Sequence[Cell],
    blocked: Set[Cell],
    threshold: int = 65,
) -> List[Cell]:
    """Remove unnecessary waypoints without relaxing collision checks."""
    if len(path) < 3:
        return list(path)

    simplified = [path[0]]
    anchor = 0
    while anchor < len(path) - 1:
        furthest = anchor + 1
        for index in range(anchor + 2, len(path)):
            if line_is_safe(
                grid,
                path[anchor],
                path[index],
                blocked,
                threshold,
            ):
                furthest = index
            else:
                break
        simplified.append(path[furthest])
        anchor = furthest
    return simplified


def round_path_corners(
    grid: GridMap,
    path: Sequence[Cell],
    blocked: Set[Cell],
    threshold: int = 65,
    fraction: float = 0.35,
) -> List[Cell]:
    """Cut safe corners from a polyline to reduce stop-and-turn motion."""
    if len(path) < 3:
        return list(path)
    fraction = max(0.05, min(0.45, float(fraction)))
    rounded = [path[0]]

    for index in range(1, len(path) - 1):
        previous = path[index - 1]
        corner = path[index]
        following = path[index + 1]
        entry = (
            round(corner[0] + (previous[0] - corner[0]) * fraction),
            round(corner[1] + (previous[1] - corner[1]) * fraction),
        )
        exit_cell = (
            round(corner[0] + (following[0] - corner[0]) * fraction),
            round(corner[1] + (following[1] - corner[1]) * fraction),
        )
        if (
            entry != exit_cell
            and line_is_safe(
                grid, rounded[-1], entry, blocked, threshold
            )
            and line_is_safe(
                grid, entry, exit_cell, blocked, threshold
            )
        ):
            for candidate in (entry, exit_cell):
                if rounded[-1] != candidate:
                    rounded.append(candidate)
        elif rounded[-1] != corner:
            rounded.append(corner)

    if rounded[-1] != path[-1]:
        rounded.append(path[-1])
    if path_is_safe(grid, rounded, blocked, threshold):
        return rounded
    return list(path)


def path_is_safe(
    grid: GridMap,
    path: Sequence[Cell],
    blocked: Set[Cell],
    threshold: int = 65,
) -> bool:
    if not path:
        return False
    return (
        path[0] not in blocked
        and is_free(grid, path[0], threshold)
        and all(
            line_is_safe(grid, first, second, blocked, threshold)
            for first, second in zip(path, path[1:])
        )
    )


def path_cost(path: Optional[Sequence[Cell]]) -> float:
    if not path:
        return float('inf')
    return sum(
        math.hypot(second[0] - first[0], second[1] - first[1])
        for first, second in zip(path, path[1:])
    )


def information_gain(
    grid: GridMap,
    goal: Cell,
    radius_cells: int = 8,
) -> int:
    return sum(
        1
        for dy in range(-radius_cells, radius_cells + 1)
        for dx in range(-radius_cells, radius_cells + 1)
        if dx * dx + dy * dy <= radius_cells * radius_cells
        and is_unknown(grid, (goal[0] + dx, goal[1] + dy))
    )


def obstacle_clearance_risk(
    path: Sequence[Cell],
    blocked: Set[Cell],
    radius_cells: int = 4,
) -> float:
    """Estimate risk from the nearest inflated obstacle around path samples."""
    if not path or not blocked:
        return 0.0
    sampled = path[:: max(1, len(path) // 20)]
    risks = []
    for cell in sampled:
        nearest = min(
            (
                math.hypot(cell[0] - obstacle[0], cell[1] - obstacle[1])
                for obstacle in blocked
                if abs(cell[0] - obstacle[0]) <= radius_cells
                and abs(cell[1] - obstacle[1]) <= radius_cells
            ),
            default=float('inf'),
        )
        risks.append(
            0.0 if math.isinf(nearest) else 1.0 / max(nearest, 0.5)
        )
    return sum(risks) / len(risks)


def known_area(grid: GridMap) -> int:
    return sum(value >= 0 for value in grid.data)


def initial_scan_velocity(speed: float) -> Tuple[float, float]:
    """Return a safe in-place scan command as (linear, angular)."""
    return 0.0, abs(float(speed))


def normalize_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def scan_directional_minima(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    front_half_angle: float = math.pi / 6.0,
    side_half_angle: float = math.pi / 2.0,
) -> Optional[Tuple[float, float, float, float]]:
    """Return (left, front, right, rear) directional scan minima."""
    left: List[float] = []
    front: List[float] = []
    right: List[float] = []
    rear: List[float] = []
    for index, distance in enumerate(ranges):
        if not math.isfinite(distance):
            continue
        if distance < range_min or distance > range_max:
            continue
        angle = normalize_angle(angle_min + index * angle_increment)
        if abs(angle) <= front_half_angle:
            front.append(distance)
        elif front_half_angle < angle <= side_half_angle:
            left.append(distance)
        elif -side_half_angle <= angle < -front_half_angle:
            right.append(distance)
        elif abs(abs(angle) - math.pi) <= front_half_angle:
            rear.append(distance)

    if not front and not left and not right and not rear:
        return None
    return (
        min(left, default=float('inf')),
        min(front, default=float('inf')),
        min(right, default=float('inf')),
        min(rear, default=float('inf')),
    )


def scan_sector_minima(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    front_half_angle: float = math.pi / 6.0,
    side_half_angle: float = math.pi / 2.0,
) -> Optional[Tuple[float, float, float]]:
    """Return (left, front, right) minima for either scan convention."""
    directional = scan_directional_minima(
        ranges,
        angle_min,
        angle_increment,
        range_min,
        range_max,
        front_half_angle,
        side_half_angle,
    )
    if directional is None:
        return None
    return directional[:3]
