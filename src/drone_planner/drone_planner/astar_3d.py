"""ROS-independent 26-neighbour three-dimensional grid A* planner."""

from __future__ import annotations

from dataclasses import dataclass
import heapq
import itertools
import math
import time
from typing import Dict, Iterator, Sequence, Tuple

from drone_map.collision_geometry import Point3
from drone_map.collision_geometry import StaticMap

GridIndex = Tuple[int, int, int]


@dataclass(frozen=True)
class PlanningResult:
    """Structured A* success/failure result and reproducible metrics."""

    success: bool
    reason: str
    path: Tuple[Point3, ...]
    raw_path_points: int
    expanded_nodes: int
    planning_time_sec: float
    raw_path_length: float


def path_length(path: Sequence[Sequence[float]]) -> float:
    """Return the Euclidean length of a polyline."""
    return sum(
        math.dist(first, second)
        for first, second in zip(path, path[1:])
    )


class AStar3D:
    """Bounded A* search over grid-cell centres with 26 connectivity."""

    def __init__(
        self,
        static_map: StaticMap,
        max_expanded_nodes: int = 100000,
        planning_timeout_sec: float = 2.0,
    ) -> None:
        if max_expanded_nodes <= 0:
            raise ValueError("max_expanded_nodes must be positive")
        if (
            not math.isfinite(planning_timeout_sec)
            or planning_timeout_sec <= 0.0
        ):
            raise ValueError("planning_timeout_sec must be finite and positive")
        self.static_map = static_map
        self.resolution = static_map.grid_resolution
        self.max_expanded_nodes = max_expanded_nodes
        self.planning_timeout_sec = planning_timeout_sec
        self.shape = tuple(
            int(math.ceil((upper - lower) / self.resolution))
            for lower, upper in zip(static_map.minimum, static_map.maximum)
        )
        self._motions = tuple(
            (
                (dx, dy, dz),
                self.resolution * math.sqrt(dx * dx + dy * dy + dz * dz),
            )
            for dx, dy, dz in itertools.product((-1, 0, 1), repeat=3)
            if (dx, dy, dz) != (0, 0, 0)
        )

    def world_to_grid(self, point: Sequence[float]) -> GridIndex:
        """
        Map a world point to its containing cell.

        Cell index zero starts at the lower map bound. The exact upper bound
        belongs to the final cell so both inclusive map faces are representable.
        """
        checked = tuple(float(value) for value in point)
        if len(checked) != 3 or not all(math.isfinite(v) for v in checked):
            raise ValueError("world point must contain three finite values")
        if not self.static_map.contains_point(checked):
            raise ValueError("world point is outside map bounds")
        result = []
        for index in range(3):
            cell = int(
                math.floor(
                    (checked[index] - self.static_map.minimum[index])
                    / self.resolution
                )
            )
            result.append(min(self.shape[index] - 1, cell))
        return result[0], result[1], result[2]

    def grid_to_world(self, index: Sequence[int]) -> Point3:
        """Return the centre coordinate of a valid grid cell."""
        if len(index) != 3:
            raise ValueError("grid index must contain three values")
        checked = tuple(int(value) for value in index)
        if not self._valid_index(checked):
            raise ValueError("grid index is outside map bounds")
        point = tuple(
            self.static_map.minimum[axis]
            + (checked[axis] + 0.5) * self.resolution
            for axis in range(3)
        )
        return point  # type: ignore[return-value]

    def plan(
        self,
        start: Sequence[float],
        goal: Sequence[float],
    ) -> PlanningResult:
        """Plan from exact world ``start`` to exact world ``goal``."""
        started = time.monotonic()
        checked_start = self._validate_input(start, "START")
        if isinstance(checked_start, str):
            return self._failure(checked_start, 0, started)
        checked_goal = self._validate_input(goal, "GOAL")
        if isinstance(checked_goal, str):
            return self._failure(checked_goal, 0, started)

        start_index = self.world_to_grid(checked_start)
        goal_index = self.world_to_grid(checked_goal)
        start_center = self.grid_to_world(start_index)
        goal_center = self.grid_to_world(goal_index)
        if not self.static_map.is_traversable(start_center):
            return self._failure("START_CELL_OCCUPIED", 0, started)
        if not self.static_map.is_traversable(goal_center):
            return self._failure("GOAL_CELL_OCCUPIED", 0, started)
        if not self.static_map.segment_is_collision_free(
            checked_start, start_center
        ):
            return self._failure("START_CELL_DISCONNECTED", 0, started)
        if not self.static_map.segment_is_collision_free(
            goal_center, checked_goal
        ):
            return self._failure("GOAL_CELL_DISCONNECTED", 0, started)

        frontier = []
        sequence = itertools.count()
        heapq.heappush(
            frontier,
            (
                self._heuristic(start_index, goal_index),
                next(sequence),
                start_index,
            ),
        )
        cost_so_far: Dict[GridIndex, float] = {start_index: 0.0}
        parent: Dict[GridIndex, GridIndex] = {}
        closed = set()
        expanded = 0

        while frontier:
            if time.monotonic() - started > self.planning_timeout_sec:
                return self._failure("TIMEOUT", expanded, started)
            _, _, current = heapq.heappop(frontier)
            if current in closed:
                continue
            if current == goal_index:
                path = self._reconstruct(
                    current,
                    parent,
                    checked_start,
                    checked_goal,
                )
                return PlanningResult(
                    success=True,
                    reason="SUCCESS",
                    path=path,
                    raw_path_points=len(path),
                    expanded_nodes=expanded,
                    planning_time_sec=time.monotonic() - started,
                    raw_path_length=path_length(path),
                )
            closed.add(current)
            expanded += 1
            if expanded > self.max_expanded_nodes:
                return self._failure("MAX_EXPANSIONS", expanded, started)

            current_world = self.grid_to_world(current)
            for neighbour, movement_cost in self._neighbours(current):
                if neighbour in closed:
                    continue
                neighbour_world = self.grid_to_world(neighbour)
                if not self.static_map.is_traversable(neighbour_world):
                    continue
                if not self.static_map.segment_is_collision_free(
                    current_world, neighbour_world
                ):
                    continue
                candidate = cost_so_far[current] + movement_cost
                if candidate >= cost_so_far.get(neighbour, math.inf):
                    continue
                cost_so_far[neighbour] = candidate
                parent[neighbour] = current
                priority = candidate + self._heuristic(
                    neighbour, goal_index
                )
                heapq.heappush(
                    frontier,
                    (priority, next(sequence), neighbour),
                )

        return self._failure("NO_PATH", expanded, started)

    def _validate_input(
        self,
        point: Sequence[float],
        role: str,
    ) -> Point3 | str:
        try:
            checked = tuple(float(value) for value in point)
        except (TypeError, ValueError):
            return f"{role}_NON_FINITE"
        if len(checked) != 3 or not all(math.isfinite(v) for v in checked):
            return f"{role}_NON_FINITE"
        if not self.static_map.contains_point(checked):
            return f"{role}_OUT_OF_BOUNDS"
        if self.static_map.is_occupied(checked):
            return f"{role}_OCCUPIED"
        return checked  # type: ignore[return-value]

    def _valid_index(self, index: GridIndex) -> bool:
        return all(
            0 <= index[axis] < self.shape[axis]
            for axis in range(3)
        )

    def _neighbours(
        self,
        current: GridIndex,
    ) -> Iterator[Tuple[GridIndex, float]]:
        for offset, cost in self._motions:
            neighbour = (
                current[0] + offset[0],
                current[1] + offset[1],
                current[2] + offset[2],
            )
            if self._valid_index(neighbour):
                yield neighbour, cost

    def _heuristic(self, first: GridIndex, second: GridIndex) -> float:
        return self.resolution * math.sqrt(
            sum(
                (first[axis] - second[axis]) ** 2
                for axis in range(3)
            )
        )

    def _reconstruct(
        self,
        current: GridIndex,
        parent: Dict[GridIndex, GridIndex],
        start: Point3,
        goal: Point3,
    ) -> Tuple[Point3, ...]:
        indices = [current]
        while current in parent:
            current = parent[current]
            indices.append(current)
        indices.reverse()
        centres = [self.grid_to_world(index) for index in indices]
        points = [start]
        for centre in centres:
            if math.dist(points[-1], centre) > 1.0e-12:
                points.append(centre)
        if math.dist(points[-1], goal) > 1.0e-12:
            points.append(goal)
        else:
            points[-1] = goal
        return tuple(points)

    @staticmethod
    def _failure(
        reason: str,
        expanded_nodes: int,
        started: float,
    ) -> PlanningResult:
        return PlanningResult(
            success=False,
            reason=reason,
            path=(),
            raw_path_points=0,
            expanded_nodes=expanded_nodes,
            planning_time_sec=time.monotonic() - started,
            raw_path_length=0.0,
        )
