from drone_map.collision_geometry import AABB
from drone_map.collision_geometry import StaticMap
from drone_map.random_map import generate_validated_map
from drone_planner.astar_3d import AStar3D


def _validator(payload, start, goal):
    data = payload["map"]
    obstacles = tuple(
        AABB.from_center_size(
            item["id"],
            (
                item["center_x"],
                item["center_y"],
                item["center_z"],
            ),
            (item["size_x"], item["size_y"], item["size_z"]),
        )
        for item in data["obstacles"]
    )
    static_map = StaticMap(
        (data["x_min"], data["y_min"], data["z_min"]),
        (data["x_max"], data["y_max"], data["z_max"]),
        data["drone_radius"],
        data["safety_margin"],
        data["grid_resolution"],
        obstacles,
    )
    return AStar3D(
        static_map,
        planning_timeout_sec=2.0,
        minimum_z=0.8,
    ).plan(start, goal).success


def test_random_generation_is_seeded_and_astar_validated():
    arguments = (42, 5, (0.0, 0.0, 1.2), (5.0, 0.0, 1.5), _validator)
    first = generate_validated_map(*arguments)
    second = generate_validated_map(*arguments)
    assert first == second
    assert _validator(
        first,
        (0.0, 0.0, 1.2),
        (5.0, 0.0, 1.5),
    )
