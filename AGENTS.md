# AGENTS.md

## Scope

These instructions apply to the entire repository. Keep this file limited to stable project conventions. Do not add sprint status, one-off requests, temporary test results, personal paths, copied third-party documentation, or information likely to change frequently.

## Project purpose

This repository implements a small standalone ROS2 quadrotor simulator. The required core is:

- four-motor RPM input;
- 6DoF rigid-body dynamics;
- position and attitude control;
- odometry, IMU, TF, path, and actual RPM outputs;
- RViz visualization;
- repeatable automated acceptance tests.

Static maps and path planning are optional extensions. Never describe placeholder packages as implemented features.

## Supported stack

- Ubuntu 22.04
- ROS2 Humble
- `colcon` with `ament_cmake` and `ament_python`
- C++17 for dynamics and control
- Python 3 with `rclpy` for bringup, visualization helpers, and test orchestration
- Eigen3 for vector, matrix, and quaternion math
- RViz2, URDF, Robot State Publisher, and tf2
- YAML for runtime parameters

Do not introduce Gazebo, another simulator, a GUI framework, custom message packages, or a planning library without a task that requires the architectural change.

## Repository layout

- `src/drone_dynamics`: motor response, rigid-body dynamics, state publishers, dynamic TF.
- `src/drone_controller`: position control, desired attitude, geometric attitude control, mixer.
- `src/drone_bringup`: parameters, launch files, URDF, RViz, goal marker, acceptance tooling.
- `src/drone_map`: optional map package; treat as unimplemented until it contains a tested node.
- `src/drone_planner`: optional planning package; treat as unimplemented until it contains a tested node.
- `src/drone_msgs`: reserved for justified custom interfaces; do not add messages merely to replace a stable standard message.
- `docs`: architecture, execution, testing, report, and evidence documents.
- `scripts`: repeatable operator commands.
- `output`: final generated artifacts only.
- `build`, `install`, `log`: local generated directories; never commit them.

## Sources of truth

Use this precedence when facts conflict:

1. Executed tests from the current commit and parameter set.
2. Source code and launch files.
3. `src/drone_bringup/config/vertical_sim.yaml`.
4. Current repository documentation.
5. Historical handoff notes.

Never copy a historical metric into README or a report unless it is reproduced and linked to evidence from the current commit.

## Stable interface contract

Preserve these interfaces unless the task explicitly authorizes a breaking change:

- `/drone/goal`: `geometry_msgs/msg/PoseStamped`
- `/drone/motor_rpm_cmd`: `std_msgs/msg/Float32MultiArray`, exactly four RPM values
- `/drone/motor_rpm`: `std_msgs/msg/Float32MultiArray`, exactly four RPM values
- `/drone/odom`: `nav_msgs/msg/Odometry`
- `/drone/imu`: `sensor_msgs/msg/Imu`
- `/drone/path`: `nav_msgs/msg/Path`
- `/drone/desired_pose`: `geometry_msgs/msg/PoseStamped`
- `/drone/goal_marker`: `visualization_msgs/msg/Marker`
- dynamic TF: `map -> base_link`
- fixed URDF transform: `base_link -> body_link`

The motor order is invariant:

1. M1 front-left
2. M2 front-right
3. M3 rear-right
4. M4 rear-left

Any change to topic names, message types, frames, motor order, units, or QoS requires synchronized code, launch, RViz, tests, README, and architecture updates.

## Units and mathematical conventions

- Use SI units internally: metres, seconds, kilograms, radians, newtons, and newton-metres.
- The motor topic boundary uses RPM. Convert RPM to rad/s exactly once before applying thrust and moment coefficients.
- `orientation` represents body-to-world rotation.
- World gravity points along negative z.
- Odometry linear velocity is world-frame velocity; angular velocity is body-frame angular velocity.
- Normalize input and integrated quaternions. Reject or safely handle non-finite values.
- Keep shared physical values identical between dynamics, controller, YAML, URDF, tests, and documentation.
- Do not change force, torque, quaternion, mixer, or motor-spin signs without dedicated sign tests.
- State the integration method explicitly when changing it; numerical changes require regression evidence.

## C++ conventions

- Use C++17 explicitly in every CMake package that compiles C++.
- Follow ROS2 C++ style and keep `-Wall -Wextra -Wpedantic`.
- Prefer small pure functions or classes for dynamics and control math so they can be tested without a ROS graph.
- Use `Eigen::Vector3d`, `Eigen::Matrix3d`, and `Eigen::Quaterniond` consistently for 3D math.
- Use `std::array<double, 4>` for fixed four-motor data inside C++.
- Use `std::clamp` for hard limits and validate parameter ranges at node startup.
- Check every external numeric input with `std::isfinite`.
- Avoid heap allocation in high-rate update loops.
- Use ROS throttled logging for high-rate nodes; never log at 100-200 Hz without throttling.
- Keep callbacks and timer functions short. Move reusable math out of ROS message handling.

## Python conventions

- Follow PEP 8 and the existing ament flake8/pep257 checks.
- Use four spaces, type hints, descriptive names, and docstrings for public classes/functions.
- Keep launch files declarative; move reusable launch parameter conversion into small helpers.
- Use `pathlib.Path` for paths in new Python code.
- Avoid `os._exit()` unless a process-lifecycle test proves it is required and logs are flushed.
- Do not use sleeps as synchronization when ROS graph discovery, events, or explicit readiness checks are available.
- Test both successful shutdown and failure shutdown paths.

## YAML, launch, URDF, and RViz conventions

- Keep one canonical default parameter file unless a scenario genuinely requires an override.
- Use node-name keys under `ros__parameters`.
- Add units to parameter documentation and include units in names when ambiguity is likely.
- Launch arguments must have descriptions and explicit value types.
- New launch files should compose existing launch descriptions instead of duplicating the same node list.
- Keep URDF mass and inertia synchronized with dynamics defaults.
- RViz topics and frames must match the interface contract.
- A visualization-only object must not be used as the sole collision representation for planning.

## Required workflow for changes

1. Inspect `git status` and preserve unrelated user changes.
2. Identify the smallest affected package and interface surface.
3. Add or update the narrowest useful test before or with the implementation.
4. Build the affected package.
5. Run its unit/lint tests.
6. Run the core acceptance test for any runtime, parameter, launch, dynamics, controller, topic, TF, or dependency change.
7. Run the full suite before release or when shared interfaces change.
8. Update documentation when behavior, parameters, interfaces, commands, or results change.
9. Review `git diff` and confirm no generated artifacts are included.

Do not combine unrelated algorithm, formatting, documentation, and feature changes in one commit.

## Canonical commands

Run commands from an Ubuntu bash shell:

```bash
cd ~/ros2_quadrotor_sim
source /opt/ros/humble/setup.bash

colcon build --symlink-install
source install/setup.bash

colcon test
colcon test-result --verbose

ros2 launch drone_bringup sim.launch.py
ros2 launch drone_bringup sim.launch.py rviz:=false
ros2 launch drone_bringup acceptance_test.launch.py
```

For a package-scoped iteration:

```bash
colcon build --symlink-install --packages-select <package>
source install/setup.bash
colcon test --packages-select <package>
colcon test-result --verbose
```

Do not run ROS2 commands before sourcing both Humble and the workspace overlay.

## Test policy

### Always

- Build every changed package.
- Run its lint and unit tests.
- Preserve test logs for failures until the root cause is understood.

### Core acceptance required

Run the core acceptance test after changes to:

- dynamics or controller code;
- shared physical parameters;
- message topics, types, QoS, or frames;
- launch and process lifecycle;
- dependency declarations that affect runtime;
- URDF inertia or transform semantics.

### Full regression required

Run the full suite after:

- a shared interface change;
- changes spanning more than one package;
- map/planner integration;
- scenario or acceptance criteria changes;
- release preparation.

Acceptance success must be confirmed by the explicit `ACCEPTANCE TEST: PASS` record, the acceptance process result, and the wrapper/CI result. Do not infer success from the outer launch command's exit code alone.

## Known traps

- Dynamics and controller duplicate several physical parameters. A mismatch can make a stable controller fail without a compile error.
- Thrust and moment coefficients operate on angular velocity squared in rad/s, while ROS topics use RPM.
- Mixer and dynamics motor signs are coupled. A local “sign cleanup” can invert roll, pitch, or yaw.
- A four-element `Float32MultiArray` has no schema; validate length and document order everywhere.
- A zero goal quaternion is treated as yaw zero. Do not silently reinterpret it differently in another node.
- The controller intentionally outputs zero RPM until it has both goal and odometry.
- `map -> base_link` comes from dynamics; `base_link -> body_link` comes from Robot State Publisher. Publishing either transform twice creates TF conflicts.
- Wall timers are used. Simulated-time assumptions are invalid unless `/clock` support is explicitly added.
- Current IMU data is idealized and has no noise, bias, latency, or covariance model.
- Independent motor saturation changes the requested force/torque balance.
- Python copyright tests generated by ROS templates may be skipped; a green summary can still include skipped compliance checks.
- Placeholder packages can build successfully while providing no runtime capability.
- Build success and lint success do not prove the flight control loop is working.

## Safety and scope rules

- Preserve core dynamics and control behavior unless the task explicitly requests an algorithmic change.
- Never hide a failing scenario by loosening tolerances without a documented engineering reason.
- Never fabricate a result, graph, topic, parameter, feature, citation, or comparison.
- Never commit secrets, personal access tokens, ROS bags with sensitive data, or machine-specific absolute paths.
- Never delete `build/install/log` outside the resolved repository root.
- Avoid adding heavy dependencies for a feature that can be implemented with the existing stack.
- Optional map/planner work must remain disableable and must not break the mandatory core simulator.

## Documentation rules

- Use Chinese for project-facing narrative unless a deliverable requires English.
- Keep code identifiers, topic names, frame names, message types, commands, and units in their original technical form.
- Link every reported metric to a reproducible command and raw artifact.
- Label historical results as historical until reproduced.
- Keep `docs/ARCHITECTURE_OVERVIEW.md` synchronized with runtime interfaces.
- Keep `docs/PROJECT_EXECUTION_PLAN.md` as the phase checklist and evidence index.
- Follow `docs/WORD_WRITING_GUIDE.md` for Word/report formatting.
- If a Word template becomes available, treat it as the formatting authority and update the writing guide from measured template properties.

## Git hygiene

- Use focused branches prefixed with `codex/` when Codex creates a branch.
- Use conventional commit prefixes such as `feat:`, `fix:`, `test:`, `docs:`, `refactor:`, and `chore:`.
- Do not commit `build/`, `install/`, `log/`, `__pycache__/`, `*.pyc`, editor state, temporary renders, or LaTeX auxiliary files.
- Keep final requested PDFs under `output/pdf/`; keep temporary render images under `tmp/`.
- Do not rewrite user history, force-push, or discard unrelated changes without explicit authorization.
