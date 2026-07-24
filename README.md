# ROS2 四旋翼无人机仿真与规划展示系统

本仓库实现一个不依赖 Gazebo 的 ROS2 Humble 四旋翼闭环仿真：C++ 6DoF
动力学与位置/姿态控制器负责飞行，Python 节点负责静态三维地图、3D A*、
任务管理、自动验收和 PyQt5 地面站。当前版本只支持单机。

## 功能矩阵

| 能力 | 状态 | 入口 |
|---|---|---|
| 四电机与 6DoF 刚体动力学 | 已实现 | `drone_dynamics` |
| 位置/姿态控制与 mixer | 已实现 | `drone_controller` |
| 3D AABB 地图与膨胀碰撞 | 已实现 | `drone_map` |
| 26 邻域 3D A* | 已实现 | `drone_planner` |
| 安全平滑、重采样、前视执行 | 已实现 | `planner_node` |
| 单目标与多航点任务 | 已实现 | `mission_manager_node` |
| hover/point/square/circle/figure-eight | 已实现 | `trajectories.py` |
| RViz 原始/膨胀障碍物与可选点云 | 已实现 | `planned_quadrotor.rviz` |
| PyQt5 地面站、曲线与数据导出 | 已实现 | `drone_ground_station` |
| 动态障碍物、SLAM、多机、Gazebo | 未实现 | 不在当前范围 |

## ROS2 package

| package | 作用 |
|---|---|
| `drone_dynamics` | 电机一阶响应、6DoF 动力学、Odom/IMU/TF/Path/RPM |
| `drone_controller` | 位置 PD、几何姿态控制、X 型四电机 mixer |
| `drone_bringup` | 参数、URDF、launch、RViz、自动验收 |
| `drone_map` | YAML 地图、AABB、膨胀、净空、Marker/PointCloud2 |
| `drone_planner` | 3D A*、LOS、平滑、前视执行、任务管理与轨迹生成 |
| `drone_ground_station` | PyQt5 监控、任务控制、曲线与导出 |
| `drone_msgs` | 预留空包；当前继续使用 ROS2 标准消息 |

## 地图与安全模型

世界坐标系为 `map`，z 轴向上。默认地图边界为
`x[-1,6] m、y[-4,4] m、z[0,3.5] m`。无人机半径 `0.25 m`，
默认安全余量 `0.45 m`，因此默认碰撞膨胀距离为 `0.70 m`。

默认 `static_map.yaml` 包含 6 个障碍物：

| ID | 中心 (m) | 尺寸 x/y/z (m) |
|---|---|---|
| `central_block` | `(2.20, 0.00, 1.05)` | `(0.90, 1.20, 2.10)` |
| `north_corridor_wall` | `(2.225, 2.825, 1.325)` | `(1.75, 0.75, 2.65)` |
| `south_corridor_wall` | `(2.225, -2.70, 1.325)` | `(1.75, 1.00, 2.65)` |
| `north_rear_block` | `(3.75, 1.10, 1.175)` | `(0.80, 0.80, 2.35)` |
| `south_rear_block` | `(3.75, -1.30, 1.175)` | `(0.80, 0.80, 2.35)` |
| `far_corner_pillar` | `(4.975, 2.775, 1.40)` | `(0.65, 0.65, 2.80)` |

场景库还包括 `single_obstacle.yaml`、`narrow_passage.yaml`、
`random_seed_42.yaml`、`open_space.yaml` 和 `no_path.yaml`。
RViz 中红色为原始实体，蓝色透明体为严格按
`drone_radius + safety_margin` 生成的膨胀区域。肉眼看见的缝隙若被两个
膨胀体覆盖，仍不可通行；原始 Marker 不能作为规划碰撞真值。

`publish_point_cloud:=true` 时，地图节点按固定 spacing 对原始障碍物表面
采样并发布 `/map/obstacle_points`。点云仅用于显示。

## 规划与执行

规划输入为 `map` 系 `/drone/mission_goal`。A* 在膨胀 AABB 上使用 26
邻域搜索，并在超时、超出扩展数、目标越界/占用或无路径时明确返回失败。
完整路径链路为：

```text
A* raw path
→ line-of-sight simplification
→ Chaikin collision-safe smoothing
→ <= 0.05 m 致密检查 + 精确线段/AABB 检查
→ 保留关键拐点的弧长重采样
→ 20 Hz 弧长前视参考点
```

若任一平滑候选不安全，规划器回退到 LOS 安全折线；重采样保留原拐点，
避免跨拐点切角。默认 `minimum_flight_z=1.0 m`、
`cruise_altitude=1.5 m`、`takeoff_required=true`。低高度任务先垂直起飞，
普通中间路径点不得低于最低飞行高度；低目标只允许在经碰撞验证的最终
下降段到达。

状态包括 `TAKEOFF / PLANNING / EXECUTING / FINAL_APPROACH /
COMPLETED / FAILED / PAUSED / CANCELLED`。`PAUSE` 保持当前位置，
`RESUME` 继续，`CANCEL` 终止任务并保持当前位置。

## 节点与主要 Topic

| Topic | 类型 | 语义 |
|---|---|---|
| `/drone/mission_goal` | `geometry_msgs/PoseStamped` | 单个安全规划目标 |
| `/drone/mission_waypoints` | `nav_msgs/Path` | 多航点任务 |
| `/drone/mission_command` | `std_msgs/String` | START/PAUSE/RESUME/CANCEL/CLEAR |
| `/drone/goal` | `geometry_msgs/PoseStamped` | 控制器当前参考 |
| `/drone/planned_path` | `nav_msgs/Path` | 执行路径 |
| `/drone/path` | `nav_msgs/Path` | 实际轨迹 |
| `/drone/planner_status` | `std_msgs/String` | 状态与指标 JSON |
| `/drone/planning_metrics` | `std_msgs/String` | 可解析规划指标 JSON |
| `/drone/mission_status` | `std_msgs/String` | 多段任务状态 JSON |
| `/drone/mission_progress` | `std_msgs/Float32` | `[0,1]` 任务进度 |
| `/drone/avoidance_active` | `std_msgs/Bool` | 是否需要绕开直线路径 |
| `/drone/min_obstacle_clearance` | `std_msgs/Float32` | 规划路径最小净空 m |
| `/drone/current_waypoint_index` | `std_msgs/Int32` | 当前参考索引 |
| `/map/obstacles` | `visualization_msgs/MarkerArray` | 原始障碍物与边界 |
| `/map/inflated_obstacles` | `visualization_msgs/MarkerArray` | 膨胀安全区 |
| `/map/obstacle_points` | `sensor_msgs/PointCloud2` | 可选表面点云 |
| `/drone/odom`、`/drone/imu` | 标准消息 | 仿真状态 |
| `/drone/motor_rpm_cmd`、`/drone/motor_rpm` | `Float32MultiArray` | 四电机 RPM |

## 构建

系统依赖：Ubuntu 22.04、ROS2 Humble、Eigen3、RViz2、
`python3-pyqt5`、`python3-matplotlib` 和 `python3-yaml`。

中文界面还需要系统具备 CJK 字体（推荐 `fonts-noto-cjk`）。在没有管理员权限的
精简 WSL 环境中，也可把已有字体文件显式传给地面站：

```bash
export DRONE_GROUND_STATION_FONT_FILE=/path/to/a/cjk-font.ttf
```

```bash
cd ~/ros2_quadrotor_sim
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

从 GitHub clone 后不要提交 `build/ install/ log/ __pycache__/`。

## 运行

基础仿真：

```bash
ros2 launch drone_bringup sim.launch.py
```

规划仿真：

```bash
ros2 launch drone_bringup planned_sim.launch.py
```

Qt + 可选 RViz 一键展示：

```bash
ros2 launch drone_bringup showcase.launch.py gui:=true rviz:=true
```

独立地面站：

```bash
ros2 run drone_ground_station ground_station
ros2 launch drone_ground_station ground_station.launch.py
```

单目标命令：

```bash
ros2 topic pub --once /drone/mission_goal \
  geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 5.0, y: 0.0, z: 1.5}, orientation: {w: 1.0}}}"
```

多航点先向 `/drone/mission_waypoints` 发布 `nav_msgs/Path`，再发布：

```bash
ros2 topic pub --once /drone/mission_command std_msgs/msg/String "{data: START}"
```

## 自动测试与实验

```bash
./scripts/run_acceptance.sh
./scripts/run_planning_acceptance.sh scenario:=single
./scripts/run_planning_acceptance.sh scenario:=multi
./scripts/run_showcase_acceptance.sh scenario:=five_obstacles
./scripts/run_showcase_acceptance.sh scenario:=narrow_passage
./scripts/run_showcase_acceptance.sh scenario:=random random_seed:=42
./scripts/run_showcase_acceptance.sh scenario:=multi_segment
./scripts/run_showcase_acceptance.sh scenario:=invalid_goal  # 预期非零
./scripts/run_showcase_acceptance.sh scenario:=no_path       # 预期非零
./scripts/run_showcase_experiments.sh
```

批量实验把真实日志解析为 CSV、JSON 和 PNG，保存到
`output/showcase_experiments/<timestamp>/`。

本分支一次实测结果：

| 场景 | 规划时间 s | 扩展节点 | 最终误差 m | 最小规划净空 m | 碰撞 |
|---|---:|---:|---:|---:|---:|
| 默认 6 障碍物 | 0.179 | 630 | 0.0094 | 0.763 | 0 |
| 窄通道 | 0.063 | 216 | 0.0076 | 0.720 | 0 |
| 固定 seed 42 | 0.129 | 483 | 0.0109 | 0.703 | 0 |
| 开放空间方形任务 | 末段 0.0020 | 末段 13 | 0.0088 | 无障碍 | 0 |

原始证据位于 `log/showcase_acceptance/`；日志目录不进入 Git。

## PyQt5 地面站

地面站包含系统总览、地图与轨迹、飞行曲线、RPM、规划详情、任务控制、
测试中心和日志页面。ROS executor 在后台 `QThread` 中运行，ROS 回调只发
Qt signal，所有 QWidget 更新留在主线程。仿真通过 `QProcess` 启动独立
进程组；停止时只向该进程组依次发送 `SIGINT/TERM/KILL`，不使用全局
`pkill`。

GUI 支持地图选择、自定义 YAML、规划参数校验、JSON profile、单目标、
正方形/圆形/八字/自定义航点、二维俯视图、滚动曲线、CSV/PNG/JSON 和
截图导出。Vehicle Count 被限制为 1；多机只是预留，不是当前功能。

## 仓库与交付位置

- `src/`：七个 ROS2 package；
- `scripts/`：可重复运行和实验命令；
- `docs/`：架构、阶段计划、写作规范与证据索引；
- `output/`：后续最终报告、PDF 或实验导出；
- `ai_usage.md`：AI 辅助开发记录。

当前仓库没有最终 LaTeX 报告、最终 PDF 或演示视频；这些属于后续阶段，
不得把本 README 的测试摘要当作最终报告。

## 当前限制

- 静态已知 AABB 地图，不支持动态障碍物、SLAM 或 OctoMap；
- 轨迹由离散航点组成，不声称连续最优；
- IMU 无噪声、偏置和延迟模型；
- 电机独立饱和会改变期望力/矩比例；
- 单机 Topic 固定为 `/drone/*`，未实现真正 namespace 多机隔离；
- Qt 二维图用于操作与观察，不能替代 RViz 三维语义；
- 随机地图生成器需要调用项目 A* validator，失败会明确抛错。
