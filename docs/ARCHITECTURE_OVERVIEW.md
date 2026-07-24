# ROS2 四旋翼仿真项目架构概览

## 1. 文档范围

本文描述当前仓库中实际存在的核心仿真与静态规划架构。接口、默认参数和公式以 `src/` 下源码、`vertical_sim.yaml` 和 `drone_map/config/*.yaml` 为准；实测结论以当前日志和阶段证据为准。

当前系统是一个进程级 ROS2 闭环仿真，不依赖 Gazebo：

```text
任务目标或直接控制目标
  -> 可选静态地图与 3D A* 航点层
  -> 位置/姿态控制器
  -> 4 路电机 RPM 命令
  -> 6DoF 动力学
  -> Odom/IMU/TF/Path/实际 RPM
  -> 控制器反馈与 RViz 显示
```

## 2. 当前完成度边界

| 能力 | 当前状态 | 说明 |
|---|---|---|
| 6DoF 动力学 | 已有实现 | C++，固定步长墙钟定时器 |
| 四电机一阶响应 | 已有实现 | RPM 命令到实际 RPM |
| 位置与姿态控制 | 已有实现 | 位置 PD + 几何姿态反馈 |
| 电机 mixer | 已有实现 | 总推力/三轴力矩到 4 路 RPM |
| Odom、IMU、TF、Path | 已有实现 | 由动力学节点发布 |
| URDF、RViz、目标 Marker | 已有实现 | `drone_bringup` |
| 单目标自动验收 | 已有实现 | `run_acceptance.sh` 当前复测 PASS |
| 自定义消息 | 未实现 | `drone_msgs` 为空包 |
| 静态地图 | 已有实现 | YAML AABB、膨胀几何和 transient-local MarkerArray |
| 路径规划/避障 | 已有实现 | 26 邻域 3D A*、LOS 简化和航点执行 |
| 规划系统验收 | 已有实现 | 单/多障碍正向与非法目标/无路负向场景 |
| 地面站 | 未实现 | 当前采用 RViz |

## 3. 包级架构

```mermaid
flowchart LR
    B["drone_bringup<br/>launch / config / URDF / RViz / 验收"]
    C["drone_controller<br/>位置与姿态控制"]
    D["drone_dynamics<br/>电机与 6DoF 刚体动力学"]
    M["drone_msgs<br/>空骨架"]
    MAP["drone_map<br/>YAML / AABB / 碰撞几何 / Marker"]
    P["drone_planner<br/>3D A* / LOS 简化 / 航点执行"]

    B --> C
    B --> D
    B --> MAP
    B --> P
    M -. "当前未被引用" .-> C
    MAP --> P
    P --> C
```

### 3.1 `drone_dynamics`

职责：

- 订阅四路电机 RPM 命令；
- 模拟电机一阶动态；
- 计算各旋翼推力、总推力和三轴力矩；
- 积分位置、速度、姿态和机体系角速度；
- 施加线性阻力、角阻力、重力和简化地面约束；
- 发布 odometry、理想 IMU、轨迹、实际 RPM 和动态 TF。

非职责：

- 不做碰撞检测；
- 不模拟电池、电机电气模型、桨叶气动耦合或地效；
- 不添加传感器噪声、偏置和延迟；
- 不发布 `/clock`。

### 3.2 `drone_controller`

职责：

- 接收目标位姿和当前 odometry；
- 由位置误差和速度反馈计算期望加速度；
- 执行水平/竖直加速度限制；
- 由期望合力和目标偏航构造期望姿态；
- 使用 SO(3) 几何姿态误差和角速度反馈计算力矩；
- 通过 X 型 mixer 输出 4 路 RPM；
- 发布期望姿态供 RViz 显示。

当前控制器是 PD 控制器，没有积分项、轨迹前馈、抗积分饱和或电机失效分配。

### 3.3 `drone_bringup`

职责：

- 保存全局默认参数；
- 启动动力学、控制器、Robot State Publisher 和 RViz；
- 提供四旋翼 URDF 与 RViz 配置；
- 将目标位姿转换为持续显示的 Marker；
- 提供单目标自动验收节点和 launch。
- 提供规划仿真、规划验收 launch、独立 RViz 与场景 wrapper。

### 3.4 `drone_map`

职责：

- 从固定 YAML 加载地图边界、无人机半径、安全余量、栅格分辨率和 AABB；
- 验证有限值、正尺寸、边界关系、唯一 ID 和障碍物范围；
- 提供与 ROS 解耦的点/线段/折线路径碰撞、膨胀和连续净空计算；
- 以 Reliable、Transient Local QoS 发布 `/map/obstacles`。

### 3.5 `drone_planner`

职责：

- 在膨胀 AABB 上执行有扩展数与时间上限的 26 邻域 3D A*；
- 使用连续线段碰撞检查防止对角穿角；
- 用最远可视后继策略简化栅格路径，并再次验证每段；
- 将 `/drone/mission_goal` 转换为顺序 `/drone/goal` 航点；
- 处理新任务重规划、航点切换、最终 yaw 和稳定完成状态。

### 3.6 预留包

`drone_msgs` 仍没有 `.msg/.srv/.action` 定义。地图与规划继续复用标准消息。

## 4. 运行时节点与进程

| 节点 | 包 | 语言 | 默认触发频率 | 主要职责 |
|---|---|---|---:|---|
| `quadrotor_dynamics_node` | `drone_dynamics` | C++ | 积分 200 Hz | 动力学和状态发布 |
| `position_controller_node` | `drone_controller` | C++ | 100 Hz | 位置/姿态控制和 mixer |
| `robot_state_publisher` | 外部 ROS2 包 | C++ | 事件驱动 | 发布 URDF 固定关节 |
| `goal_marker_node` | `drone_bringup` | Python | Marker 重发 5 Hz | RViz 目标球 |
| `rviz2` | 外部 ROS2 包 | C++ | 画面 30 FPS | 可视化 |
| `acceptance_test_node` | `drone_bringup` | Python | 10 Hz | 发目标并判定收敛 |
| `static_map_node` | `drone_map` | Python | 静态一次发布 | 地图 MarkerArray |
| `planner_node` | `drone_planner` | Python | 事件驱动 | 3D A*、简化和航点执行 |
| `planning_acceptance_node` | `drone_bringup` | Python | 10 Hz | 规划路径与实际轨迹联合验收 |

`acceptance_test_node` 只在基础验收启动。`planning_acceptance_node` 只在规划验收启动；它结束时触发整个 launch 清理。

## 5. 当前闭环数据流

```mermaid
flowchart LR
    U["用户/规划验收"]
    MG["/drone/mission_goal"]
    MAP["static_map_node<br/>共享 YAML + AABB"]
    PLAN["planner_node<br/>3D A* + LOS + 航点"]
    PP["/drone/planned_path"]
    G["/drone/goal"]
    C["position_controller_node"]
    CMD["/drone/motor_rpm_cmd"]
    D["quadrotor_dynamics_node"]
    O["/drone/odom"]
    ACT["/drone/path"]
    V["RViz2"]

    U --> MG --> PLAN
    MAP --> PLAN
    MAP --> V
    PLAN --> PP --> V
    PLAN --> G --> C
    C --> CMD --> D
    D --> O
    O --> C
    O --> PLAN
    D --> ACT --> V
```

普通 `sim.launch.py` 仍允许用户直接发布 `/drone/goal`，用于不启用规划器的基础闭环。规划模式下用户只向 `/drone/mission_goal` 发最终任务目标，规划器独占安全航点生成。

## 6. ROS2 接口契约

高频控制与状态接口使用 depth 10 的 Reliable、Volatile、Keep Last。静态地图、规划路径、规划状态和规划 Marker 使用 depth 1 的 Reliable、Transient Local QoS，使晚启动 RViz/验收订阅者能获得最新样本。

| Topic | 消息类型 | Publisher | Subscriber | 语义/单位 | 预期频率 |
|---|---|---|---|---|---:|
| `/drone/mission_goal` | `geometry_msgs/msg/PoseStamped` | 用户或规划验收 | 规划器 | `map` 系最终任务目标 | 事件驱动 |
| `/drone/goal` | `geometry_msgs/msg/PoseStamped` | 用户/基础验收或规划器 | 控制器、目标 Marker | 当前控制目标或安全航点 | 事件驱动 |
| `/map/obstacles` | `visualization_msgs/msg/MarkerArray` | 静态地图 | RViz | 原始 AABB 和地图边界；仅用于显示 | 静态 |
| `/drone/planned_path` | `nav_msgs/msg/Path` | 规划器 | RViz、规划验收 | 简化后的安全航点折线 | 每次成功规划 |
| `/drone/current_waypoint` | `visualization_msgs/msg/Marker` | 规划器 | RViz | 当前控制航点 | 航点切换 |
| `/drone/mission_goal_marker` | `visualization_msgs/msg/Marker` | 规划器 | RViz | 最终任务目标 | 每次新任务 |
| `/drone/planner_status` | `std_msgs/msg/String` | 规划器 | 规划验收/观察者 | JSON；状态与规划指标 | 状态变化 |
| `/drone/motor_rpm_cmd` | `std_msgs/msg/Float32MultiArray` | 控制器 | 动力学 | `[M1,M2,M3,M4]`，单位 RPM | 100 Hz |
| `/drone/motor_rpm` | `std_msgs/msg/Float32MultiArray` | 动力学 | 外部观察者 | 实际电机 RPM | 约 50 Hz |
| `/drone/odom` | `nav_msgs/msg/Odometry` | 动力学 | 控制器、验收节点 | 世界系位置/速度，机体系角速度 | 约 50 Hz |
| `/drone/imu` | `sensor_msgs/msg/Imu` | 动力学 | 外部观察者 | 理想姿态、角速度、机体系比力 | 约 50 Hz |
| `/drone/path` | `nav_msgs/msg/Path` | 动力学 | RViz | 最近最多 5000 个实际位姿 | 约 10 Hz |
| `/drone/desired_pose` | `geometry_msgs/msg/PoseStamped` | 控制器 | RViz | 目标位置和期望姿态 | 100 Hz |
| `/drone/goal_marker` | `visualization_msgs/msg/Marker` | 目标 Marker | RViz | 绿色球形目标 | 5 Hz 重发 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 动力学等 | RViz/TF 消费者 | 动态 `map -> base_link` | 约 50 Hz |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | Robot State Publisher | RViz/TF 消费者 | 固定 `base_link -> body_link` | Transient Local |
| `/robot_description` | `std_msgs/msg/String` | Robot State Publisher | RViz | URDF 描述 | Transient Local |

> “预期频率”来自源码定时器和分频逻辑，必须在阶段测试中实测确认。

## 7. 坐标系与模型树

```mermaid
flowchart TD
    MAP["map<br/>世界惯性坐标系"]
    BASE["base_link<br/>动力学位姿根节点"]
    BODY["body_link<br/>URDF 机体可视/惯性节点"]

    MAP -- "动态 TF：位置 + 姿态" --> BASE
    BASE -- "固定 TF：零平移、零旋转" --> BODY
```

- `map`：世界坐标系，重力方向为负 z。
- `base_link`：动力学节点发布的无人机位姿。
- `body_link`：URDF 中包含质量、惯量和全部可视几何。
- 四元数 `orientation_` 表示机体系到世界系的旋转。
- odometry 的线速度来自世界系积分结果；角速度为机体系角速度。

## 8. 电机布局与单位

电机顺序是跨动力学、控制器、测试和文档的硬接口：

```text
               机头 +X

        M1 前左          M2 前右
             \          /
              \        /
               机体中心
              /        \
             /          \
        M4 后左          M3 后右

                    +Y 指向左侧
```

当前力矩符号由源码定义：

```text
tau_x = l/sqrt(2) * (F1 - F2 - F3 + F4)
tau_y = l/sqrt(2) * (-F1 - F2 + F3 + F4)
tau_z = kM * (-w1^2 + w2^2 - w3^2 + w4^2)
```

外部接口使用 RPM；进入推力模型前转换为 rad/s：

```text
omega_i = rpm_i * 2*pi/60
F_i     = kF * omega_i^2
```

禁止将 RPM 直接代入 `kF * omega^2`。

## 9. 动力学模型摘要

### 9.1 电机一阶响应

源码使用离散一阶逼近：

```text
alpha        = clamp(dt / tau_motor, 0, 1)
rpm_actual  += alpha * (rpm_command - rpm_actual)
```

### 9.2 平动

```text
F_world = R_body_to_world * [0, 0, sum(F_i)]
F_g     = [0, 0, -m*g]
F_drag  = -D_v * v
a       = (F_world + F_g + F_drag) / m
v      <- v + a*dt
p      <- p + v*dt
```

位置更新使用已经更新后的速度，属于半隐式欧拉形式。

### 9.3 转动

```text
I * omega_dot = tau - omega x (I*omega) - D_w*omega
omega        <- omega + omega_dot*dt
q_dot         = 0.5 * q ⊗ [0, omega]
q            <- normalize(q + q_dot*dt)
```

### 9.4 数值保护

- RPM 被限制到 `[0, max_rpm]`；
- 四元数每步归一化；
- 位置、速度、加速度、角速度、角加速度或四元数出现非有限值时重置状态；
- `z < 0` 时位置和向下速度被钳制到地面。

## 10. 控制器模型摘要

### 10.1 位置环

```text
e_p   = p_target - p
a_des = Kp .* e_p - Kd .* v
```

- 水平加速度按二维范数限幅；
- 竖直加速度逐分量限幅；
- 加上重力补偿得到期望合力。

### 10.2 倾角限制与期望姿态

期望合力的水平分量限制为：

```text
||F_xy|| <= F_z * tan(max_tilt)
```

期望机体 z 轴沿期望合力方向，目标偏航用于构造期望机体 x/y 轴。

### 10.3 几何姿态控制

```text
e_R = vee(0.5 * (R_d^T R - R^T R_d))
tau = -K_R .* e_R - K_w .* omega + omega x (I*omega)
```

总推力为期望合力在当前机体 z 轴上的投影，并限制到：

```text
0 <= thrust <= 4*m*g
```

### 10.4 Mixer

控制器先求各电机角速度平方，再逐电机限制到最大 RPM 对应的范围，最后转换回 RPM。当前饱和方式是独立截断，因此极端情况下实际总推力/力矩不再严格等于期望值。

## 11. 默认参数摘要

| 分类 | 参数 | 默认值 |
|---|---|---:|
| 动力学 | 更新频率 | 200 Hz |
| 机体 | 质量 | 1.0 kg |
| 环境 | 重力 | 9.81 m/s² |
| 几何 | 机臂长度 | 0.22 m |
| 电机 | 推力系数 `kF` | `8.54858e-6` |
| 电机 | 反扭矩系数 `kM` | `1.37e-7` |
| 电机 | 时间常数 | 0.05 s |
| 电机 | 最大转速 | 10000 RPM |
| 惯量 | `Ixx, Iyy, Izz` | `0.005, 0.005, 0.009 kg*m²` |
| 位置环 | `Kp` | `[1.8, 1.8, 3.0]` |
| 位置环 | `Kd` | `[2.2, 2.2, 2.5]` |
| 姿态环 | `Kr` | `[0.08, 0.08, 0.05]` |
| 角速度环 | `Kw` | `[0.018, 0.018, 0.012]` |
| 安全限制 | 最大水平加速度 | 3.0 m/s² |
| 安全限制 | 最大竖直加速度 | 3.0 m/s² |
| 安全限制 | 最大倾角 | 25 deg |
| 安全限制 | 单次目标距离 | 10 m |

控制器与动力学重复声明了质量、重力、机臂、推力系数、反扭矩系数、惯量和最大 RPM。它们必须保持一致。

## 12. 启动模式

### 12.1 完整可视化

```bash
ros2 launch drone_bringup sim.launch.py
```

启动动力学、控制器、Robot State Publisher、目标 Marker 和 RViz。

### 12.2 无 RViz

```bash
ros2 launch drone_bringup sim.launch.py rviz:=false
```

RViz 不启动，其他节点仍启动。

### 12.3 核心闭环

```bash
ros2 launch drone_bringup core_sim.launch.py
```

只启动动力学和控制器。

`vertical_hover.launch.py` 当前也只启动相同两个节点，功能上与 `core_sim.launch.py` 重叠，后续应决定保留、重命名或删除。

### 12.4 自动验收

```bash
ros2 launch drone_bringup acceptance_test.launch.py
```

启动动力学、控制器和验收节点。默认判据：

- 三维位置误差不大于 `0.05 m`；
- 偏航误差不大于 `3 deg`；
- 线速度不大于 `0.05 m/s`；
- 角速度不大于 `0.05 rad/s`；
- 连续稳定至少 `1 s`；
- discovery 最多 `15 s`，发目标后收敛最多 `20 s`。

这些阈值均由基础验收 launch 参数显式传入。

### 12.5 规划可视化

```bash
ros2 launch drone_bringup planned_sim.launch.py
```

该入口 include 普通 `sim.launch.py rviz:=false`，再启动静态地图、规划器和规划专用 RViz。`map_file` 与 `rviz_config` 均可覆盖；`rviz:=false` 可用于无界面运行。

### 12.6 规划自动验收

```bash
./scripts/run_planning_acceptance.sh scenario:=single
./scripts/run_planning_acceptance.sh scenario:=multi
./scripts/run_planning_acceptance.sh scenario:=invalid_goal
./scripts/run_planning_acceptance.sh scenario:=no_path
```

正向场景必须同时满足规划折线无膨胀碰撞、实际 odometry 轨迹无原始 AABB 碰撞、实际净空不小于无人机半径并稳定到达。两个负向场景必须输出 `PLANNING ACCEPTANCE: FAIL` 且 wrapper 返回非零。

## 13. 当前已知架构风险

1. ROS2 launch 的外层退出码不能单独代表验收结果，必须由 repository wrapper 解析显式 PASS/FAIL 和节点异常。
2. 动力学和控制器仍缺少可脱离 ROS graph 的行为级单元测试。
3. 共享物理参数在动力学和控制器中重复，配置漂移会破坏闭环。
4. `Float32MultiArray` 不携带电机顺序、单位和时间戳，接口自描述性较弱。
5. IMU 没有协方差、噪声和偏置，不能等同于真实传感器。
6. 独立电机饱和会改变 mixer 输出的力/力矩比例。
7. 当前 A* 是静态全局规划，不支持动态障碍、运行中地图更新或持续重规划。
8. AABB 表示只支持轴对齐盒；Marker 不是规划输入，二者由同一 YAML 生成但职责不同。
9. 航点切换采用距离阈值，控制器会产生与规划折线不同的连续轨迹，因此验收必须独立检查实际 odometry 净空。
10. 搜索分辨率固定来自单一地图 YAML；更细分辨率会显著增加内存与扩展数。
11. `core_sim.launch.py` 与 `vertical_hover.launch.py` 功能重叠。
12. `drone_msgs` 仍是空骨架，不能描述为已实现自定义接口。

## 14. 已实现规划边界

- 地图固定在启动时加载，运行期间不接受更新。
- 用户任务目标固定为 `/drone/mission_goal`；规划器输出 `/drone/goal`，避免反馈回环。
- 新任务会取消旧航点并从最新有效 odometry 重新规划。
- 规划失败不会向控制器发布任务最终目标。
- 当前只实现 3D 栅格 A* 与折线 LOS 简化，不包含 B 样条、RRT、SLAM、OctoMap 或动态避障。
- 碰撞规划使用膨胀 AABB；实际轨迹验收使用原始 AABB 并另行检查无人机半径净空。

## 15. 架构变更准则

- 修改 topic、frame、电机顺序、单位、QoS 或共享物理参数属于接口变更，必须同步更新测试和文档。
- 修改动力学符号、积分顺序、mixer 或姿态误差定义属于数学行为变更，必须增加针对性单元测试和场景回归。
- 地图与规划功能必须可关闭；关闭后不得改变必做闭环。
- 所有结果声明以当前提交和当前参数的可复现测试为准。
