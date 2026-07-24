# 阶段 0 自动验收基础设施测试摘要

## 1. 测试上下文

| 项目 | 实测值 |
|---|---|
| 测试日期 | 2026-07-25 |
| Git 分支 | `fix/acceptance-timing` |
| Git commit | `19810f433a906af40f8fb8878ecdba0509f7deec` |
| 操作系统 | Ubuntu 22.04（WSL） |
| ROS 发行版 | ROS2 Humble |
| `RMW_IMPLEMENTATION` | 环境变量未显式设置；运行时解析为 `rmw_fastrtps_cpp` |
| `ROS_DOMAIN_ID` | 未显式设置，使用默认 Domain 0 |
| `ROS_LOCALHOST_ONLY` | 验收包装脚本固定为 `1` |
| Fast DDS 配置 | `scripts/fastdds_acceptance.xml`，仅对验收包装脚本启动的进程生效 |
| 参数文件 | `src/drone_bringup/config/vertical_sim.yaml`，本阶段未修改 |

运行时 RMW 的确认命令：

```bash
source /opt/ros/humble/setup.bash
python3 -c \
  'from rclpy.utilities import get_rmw_implementation_identifier; print(get_rmw_implementation_identifier())'
```

输出为 `rmw_fastrtps_cpp`。

## 2. 构建与现有测试

执行命令：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose
```

结果：

- 6 个 package 全部构建成功；
- 13 项测试；
- 0 error；
- 0 failure；
- 3 skipped；
- skipped 项均为 `drone_bringup`、`drone_map` 和
  `drone_planner` 中 ROS 模板生成的版权检查。

## 3. 默认验收连续三次

每次均执行：

```bash
./scripts/run_acceptance.sh
```

三次均出现且仅出现一次 `ACCEPTANCE TEST: PASS`，包装脚本均返回
0，验收节点、控制器和动力学节点均结束，包装脚本记录的残留进程组
检查均为 0。

| 次数 | Discovery time (s) | Convergence time (s) | Position error (m) | Yaw error (deg) | Linear speed (m/s) | Angular speed (rad/s) | Shell exit code | 原始日志 |
|---:|---:|---:|---:|---:|---:|---:|---:|---|
| 1 | 3.30 | 11.40 | 0.008297 | 0.0059 | 0.004880 | 0.002107 | 0 | `log/acceptance/acceptance-20260725-004151-16884.log` |
| 2 | 3.33 | 10.60 | 0.009101 | 0.0231 | 0.008826 | 0.005091 | 0 | `log/acceptance/acceptance-20260725-004215-17235.log` |
| 3 | 3.40 | 11.44 | 0.008297 | 0.0059 | 0.004880 | 0.002107 | 0 | `log/acceptance/acceptance-20260725-004240-17579.log` |

上述三次为最终代码状态下连续执行的全部样本，没有从更多运行中只挑选
表现最好的样本。

## 4. 负向与生命周期测试

### 4.1 收敛超时

命令：

```bash
./scripts/run_acceptance.sh timeout_sec:=0.5
```

结果：

- 明确输出 `ACCEPTANCE TEST: FAIL`；
- 原因为 `Convergence timeout: tracking did not meet all thresholds`；
- Discovery time 为 3.20 s；
- Convergence time 为 0.60 s；
- shell exit code 为 1；
- 三个本次启动的节点均已结束，无残留。

原始日志：
`log/acceptance/acceptance-20260725-004315-17943.log`。

### 4.2 Discovery 超时

命令：

```bash
./scripts/run_acceptance.sh discovery_timeout_sec:=0.001
```

结果：

- 明确输出 `ACCEPTANCE TEST: FAIL`；
- 原因为
  `Discovery timeout: missing controller subscription and valid odometry`；
- Discovery time 为 0.10 s；
- Convergence time 为 0.00 s；
- shell exit code 为 1；
- 三个本次启动的节点均已结束，无残留。

原始日志：
`log/acceptance/acceptance-20260725-004335-18143.log`。

该测试使用公开 launch 参数压缩 discovery 预算，不删除或修改控制器、
动力学节点，也不制造不可恢复状态。

### 4.3 Ctrl+C

为在非交互测试中准确发送 SIGINT 并读取 WSL 内部真实退出码，执行了：

```bash
timeout --preserve-status --signal=INT 2s \
  ./scripts/run_acceptance.sh
```

结果：

- 包装脚本输出 `result: INTERRUPTED`；
- WSL shell 内观察到的包装脚本退出码为 130；
- 日志明确记录 `Wrapper interrupted by INT.`；
- 验收节点返回 130；
- 控制器和动力学节点正常结束；
- 针对
  `acceptance_test_node`、`position_controller_node` 和
  `quadrotor_dynamics_node` 的残留检查未发现进程；
- 没有使用全局 `pkill`。

原始日志：
`log/acceptance/acceptance-20260725-004103-16469.log`。

## 5. 根因与修复结论

原验收程序从节点创建时开始计算唯一的 `timeout_sec`，目标发布前等待
ROS/DDS 发现的时间会直接挤占飞行收敛预算。失败时验收节点即使返回 1，
外层 `ros2 launch` 仍可能返回 0，因而外层退出码不能单独作为验收结论。
此外，原节点依赖延迟线程和 `os._exit()`，不能证明日志与资源沿标准
ROS2 生命周期完整释放。

本阶段的结论为：

> 等待 ROS/DDS 发现消耗了原本用于飞行收敛的 timeout，同时默认 15
> 秒收敛预算缺少运行波动余量。延长 timeout 后，动力学和控制器闭环已
> 通过目标点验收。

修复内容：

1. 用独立的 `discovery_timeout_sec` 管理发现阶段；
2. 仅在控制器订阅和有限值 odometry 都就绪后发布一次目标；
3. 从 `goal_publish_time` 开始计算 `timeout_sec` 和 convergence time；
4. 默认 convergence 预算改为 20.0 s，不改变误差阈值和稳定时长；
5. 删除 `os._exit()`，由主循环观察完成状态并执行标准 shutdown；
6. wrapper 解析明确的 PASS/FAIL、节点退出和日志完整性，不依赖外层
   `ros2 launch` 退出码；
7. wrapper 只管理自己通过 `setsid` 创建的进程组；
8. 当前 WSL 的 Fast DDS 默认发现存在间歇波动，wrapper 使用仅限本机
   验收图的 loopback UDP 和 unicast initial peer 配置，未加入固定
   sleep。

## 6. 修改文件

- `src/drone_bringup/drone_bringup/acceptance_test_node.py`
- `src/drone_bringup/launch/acceptance_test.launch.py`
- `scripts/run_acceptance.sh`
- `scripts/fastdds_acceptance.xml`
- `docs/evidence/phase-00/TEST_SUMMARY.md`
- `docs/PROJECT_EXECUTION_PLAN.md`

本阶段没有修改 `src/drone_dynamics/`、`src/drone_controller/`、核心
物理参数、控制增益、误差阈值、URDF、RViz、地图、规划、现有 LaTeX
手册或任何现有 PDF。

## 7. 已知限制

- `scripts/fastdds_acceptance.xml` 专用于同机自动验收。需要从其他主机
  观察验收图时，必须另行设计网络配置，不能直接套用该本机配置。
- 完整 ROS 运行日志位于被 Git 忽略的 `log/acceptance/`；本文件只保存
  可审查摘要和路径，不复制大体积日志。
- 3 个模板版权检查仍为 skipped；它们不是飞行闭环测试。
- 本阶段没有新增动力学、控制器数学单元测试，也没有完成所有 topic
  频率与长时间稳定性测试，这些仍属于后续阶段。
- 外层 `ros2 launch` 退出码仍不单独代表验收结果；应始终使用
  `scripts/run_acceptance.sh`。
