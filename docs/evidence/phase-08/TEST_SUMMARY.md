# 阶段 8：规划系统验收证据

## 验收口径

正向场景同时检查：

1. 起点到任务目标的直线与膨胀障碍物相交；
2. A* 与 LOS 简化成功；
3. 规划折线在膨胀 AABB 上无碰撞；
4. 实际 `/drone/odom` 轨迹在原始 AABB 上无碰撞；
5. 实际最小净空不小于 `drone_radius=0.25 m`；
6. 最终位置、yaw、线速度和角速度稳定后规划器发布 `COMPLETED`；
7. wrapper 找到显式 PASS、launch 为 0 且没有异常节点退出。

规划路径安全与实际轨迹安全是两个独立判据。

## 正向场景结果

| 指标 | 单障碍 | 多障碍（3 个） |
|---|---:|---:|
| wrapper | PASS / 0 | PASS / 0 |
| planning time | 0.112059 s | 0.745144 s |
| expanded nodes | 739 | 3765 |
| raw path points | 24 | 34 |
| simplified waypoints | 4 | 5 |
| planned path length | 6.770024 m | 9.091751 m |
| actual trajectory length | 8.260899 m | 10.559956 m |
| minimum planned-path clearance | 0.925000 m | 0.825000 m |
| minimum actual-trajectory clearance | 0.994409 m | 0.764103 m |
| final position error | 0.006568 m | 0.014054 m |
| final yaw error | 0.002725 deg | 0.072384 deg |
| mission completion time | 16.100074 s | 18.748209 s |
| collision count | 0 | 0 |

原始日志：

- 单障碍：`log/planning_acceptance/single-20260725-023835.log`
  - SHA-256 `b86d2122ffb0d1120c726915ed0ce97d0b1f459af1e4dcf086773ccdbb78a281`
- 多障碍：`log/planning_acceptance/multi-20260725-023940.log`
  - SHA-256 `80f77352a4bea82c68b07eed63cf77061a460367051dab181bf092b066ecfd01`

这些数值均为日志中的单次实测样本，不代表统计分布。

## 负向场景结果

| 场景 | 规划器结果 | 扩展数 | wrapper | 安全行为 |
|---|---|---:|---:|---|
| 目标位于膨胀障碍物内 | `FAILED: GOAL_OCCUPIED` | 0 | 非零（1） | 未发布 `/drone/goal` |
| 封闭墙无路 | `FAILED: NO_PATH` | 4032 | 非零（1） | 未发布 `/drone/goal` |

原始日志：

- 非法目标：`log/planning_acceptance/invalid_goal-20260725-023911.log`
  - SHA-256 `fe8abcaac9dd52b425730e216b317a7d496a42268c9edec7d1346b46bf7ac2eb`
- 无路：`log/planning_acceptance/no_path-20260725-023921.log`
  - SHA-256 `109bb89b2c1222138990c4e9fa525e1cb0c0405f126d4ed5581e7ca6e2c0b223`

## 基础回归

`./scripts/run_acceptance.sh` 在规划实现后仍为显式 `ACCEPTANCE TEST: PASS`：

- shell exit code：0；
- final position error：`0.008247 m`；
- final yaw error：`0.0065 deg`；
- linear speed：`0.005924 m/s`；
- angular speed：`0.002901 rad/s`；
- convergence time：`11.40 s`。

原始日志：`log/acceptance/acceptance-20260725-024015-23522.log`，SHA-256 `19326d902e50b1ed5284239bf1cc39823f37e643192b86f9f4c9b1134dcd478d`。

## 残留进程检查

四个规划验收场景和基础验收结束后，以下检查均无输出：

```bash
ps -eo pid,args | grep <项目节点可执行名>
ROS2CLI_NO_DAEMON=1 ROS_LOCALHOST_ONLY=1 ros2 node list
```

没有发现 dynamics、controller、map、planner、验收、goal marker 或 Robot State Publisher 残留。

## 复现命令

```bash
./scripts/run_planning_acceptance.sh scenario:=single
./scripts/run_planning_acceptance.sh scenario:=multi
./scripts/run_planning_acceptance.sh scenario:=invalid_goal
./scripts/run_planning_acceptance.sh scenario:=no_path
```

前两条预期返回 0；后两条预期输出 `PLANNING ACCEPTANCE: FAIL` 并返回非零。

## 尚未覆盖

- 狭窄通道专用系统场景；
- 动态障碍和地图更新；
- 长时间资源与内存稳定性；
- 同一正向场景连续多次的统计重复性；
- RViz 人工截图验收。
