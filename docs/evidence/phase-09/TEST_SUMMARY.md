# 阶段 09：地图、路径与任务展示增强证据

## 已完成工作

- [x] 默认地图扩展为 6 个 AABB 障碍物；
- [x] 增加窄通道、开放空间、无路径和固定 seed 42 场景；
- [x] 随机生成器支持边界、尺寸、起终点净空、最小间距和最大尝试数；
- [x] 随机候选必须由项目 3D A* validator 确认可达；
- [x] 发布原始/膨胀 Marker 和默认关闭的表面 PointCloud2；
- [x] 增加 `TAKEOFF` 与 `minimum_flight_z=1.0 m`；
- [x] 实现 LOS、Chaikin、安全回退、关键拐点保留和弧长重采样；
- [x] 实现 20 Hz 弧长前视参考、暂停、继续和取消；
- [x] 实现多航点任务管理与轨迹纯函数；
- [x] 增加标准规划状态、进度与指标 Topic；
- [x] 增加算法和轨迹测试。

## 实际执行

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select drone_map drone_planner drone_ground_station drone_bringup
source install/setup.bash

./scripts/run_acceptance.sh
./scripts/run_planning_acceptance.sh scenario:=multi mission_timeout_sec:=140.0
./scripts/run_showcase_acceptance.sh scenario:=five_obstacles
./scripts/run_showcase_acceptance.sh scenario:=narrow_passage
./scripts/run_showcase_acceptance.sh scenario:=random random_seed:=42
./scripts/run_showcase_acceptance.sh scenario:=hover
./scripts/run_showcase_acceptance.sh scenario:=point
./scripts/run_showcase_acceptance.sh scenario:=square
```

## 真实结果

| 场景 | 结果 | 规划时间 s | 扩展节点 | 终点误差 m | 规划/实际净空 m | 最低航点 z m | 碰撞 |
|---|---|---:|---:|---:|---:|---:|---:|
| 基础控制验收 | PASS | 不适用 | 不适用 | 0.007840 | 不适用 | 不适用 | 不适用 |
| 默认 6 障碍 | PASS | 0.178616 | 630 | 0.009389 | 0.763130 / 0.759106 | 1.384969 | 0 |
| 窄通道 | PASS | 0.063337 | 216 | 0.007627 | 0.719892 / 0.670824 | 1.327681 | 0 |
| 固定 seed 42 | PASS | 0.129144 | 483 | 0.010935 | 0.703337 / 0.699385 | 1.375000 | 0 |
| 悬停 | PASS | 0.000959 | 1 | 0.018207 | 无障碍 | 1.382412 | 0 |
| 单目标 | PASS | 0.010356 | 52 | 0.014656 | 无障碍 | 1.382412 | 0 |
| 方形五段任务 | PASS | 末段 0.002040 | 末段 13 | 0.008755 | 无障碍 | 1.385106 | 0 |

规划回归在发现并修复重采样切角后复测 PASS：规划路径净空
`0.763130 m`，实际轨迹净空 `0.760277 m`，碰撞 `0`。

负向场景已实际进入预期状态：

- 非法目标：`GOAL_OCCUPIED`；
- 无路径：`NO_PATH`；
- 取消：`CANCELLED`。

最终 wrapper 契约为负向场景返回非零，不把预期失败伪装成正向 PASS。

## 发现并修复的问题

首版弧长重采样未保留 LOS 拐点，新连接线与膨胀 AABB 精确相切。
`0.05 m` 离散采样未命中切点，但连续线段检查发现失败。修复后：

- 重采样目标集合强制包含全部原折线累计弧长；
- 每一段同时执行精确 AABB 相交和致密点检查；
- 同一真实飞行场景由 FAIL 转为 PASS。

## 尚未完成

- [ ] 10/30 分钟资源与内存稳定性测试；
- [ ] 每个场景连续三次统计；
- [ ] 干净 GitHub clone + rosdep 全流程；
- [ ] 最终发布候选标签。
