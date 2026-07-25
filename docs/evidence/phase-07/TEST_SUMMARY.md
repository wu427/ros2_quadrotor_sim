# 阶段 7：静态地图与规划核心证据

## 证据范围

- 分支：`feature/static-map-3d-astar`
- 基线提交：`616e01b`
- 测试日期：2026-07-25（Asia/Shanghai）
- 地图 frame：`map`
- 正式功能均位于现有 ROS2 package；没有仓库外运行时代码。

## 地图参数

| 参数 | 值 | 单位/语义 |
|---|---:|---|
| `x_min`, `x_max` | -1.0, 6.0 | m |
| `y_min`, `y_max` | -4.0, 4.0 | m |
| `z_min`, `z_max` | 0.0, 3.5 | m |
| `drone_radius` | 0.25 | m |
| `safety_margin` | 0.45 | m |
| `grid_resolution` | 0.25 | m |
| 总膨胀距离 | 0.70 | `drone_radius + safety_margin` |

默认多障碍地图：

| ID | 中心 `(x,y,z)` m | 尺寸 `(x,y,z)` m |
|---|---|---|
| `central_gate` | `(1.8, 0.0, 1.5)` | `(0.8, 1.4, 3.0)` |
| `north_pillar` | `(3.0, 1.55, 1.5)` | `(0.8, 1.0, 3.0)` |
| `south_pillar` | `(4.0, -1.55, 1.5)` | `(0.8, 1.0, 3.0)` |

另有 `single_obstacle.yaml` 和用整面 AABB 墙封闭地图的 `no_path.yaml`。三份配置均安装到 `share/drone_map/config/`。

## 碰撞模型

- 原始障碍物为闭边界 AABB；点落在边界上按碰撞处理。
- 规划占用体为每个原始 AABB 向六个方向膨胀 `0.70 m`。
- 线段相交采用 slab clipping，不用离散采样。
- 线段到原始 AABB 的最小距离按分段二次函数求最小值，用于连续净空。
- 路径碰撞逐点、逐线段检查；Marker 只负责显示。
- 地图节点与规划器通过同一 `map_file` 参数加载同一 YAML 和同一纯 Python 几何模块。

## 3D A* 与简化配置

- 栅格点使用 cell centre，`world_to_grid` 的精确上界归入最后一个 cell。
- 26 邻域，轴向/面对角/体对角代价均为实际欧氏距离。
- 启发式为三维欧氏距离。
- 相邻栅格中心之间也执行连续膨胀 AABB 碰撞检查，禁止对角穿角。
- 默认最大扩展数 `100000`，默认规划时间上限 `2.0 s`。
- 成功路径保留真实起点和最终目标。
- 简化器从当前点寻找最远直达后继，随后重新检查每段并验证路径长度不增加。

## 单元测试

最终命令：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose
```

结果：

- 6 packages 构建成功；
- `46 tests, 0 errors, 0 failures, 3 skipped`；
- 地图几何新增 14 个行为测试；
- A* 新增 14 个行为测试；
- 路径简化新增 5 个行为测试；
- 跳过项是三个 ROS 模板 copyright 检查，不是算法测试。

原始日志：

- `log/planning_acceptance/full-build-20260725.log`
  - SHA-256 `ef885aab6e1b9285e9584e3d0bfc3e5a65de2f4f9181c215498e0c86feff269f`
- `log/planning_acceptance/full-test-20260725.log`
  - SHA-256 `65d9972fdf8c7aa29f1b3a569aab41b781c457543f78a142060ff348fc9e0db6`

## 已知限制

- 地图在节点启动时加载，不支持动态更新。
- 只支持轴对齐长方体，不支持旋转盒、网格或动态障碍。
- A* 内存和运行时间随分辨率三次方增长。
- 当前不实现 B 样条、RRT、SLAM、OctoMap 或在线持续重规划。
- RViz 独立配置已生成，但本阶段没有把人工界面检查勾选为自动通过。
