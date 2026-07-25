# AI 辅助开发说明

## 使用方式

本项目使用 OpenAI Codex 作为代码审计、实现、测试编排和文档整理工具。
AI 输出不直接视为正确；源码、当前参数和实际执行结果才是事实来源。

## AI 参与模块

- 仓库与 Git 保护区审计；
- AABB 地图场景、确定性随机生成器与点云表面采样；
- 3D A* 最低高度约束、Chaikin 平滑、重采样和前视执行；
- 多航点任务管理与预定义轨迹；
- PyQt5 地面站线程/进程架构、曲线和导出；
- showcase launch、正负向验收与实验导出脚本；
- 单元测试、ROS2 回归和项目文档。

核心动力学、控制器、mixer、物理参数和基础验收阈值未由 AI 修改。

## 关键 Prompt 摘要

1. 审计当前 ROS2 package、Topic、状态机和测试基线。
2. 在保护核心飞控的前提下把默认地图扩展到至少五个障碍物。
3. 设计膨胀后仍可通行的窄通道，并用真实飞行验证净空。
4. 实现固定 seed、起终点净空、障碍间距和 A* 反向验证。
5. 设计“LOS → Chaikin → 致密碰撞检查 → 弧长采样”流水线。
6. 在不改控制器的情况下实现弧长进度与 lookahead 参考。
7. 用标准 ROS2 消息实现多段任务和 START/PAUSE/RESUME/CANCEL。
8. 设计 Qt 主线程、ROS executor 后台线程和 signal 数据边界。
9. 设计只清理自身 QProcess 进程组的仿真启停策略。
10. 建立正向、负向、GUI offscreen 和数据导出证据链。

## 人工与自动检查

- 人工核对核心动力学/控制器 diff 必须为空；
- 检查电机顺序、Topic 类型、frame 和 SI/RPM 单位；
- 逐场景检查地图几何、膨胀尺寸和可行绕行；
- 用 `colcon test` 执行 lint 与算法测试；
- 用基础验收确认控制回路没有退化；
- 用规划和 showcase wrapper 检查进程退出、PASS/FAIL 记录与碰撞；
- 使用 `QT_QPA_PLATFORM=offscreen` 实际构造和关闭主窗口；
- 检查 CSV、JSON 和 PNG 可实际写出并可解析。

## AI 错误与修复记录

### 重采样切角

首版弧长采样没有保留 LOS 折线的关键拐点。真实飞行没有碰撞，但规划验收
发现某条新连接线与膨胀 AABB 精确相切。修复为保留所有原拐点，并同时执行
`<=0.05 m` 离散检查和精确线段/AABB 检查。相同场景复测 PASS。

### 验收计时

基础验收必须把 DDS discovery 与目标发布后的收敛计时分开。外层 launch
退出码不能替代节点的明确 PASS 记录、节点退出码和 wrapper 判断。

### Fast DDS 与 WSL

自动验收只在进程环境中设置 `ROS_LOCALHOST_ONLY=1` 和仓库内
`fastdds_acceptance.xml`，不修改 `~/.bashrc` 或系统 DDS 配置。wrapper
使用独立进程组并在中断时返回 130。

### RViz 参数作用域

`planned_sim.launch.py` 复用 `sim.launch.py` 时，内层 `rviz:=false` 必须放在
`GroupAction(scoped=True)` 中，避免覆盖外层规划 RViz 参数。实测只启动一个
RViz。

### 地图与规划验证

不能只凭 Marker 画面判断可通行；真正碰撞真值是 `drone_radius +
safety_margin` 膨胀 AABB。随机地图只有在项目 A* 返回成功后才可接受。

### Qt 验证

仅能 import PyQt5 不等于 GUI 可用。本分支实际在 offscreen 平台构造窗口，
启动 ROS 后台线程，关闭窗口并确认线程结束。GUI 启动的仿真只按保存的
QProcess PID 发送进程组信号。

## 结论

AI 用于加速实现和查错，不代替工程判断。任何未被当前测试复现的指标、
功能或界面行为都不得写成“已完成”。历史结果只有在当前 commit、当前参数
和明确原始日志下复现后，才能进入 README 或后续报告。

### Web 单页与巡航收尾

后续在人工确认 Qt 单点飞行成功后，增加离线单页 Web 地面站、点击目标、
等轴测视图和重复巡航状态机。实现中特别保留了三条人工约束：浏览器目标必须
先通过膨胀障碍物检查；每段巡航仍进入现有 3D A*；任何 UI 都不得在检测到
外部仿真时启动第二套同名节点。任务记录在权威状态源终止时冻结，单点使用
planner，巡航使用 patrol manager，解决此前导出摘要状态矛盾和悬停数据覆盖
完整飞行历史的问题。
