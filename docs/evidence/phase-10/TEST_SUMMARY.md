# 阶段 10：PyQt5 地面站与导出证据

## 已完成工作

- [x] 新增 `drone_ground_station` ament_python package；
- [x] 提供 `ros2 run` 与独立 `ground_station.launch.py`；
- [x] Qt 主线程只更新 GUI；
- [x] rclpy Node 与 SingleThreadedExecutor 运行在后台 QThread；
- [x] ROS 回调只通过 Qt signal 传递 Python 数据；
- [x] QProcess 只管理自身创建的 `setsid` 进程组；
- [x] 提供系统、地图、飞行、RPM、规划、任务、测试和日志页面；
- [x] 支持地图/自定义 YAML、规划参数校验和 JSON profile；
- [x] 支持单目标、多航点、正方形、圆形和八字形；
- [x] 支持 CSV、PNG、JSON 和窗口截图导出；
- [x] Vehicle Count 被限制为 1；
- [x] 增加 profile 与 ring buffer 单元测试；
- [x] 实际执行 offscreen GUI 构造和关闭测试。
- [x] 实际生成并解析 CSV、JSON、轨迹 PNG 与窗口截图。

## GUI 自动运行

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
QT_QPA_PLATFORM=offscreen python3 - <<'PY'
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication
from drone_ground_station.main_window import GroundStationWindow

app = QApplication([])
window = GroundStationWindow()
window.show()
QTimer.singleShot(1500, window.close)
code = app.exec_()
print("GUI_SMOKE_EXIT", code, "ROS_THREAD_RUNNING", window._ros.isRunning())
PY
```

实际记录：

```text
GUI_SMOKE_EXIT 0 ROS_THREAD_RUNNING False
EXPORT_FILES telemetry.csv mission_summary.json trajectory.png ground_station.png
```

这证明窗口确实构造、Qt event loop 实际运行、关闭后 ROS 线程已经停止。
视觉检查发现精简 WSL 默认没有 CJK 字体；通过
`DRONE_GROUND_STATION_FONT_FILE` 加载已有中文字体后，复测截图文字正常。
正式 Ubuntu 环境推荐安装 `fonts-noto-cjk`。offscreen 测试不能代替最终演示时的
人工点击检查。

## 导出

`GroundStationWindow.export_to_directory()` 使用当前真实 ring buffer 和状态
生成：

- `telemetry.csv`；
- `trajectory.png`；
- `mission_summary.json`。
- `ground_station.png`（“保存截图”或自动冒烟检查）。

JSON 包含地图、轨迹类型、目标、参数、起止时间、最大速度、最大/最终位置误差、
最小净空、规划/实际路径长度、RPM 饱和计数、碰撞数和完成状态。

批量 wrapper 的日志由 `scripts/export_showcase_results.py` 生成实验 CSV、
JSON 和 PNG。导出目录位于 `output/` 且默认不进入 Git。

## GUI 截图说明

主窗口采用左侧导航、右侧工作区：

- 总览显示 ROS/仿真/规划状态和进程按钮；
- 地图页叠加原始障碍物、规划路径、实际轨迹和当前位置；
- 飞行/RPM 页使用固定长度 ring buffer；
- 规划页显示参数表和解析后的 JSON；
- 任务页支持单目标和预定义/自定义轨迹；
- 日志页显示 QProcess 输出。

本阶段没有把临时 offscreen 截图加入 Git；最终视频/报告阶段应在有显示服务的
环境中截取真实运行画面。

## 已知限制

- 当前仅单机；
- 自定义航点使用文本输入，尚未实现完整的可排序表格编辑器；
- 地图半径/余量/分辨率保存在 profile 中，但运行碰撞真值仍以所选 YAML 为准；
- Qt 二维视图不替代 RViz；
- 尚未执行人工点击覆盖全部控件的 GUI 集成测试；
- 当前精简 WSL 未安装系统 CJK 字体，运行时需安装字体或设置字体文件环境变量；
- 本轮不生成最终 LaTeX/PDF/视频。
