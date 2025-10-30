# Qt ROS2 机器人控制面板

基于Qt5和ROS2 Humble开发的机器人控制面板，支持触摸屏操作、实时摄像头显示、RViz集成等功能。

## 功能特性

- ✅ **功能包启动控制**: 一键启动/停止底盘、激光雷达、摄像头等ROS2功能包
- ✅ **实时摄像头显示**: 支持RGB和深度图同时显示（30fps）
- ✅ **RViz集成**: 嵌入式RViz可视化显示（可通过按钮启动独立窗口）
- ✅ **系统监控**: 实时显示电池电压、速度、IMU数据、TF树等
- ✅ **话题/节点列表**: 动态显示ROS2话题和节点列表
- ✅ **日志记录**: 自动保存运行日志到 `~/robot_logs/`
- ✅ **中文界面**: 完整的中文用户界面

## 系统要求

- ROS2 Humble
- Qt5 (Core, Widgets, Gui)
- OpenCV
- cv_bridge
- rviz2

## 编译方法

```bash
cd ~/wheeltec_ros2
colcon build --packages-select qt_robot_control
source install/setup.bash
```

## 运行方法

### 方式1: 通过launch文件启动
```bash
ros2 launch qt_robot_control qt_robot_control.launch.py
```

### 方式2: 直接运行可执行文件
```bash
ros2 run qt_robot_control qt_robot_control
```

## 使用方法

1. **启动功能包**: 点击左侧功能按钮启动相应的ROS2功能包，按钮颜色会变为红色表示正在运行，再次点击停止。

2. **RViz**: 点击"启动RViz"按钮，在独立窗口中打开RViz进行可视化。

3. **摄像头显示**: 
   - 使用下拉菜单选择RGB和深度话题
   - 默认话题: `/image_raw` 和 `/camera/depth/image_raw`
   - 图像自动以30fps刷新

4. **系统监控**: 右侧面板显示:
   - 电池电压（绿色=正常，黄色=低电量，红色=电量不足）
   - 速度信息（线速度和角速度）
   - 位置信息
   - IMU数据（加速度、角速度、四元数）
   - 话题列表（每2秒更新）
   - 节点列表（每2秒更新）
   - 运行日志

## 界面布局

```
┌─────────────────────────────────────────────────────────────┐
│ 左侧功能面板(180px) │  RViz区域(250高)  │  摄像头显示(200x150) │
│ [底盘控制]         │  [RViz状态]      │  RGB图   深度图     │
│ [激光雷达]         │  [启动RViz]      │                     │
│ [摄像头]           │  ────────────────│                     │
│ [Gemini深度相机]   │                  │                     │
│ [一键启动传感器]    │   系统监控面板     │                     │
│ [键盘控制]         │  - 电池电压       │                     │
│ [GPIO控制]         │  - 速度/位置      │                     │
│                    │  - IMU数据       │                     │
│                    │  - TF树          │                     │
│                    │  - 话题列表       │                     │
│                    │  - 节点列表       │                     │
│                    │  - 日志          │                     │
└─────────────────────────────────────────────────────────────┘
```

## 日志文件

日志自动保存到 `~/robot_logs/log_YYYYMMDD_hhmmss.txt`

## 快捷键

- 无（使用触摸屏或鼠标操作）

## 故障排除

### 摄像头无法显示
- 确保已启动摄像头功能包
- 检查话题名称是否正确
- 使用 `ros2 topic list` 查看可用话题

### RViz无法启动
- 确保已安装rviz2: `sudo apt install ros-humble-rviz2`
- 检查环境变量: `source /opt/ros/humble/setup.bash`

### 功能包启动失败
- 确保已source工作空间: `source ~/wheeltec_ros2/install/setup.bash`
- 检查功能包是否正确编译
- 查看终端错误信息

## 许可证

Apache-2.0

## 作者

Wheeltec

