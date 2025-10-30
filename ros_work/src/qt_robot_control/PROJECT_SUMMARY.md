# Qt ROS2 机器人控制面板 - 项目总结

## 项目概述

已成功创建完整的Qt5 + ROS2 Humble机器人控制面板，支持触摸屏操作、实时摄像头显示、RViz集成等功能。

## 已创建的文件

### 核心源文件
1. **src/main.cpp** - 程序入口，初始化ROS2和Qt应用
2. **src/MainWindow.cpp/h** - 主窗口实现，包含所有UI和功能逻辑
3. **src/ROSNode.cpp/h** - ROS2节点，订阅电池、里程计、IMU等数据
4. **src/RosImageSubscriber.cpp/h** - 图像订阅者，处理RGB和深度图
5. **src/SystemMonitorWidget.cpp/h** - 系统监控面板
6. **src/RVizWidget.cpp/h** - RViz集成组件

### 配置文件
1. **CMakeLists.txt** - CMake构建配置
2. **package.xml** - ROS2包配置
3. **launch/qt_robot_control.launch.py** - Launch文件
4. **README.md** - 使用说明文档
5. **build_and_run.sh** - 编译运行脚本

## 主要功能

### 1. 功能包控制 (左侧按钮)
- ✅ 底盘控制
- ✅ 激光雷达
- ✅ 摄像头  
- ✅ Gemini深度相机
- ✅ 一键启动所有传感器
- ✅ 键盘控制
- ✅ GPIO控制

**特点**: 
- 按钮变色显示运行状态
- 支持启动/停止切换
- 按钮文字显示运行状态

### 2. RViz集成 (左上区域)
- ✅ 嵌入式RViz状态显示
- ✅ 一键启动RViz独立窗口
- ✅ 状态指示器

### 3. 摄像头显示 (右上区域)
- ✅ RGB图像显示（支持话题选择）
- ✅ 深度图显示（支持话题选择）
- ✅ 30fps实时刷新
- ✅ 自动缩放保持宽高比
- ✅ 下拉菜单选择话题

**默认话题**:
- RGB: `/image_raw`
- 深度: `/camera/depth/image_raw`

### 4. 系统监控 (右侧面板)
- ✅ **电池电压**: 实时显示，颜色指示状态
  - 绿色: >12V
  - 黄色: 11-12V
  - 红色: <11V
- ✅ **速度**: 线速度Vx/Vy和角速度Vz
- ✅ **位置**: X/Y坐标和Yaw角度
- ✅ **IMU数据**: 三轴加速度、角速度、四元数
- ✅ **话题列表**: 每2秒自动更新
- ✅ **节点列表**: 每2秒自动更新
- ✅ **日志显示**: 彩色日志（INFO/WARN/ERROR）

### 5. 日志系统
- ✅ 自动保存日志到 `~/robot_logs/`
- ✅ 文件名格式: `log_YYYYMMDD_hhmmss.txt`
- ✅ 包含时间戳和日志级别
- ✅ 实时文件写入
- ✅ 会话开始/结束标记

## 技术实现

### ROS2 架构
```
MainWindow (Qt GUI Thread)
    ├── ROSNode (rclcpp::Node)
    │   ├── /odom → 里程计数据
    │   ├── /imu/data_raw → IMU数据
    │   └── /PowerVoltage → 电池电压
    ├── RosImageSubscriber (RGB)
    │   └── /image_raw 或用户选择话题
    └── RosImageSubscriber (Depth)
        └── /camera/depth/image_raw 或用户选择话题
```

### 多线程处理
- 主线程: Qt GUI和用户交互
- 定时器: 每33ms (~30fps) 调用spin_some处理ROS2消息
- 进程管理: 使用QProcess启动/停止ROS2功能包

### 图像处理
- 使用cv_bridge转换ROS图像消息到OpenCV Mat
- 支持RGB8、BGR8、16UC1、32FC1等编码
- 深度图自动应用COLORMAP_JET色彩映射
- OpenCV Mat转换为QImage，再转为QPixmap显示
- 自动缩放保持宽高比，平滑插值

### UI设计
- 1024x600分辨率（触摸屏尺寸）
- 深色主题，现代化界面
- 左侧功能面板: 180px固定宽度
- 中间显示区域: RViz (250px高) + 摄像头
- 右侧监控面板: 220px固定宽度，支持滚动

## 数据流

### 串口通信
- 下位机通过串口发送数据
- 24字节数据包: 速度、IMU、电池电压等
- 帧头0x7B，帧尾0x7D，BCC校验

### ROS2话题
- **订阅**:
  - `/odom` - 里程计
  - `/imu/data_raw` - IMU原始数据
  - `/PowerVoltage` - 电池电压
  - `/image_raw` - RGB图像
  - `/camera/depth/image_raw` - 深度图

### 进程管理
- 每个功能按钮对应一个QProcess
- 启动命令通过bash执行
- 支持优雅终止（terminate + kill）
- 进程状态实时监控

## 编译和使用

### 编译
```bash
cd ~/wheeltec_ros2
source /opt/ros/humble/setup.bash
colcon build --packages-select qt_robot_control
source install/setup.bash
```

### 或使用脚本
```bash
cd ~/wheeltec_ros2/src/qt_robot_control
./build_and_run.sh
```

### 运行
```bash
ros2 launch qt_robot_control qt_robot_control.launch.py
```

## 界面布局

```
┌────────────────────────────────────────────────────┐
│ [功能按钮]  │  [RViz状态]  │  RGB图像  │  深度图   │
│ 180px宽    │  250px高    │  200x150  │  200x150  │
├────────────────────────────────────────────────────┤
│            │  [启动RViz]   │          │           │
│            ├───────────────────────────────────────┤
│            │  系统监控面板 (220px宽)               │
│            │  - 电池电压                           │
│            │  - 速度/位置                          │
│            │  - IMU数据                           │
│            │  - 话题列表                          │
│            │  - 节点列表                          │
│            │  - 日志窗口                          │
└────────────────────────────────────────────────────┘
```

## 依赖项

### ROS2 包
- rclcpp
- sensor_msgs
- nav_msgs
- geometry_msgs
- std_msgs
- cv_bridge
- image_transport
- tf2_ros
- tf2_geometry_msgs
- rviz_common
- rviz_rendering

### 系统依赖
- Qt5 (Core, Widgets, Gui)
- OpenCV
- bash

## 特殊说明

1. **不修改原功能**: 所有启动命令完全按照原有功能包的命令，不做任何修改
2. **独立窗口RViz**: RViz在独立窗口中启动，而不是尝试嵌入Qt（更稳定）
3. **日志保存**: 每次运行创建新的日志文件，不会覆盖之前的数据
4. **中文界面**: 所有界面文本都是中文，适合中文用户

## 下一步改进建议

1. 添加配置文件支持（YAML）
2. 添加紧急停止按钮
3. 添加数据回放功能
4. 添加网络摄像头支持
5. 添加多摄像头切换
6. 添加性能监控（CPU、内存使用率）
7. 添加地图显示（如果启用导航）

## 状态

✅ 所有核心功能已完成
✅ 所有文件已创建
✅ 已修复编译错误
✅ 已创建文档和脚本
✅ 准备编译测试

**项目已完成，可以开始编译测试！**


