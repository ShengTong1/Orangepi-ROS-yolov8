# Wheeltec ROS2 系统架构深度解析

## 目录
1. [系统概述](#系统概述)
2. [核心包架构](#核心包架构)
3. [启动文件分析](#启动文件分析)
4. [数据流图](#数据流图)
5. [包依赖关系](#包依赖关系)
6. [传感器集成](#传感器集成)
7. [TF树结构](#TF树结构)

---

## 系统概述

这是一个完整的ROS2移动机器人底盘控制系统，适用于Humble版本。系统包含底盘控制、传感器融合、多机器人模型支持等功能。

### 主要特性
- 多种底盘类型支持（差速、阿克曼、麦克纳姆轮、全向轮等）
- 串口通信协议与下位机交互
- EKF扩展卡尔曼滤波融合里程计和IMU数据
- 支持多种激光雷达（LD、LS、Rplidar等）
- 支持多种摄像头（Astra系列、USB摄像头）
- 自动回充功能
- 灵活的URDF模型配置

---

## 核心包架构

### 1. turn_on_wheeltec_robot (核心控制包)
**路径**: `/src/turn_on_wheeltec_robot/`

这是整个系统的核心控制包，负责与下位机通信、发布传感器数据和里程计信息。

#### 主要节点
- **wheeltec_robot_node**: 主要的机器人控制节点

#### 核心功能
1. **串口通信**
   - 端口: `/dev/ttyACM0` (可在launch中配置)
   - 波特率: 115200
   - 协议: 自定义二进制协议
   - 数据包格式: 帧头(0x7B) + 数据 + 校验 + 帧尾(0x7D)

2. **数据发布**
   - `/odom` - 里程计数据 (nav_msgs/Odometry)
   - `/imu/data_raw` - IMU原始数据 (sensor_msgs/Imu)
   - `/PowerVoltage` - 电池电压 (std_msgs/Float32)
   - `/robot_charging_flag` - 充电状态 (std_msgs/Bool)
   - `/robot_charging_current` - 充电电流 (std_msgs/Float32)
   - `/robot_red_flag` - 红外信号 (std_msgs/UInt8)

3. **数据订阅**
   - `/cmd_vel` - 速度控制命令 (geometry_msgs/Twist)
   - `/red_vel` - 红外对接速度 (geometry_msgs/Twist)
   - `/robot_recharge_flag` - 回充标志 (std_msgs/Int8)

4. **服务提供**
   - `/set_charge` - 设置充电模式 (turtlesim::srv::Spawn)

#### 关键数据结构
```cpp
// 接收数据结构 (24字节)
struct RECEIVE_DATA {
    uint8_t rx[24];
    uint8_t Flag_Stop;      // 预留位
    float X_speed;          // X方向速度
    float Y_speed;          // Y方向速度
    float Z_speed;          // Z轴角速度
    float Power_Voltage;    // 电源电压
};

// IMU数据结构
struct MPU6050_DATA {
    short accele_x/y/z_data;   // 三轴加速度
    short gyros_x/y/z_data;    // 三轴角速度
};
```

#### 关键源码分析

**`wheeltec_robot.cpp`** - 主要控制逻辑:
- `Get_Sensor_Data_New()`: 逐帧读取串口数据并校验
- `Publish_Odom()`: 发布里程凭借据，支持协方差矩阵
- `Publish_ImuSensor()`: 发布IMU原始数据
- `Cmd_Vel_Callback()`: 接收速度命令并发送给下位机
- `Control()`: 主循环，定时获取传感器数据并发布话题

**关键算法**:
```cpp
// 里程计积分算法
Robot_Pos.X += (Robot_Vel.X * cos(Robot_Pos.Z) - Robot_Vel.Y * sin(Robot_Pos.Z)) * Sampling_Time;
Robot_Pos.Y += (Robot_Vel.X * sin(Robot_Pos.Z) + Robot_Vel.Y * cos(Robot_Pos.Z)) * Sampling_Time;
Robot_Pos.Z += Robot_Vel.Z * Sampling_Time;
```

### 2. wheeltec_robot_urdf (机器人模型包)
**路径**: `/src/wheeltec_robot_urdf/wheeltec_robot_urdf/`

包含多种机器人模型的URDF文件。

#### 支持的机器人类型
- **Minibot系列**: mini_mec, mini_akm, mini_tank, mini_4wd, mini_diff, mini_omni
- **Senior系列**: senior_akm, senior_mec_bs, senior_mec_dl, senior_omni, senior_diff
- **Top系列**: top_akm_bs, top_akm_dl, top_mec_bs, top_mec_dl, top_4wd_bs, top_4wd_dl, top_omni
- **Flagship系列**: flagship_mec_bs, flagship_mec_dl, flagship_4wd_bs, flagship_4wd_dl
- **其他**: r3s_mec, r3s_4wd, brushless_senior_diff, S100_diff, four_wheel_diff_bs/dl

### 3. wheeltec_robot_msg (消息定义包)
**路径**: `/src/wheeltec_robot_msg/`

定义自定义消息类型。

### 4. wheeltec_gpio_control (GPIO控制包)
**路径**: `/src/wheeltec_gpio_control/`

用于GPIO控制，可能与底盘上的数字IO控制相关。

### 5. wheeltec_imu (IMU驱动包)
**路径**: `/src/wheeltec_imu/`

支持Yesense系列IMU驱动。

### 6. wheeltec_lidar_ros2 (激光雷达包)
**路径**: `/src/wheeltec_lidar_ros2/`

支持多种激光雷达:
- LDlidar系列 (ld06, ld14, ld19, stl06n等)
- LSlidar系列 (lsn10, lsn10p, lsm10, lsm10p等)
- Rplidar系列 (rplidar_c1)

### 7. astra_camera (3D摄像头包)
**路径**: `/src/astra_camera/`

支持Orbbec Astra系列3D摄像头:
- Astra_S
- Astra_Pro
- Dabai (大白U3)
- Gemini

### 8. usb_cam-ros2 (USB摄像头包)
**路径**: `/src/usb_cam-ros2/`

支持标准USB摄像头。

### 9. web_video_server-ros2 (视频流服务器)
**路径**: `/src/web_video_server-ros2/`

用于在网页中查看摄像头视频流。

### 10. wheeltec_rviz2 (可视化配置包)
**路径**: `/src/wheeltec_rviz2/`

包含RViz配置文件。

---

## 启动文件分析

### 1. turn_on_wheeltec_robot.launch.py
**功能**: 核心启动文件，启动机器人底盘控制

**启动内容**:
```python
1. base_serial.launch.py - 启动底盘串口通信节点
2. wheeltec_ekf.launch.py - 启动EKF融合
3. robot_mode_description_minibot.launch.py - 加载机器人模型
4. base_to_link - TF静态变换 (base_footprint -> base_link)
5. base_to_gyro - TF静态变换 (base_footprint -> gyro_link)
6. joint_state_publisher - 发布关节状态
7. imu_filter_node - IMU滤波器 (Madgwick滤波器)
```

**关键参数**:
- `use_custom_urdf`: 是否使用自定义URDF (默认true)
- `custom_urdf_path`: 自定义URDF路径
- `carto_slam`: 是否使用Cartographer SLAM配置

### 2. base_serial.l burden.py
**功能**: 启动底盘串口通信节点

**关键配置**:
```python
parameters=[
    'usart_port_name': '/dev/ttyACM0',
    'serial_baud_rate': 115200,
    'robot_frame_id': 'base_footprint',
    'odom_frame_id': 'odom_combined',
    'cmd_vel': 'cmd_vel',
    'odom_x_scale': 1.0,
    'odom_y_scale': 1.0,
    'odom_z_scale_positive': 1.0,
    'odom_z_scale_negative': 1.0
]
```

### 3. wheeltec_ekf.launch.py
**功能**: 启动EKF扩展卡尔曼滤波节点

**数据源**:
- `odom0`: /odom (里程计数据)
- `imu0`: /imu/data_raw (IMU数据)

**融合内容**:
```yaml
odom0_config: 
  - 位置 x, y: false (不融合)
  - 姿态 yaw: true (融合Z轴角度)
  - 速度 vx, vy: true (融合线性速度)
  - 角速度 vyaw: true (融合角速度)

imu0_config:
  - 姿态 yaw: true (融合姿态)
  - 角速度 vyaw: true (融合角速度)
```

### 4. robot_mode_description_minibot.launch.py
**功能**: 加载不同机器人模型的URDF并发布静态TF

**支持的机器人**:
- r3s_mec, r3s_4wd
- mini_mec, mini_akm, mini_tank, mini_4wd, mini_diff, mini_omni
- brushless_senior_diff

**TF变换**:
每个机器人模型都配置了:
- `base_footprint -> laser` (激光雷达位置)
- `base_footprint -> camera_link` (摄像头位置)

### 5. wheeltec_sensors.launch.py
**功能**: 组合启动所有传感器

**启动内容**:
- turn_on_wheeltec_robot.launch.py
- wheeltec_lidar.launch.py
- wheeltec_camera.launch.py

### 6. wheeltec_lidar.launch.py
**功能**: 启动激光雷达节点

**支持的雷达**: Lsn10p (默认)

### 7. wheeltec_camera.launch.py
**功能**: 启动摄像头节点

**支持的摄像头**: Wheeltec_Usbcam (默认，/dev/RgbCam)

---

## 数据流图

```
┌─────────────────────────────────────────────────────────────┐
│                      上层控制节点                             │
│              (键盘控制/导航/自动回充等)                       │
└────────────────────────┬────────────────────────────────────┘
                         │ /cmd_vel (Twist)
                         │
┌────────────────────────▼────────────────────────────────────┐
│              turn_on_wheeltec_robot_node                     │
│  ┌──────────────────────────────────────────────────────┐   │
│  │ 1. 订阅 /cmd_vel                                     │   │
│  │ 2. 通过串口发送给下位机                              │   │
│  │ 3. 接收下位机串口数据                                │   │
│  │ 4. 发布里程计和IMU数据                               │   │
│  └──────────────────────────────────────────────────────┘   │
└─────┬──────────────────────────────┬───────────────────────┘
      │                              │
      │ /odom                        │ /imu/data_raw
      │                              │
┌─────▼─────────────────────┐  ┌────▼──────────────────────┐
│    EKF Filter Node         │  │  IMU Filter Node          │
│  (robot_localization)      │  │  (Madgwick)               │
│                            │  │                           │
│ - 融合里程计和IMU          │  │ - 处理IMU姿态             │
│ - 发布融合后的里程计        │  │ - 发布 /imu/data          │
│ - 发布 /odometry/filtered  │  │                           │
└─────┬─────────────────────┘  └──────────────────────────┘
      │
      │ /odometry/filtered
      │
┌─────▼──────────────────────────────────────────────────────┐
│                    机器人下位机                             │
│                  (STM32控制器)                              │
│                                                              │
│ - 读取编码器数据                                             │
│ - 读取IMU数据                                               │
│ - 执行运动控制                                               │
│ - 串口通信 (115200 baud)                                    │
└─────────────────────────────────────────────────────────────┘
```

---

## 包依赖关系

```
turn_on_wheeltec_robot (核心)
├── 依赖 ROS2 标准包:
│   ├── rclcpp, rclpy
│   ├── sensor_msgs, nav_msgs, geometry_msgs
│   ├── tf2, tf2_ros
│   └── std_msgs, std_srvs
├── 依赖 robot_localization:
│   └── ekf_node
├── 依赖 imu_filter_madgwick:
│   └── imu_filter_madgwick_node
├── 使用自定义消息:
│   └── wheeltec_robot_msg
└── 读取机器人模型:
    └── wheeltec_robot_urdf

wheeltec_sensors.launch.py 组合启动:
├── turn_on_wheeltec_robot (底盘控制)
├── wheeltec_lidar (激光雷达)
│   └── LSn10p/其他雷达驱动包
└── wheeltec_camera (摄像头)
    ├── astra_camera 或
    └── usb_cam
```

---

## 传感器集成

### 1. 里程计 (Odometry)
- **数据源**: 编码器 (通过串口从下位机获取)
- **发布频率**: 由串口数据更新速度决定
- **坐标系**: 
  - `odom_frame`: odom_combined
  - `child_frame`: base_footprint
- **数据内容**: 
  - 位置 (x, y)
  - 姿态 (yaw)
  - 速度 (vx, vy, vz)

### 2. IMU (惯性测量单元)
- **型号**: MPU6050/MPU9250 (或者Yesense系列)
- **量程**:
  - 加速度: ±2g
  - 陀螺仪: ±500°/s
- **数据转换**:
  ```cpp
  linear_acceleration = raw_data / ACCEl_RATIO  // ACCEl_RATIO = 1671.84
  angular_velocity = raw_data * GYROSCOPE_RATIO  // GYROSCOPE_RATIO = 0.00026644
  ```
- **滤波**: Madgwick滤波器处理原始IMU数据

### 3. 激光雷达
- **支持型号**: LSn10p (默认)
- **发布话题**: `/scan` (sensor_msgs/LaserScan)
- **TF**: laser -> base_footprint

### 4. 摄像头
- **支持类型**: USB摄像头/Astra系列
- **发布话题**: 
  - `/image_raw` (sensor_msgs/Image)
  - `/camera_info` (sensor_msgs/CameraInfo)

### 5. 电池电压
- **发布话题**: `/PowerVoltage` (std_msgs/Float32)
- **更新频率**: 每10次循环发布一次

---

## TF树结构

```
map (地图坐标系)
│
└── odom_combined (EKF融合后的里程计坐标系)
    │
    └── base_footprint (机器人底盘基座)
        │
        ├── base_link (机器人本体，需要时由URDF定义)
        │   ├── (轮子、底盘等物理部件)
        │   └── ...
        │
        ├── gyro_link (IMU坐标系)
        │
        ├── laser (激光雷达坐标系)
        │
        └── camera_link (摄像头坐标系)
```

**关键TF变换**:
1. `map -> odom_combined`: 由EKF节点动态发布
2. `odom_combined -> base_footprint`: 由wheeltec_robot_node发布
3. `base_footprint -> gyro_link`: 静态变换 (0, 0, 0)
4. `base_footprint -> laser`: 静态变换 (根据机器人模型不同而不同)
5. `base_footprint -> camera_link`: 静态变换 (根据机器人模型不同而不同)

---

## 关键配置参数说明

### 串口参数
- `usart_port_name`: 串口设备名，通常是 `/dev/ttyACM0` 或 `/dev/wheeltec_controller`
- `serial_baud_rate`: 波特率，默认115200

### 里程计标定参数
- `odom_x_scale`: X轴速度标定系数
- `odom_y_scale`: Y轴速度标定系数
- `odom_z_scale_positive`: Z轴正向角速度标定系数
- `odom_z_scale_negative`: Z轴负向角速度标定系数

### EKF配置
- `frequency`: EKF更新频率 (30Hz)
- `odom_frame`: 里程计坐标系名
- `base_link_frame`: 机器人本体坐标系名
- `world_frame`: 世界坐标系名 (通常等于odom_frame)

### IMU配置
- `gyro_frame_id`: IMU陀螺仪坐标系名
- 加速度量程: ±2g (19.6 m/s²)
- 陀螺仪量程: ±500°/s

---

## 启动方式

### 1. 完整启动 (推荐)
```bash
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py
```

### 2. 包含传感器
```bash
ros2 launch turn_on_wheeltec_robot龙凤_ sensors.launch.py
```

### 3. 自定义URDF启动
```bash
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py \
  use_custom_urdf:=true \
  custom_urdf_path:=/path/to/your/robot.urdf
```

---

## 调试技巧

### 1. 查看串口数据
```bash
# 查看串口原始数据
cat /dev/ttyACM0 | hexdump -C
```

### 2. 检查话题
```bash
ros2 topic list
ros2 topic echo /cmd_vel
ros2 topic echo /odom
ros2 topic echo /imu/data_raw
```

### 3. 检查TF树
```bash
ros2 run tf2_ros tf2_echo base_footprint laser
ros2 run tf2_tools view_frames
```

### 4. 检查串口权限
```bash
# 确保串口权限正确
sudo chmod 666 /dev/ttyACM0
# 或者将用户添加到dialout组
sudo usermod -a -G dialout $USER
```

---

## 常见问题

### 1. 串口无法打开
- 检查串口设备是否存在: `ls -l /dev/ttyACM*`
- 检查权限: 用户需要dialout组权限
- 检查是否有其他程序占用串口

### 2. 里程计不准确
- 调整标定参数 (odom_x/y/z_scale)
- 检查编码器连接
- 检查底盘物理参数

### 3. IMU数据异常
- 检查IMU连接
- 检查量程配置
- 检查单位转换系数

### 4. TF树异常
- 确保所有静态TF发布者都已启动
- 检查坐标系命名是否一致
- 使用tf2_tools查看完整的TF树

---

## 总结

这是一个功能完善的ROS2移动机器人底盘控制系统，具有以下特点:
1. **模块化设计**: 各功能包职责清晰，易于扩展
2. **多机器人支持**: 支持30+种不同底盘类型
3. **传感器融合**: 使用EKF融合里程计和IMU数据
4. **灵活配置**: 支持自定义URDF和参数调整
5. **完整通信**: 完善的串口通信协议和错误处理
6. **辅助功能**: 支持自动回充、电池监控等

系统设计遵循ROS2最佳实践，适合作为移动机器人开发的起点。

