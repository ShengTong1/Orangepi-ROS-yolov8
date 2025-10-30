# wheeltec_gpio_control

GPIO控制ROS2包，用于控制Orange Pi AI Pro开发板的GPIO引脚。

## 功能特性

- 支持键盘实时控制GPIO高低电平
- 支持ROS2 topic方式控制GPIO
- 自动发布GPIO状态到topic（10Hz）
- 键盘控制和topic控制并行工作
- 支持查看GPIO当前状态

## 默认GPIO配置

- GPIO组：7
- GPIO引脚：05
- 默认初始状态：低电平 (0)

## 使用方法

### 1. 启动GPIO控制节点

#### 方法1：使用launch文件（推荐）

```bash
ros2 launch wheeltec_gpio_control gpio_control.launch.py
```

#### 方法2：直接运行节点

```bash
ros2 run wheeltec_gpio_control gpio_control_node
```

### 2. 键盘控制

启动节点后，在终端中可以使用以下按键：

- `1` - 拉高GPIO（设置为高电平）
- `0` - 拉低GPIO（设置为低电平）
- `s` - 查看当前GPIO状态
- `q` - 退出程序

### 3. ROS2 Topic控制

#### 查看GPIO状态

```bash
ros2 topic echo /gpio_state
```

#### 控制GPIO

```bash
# 设置为高电平
ros2 topic pub --once /gpio_control std_msgs/Int32 "{data: 1}"

# 设置为低电平
ros2 topic pub --once /gpio_control std_msgs/Int32 "{data: 0}"
```

#### 持续发布控制（交互式）

```bash
ros2 topic pub -r 1 /gpio_control std_msgs/Int32 "{data: 1}"
```

### 4. Topic列表

#### 发布的话题

- `/gpio_state` (std_msgs/Int32)
  - 频率：10Hz
  - 内容：当前GPIO状态 (0=低电平, 1=高电平)

#### 订阅的话题

- `/gpio_control` (std_msgs/Int32)
  - 内容：目标GPIO状态 (0=低电平, 1=高电平)

## 实现细节

- 使用 `sudo gpio_operate set_value` 命令控制GPIO
- 键盘输入使用非阻塞模式，不影响ROS2节点运行
- 状态发布和键盘控制运行在不同线程中，互不干扰
- 支持键盘和topic同时控制（后到先得）

## 注意事项

1. 需要sudo权限执行 `gpio_operate` 命令
2. 确保 `gpio_operate` 工具在系统PATH中
3. 如果GPIO被占用，可能无法正常控制
4. 退出程序不会自动重置GPIO状态
5. **键盘控制模式**：仅当在真实终端环境中运行时才支持键盘输入（如直接运行`ros2 run`）
6. **无终端模式**：当使用`ros2 launch`启动时，键盘控制会自动禁用，仅支持ROS topic控制

## 依赖

- rclpy
- std_msgs

## 日志输出示例

```
[INFO] [gpio_control_node]: GPIO控制节点已启动
[INFO] [gpio_control_node]: 控制GPIO: 7_05
[INFO] [gpio_control_node]: 键盘控制: 1=拉高, 0=拉低, s=查看状态, q=退出
[INFO] [gpio_control_node]: [键盘] GPIO 7_05 设置为: 高电平
[INFO] [gpio_control_node]: GPIO 7_05 当前状态: 高电平 (1)
[INFO] [gpio_control_node]: [ROS topic] GPIO 7_05 设置为: 低电平
```

## 许可证

TODO: License declaration

