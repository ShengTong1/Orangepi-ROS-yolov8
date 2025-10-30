#!/bin/bash

# Qt ROS2 Robot Control Panel - Build and Run Script
# 编译并运行Qt机器人控制面板

echo "========================================="
echo "Qt机器人控制面板 - 编译和运行脚本"
echo "========================================="

# 进入工作空间根目录
cd /home/HwHiAiUser/wheeltec_ros2

# Source ROS2环境
source /opt/ros/humble/setup.bash

# 编译包
echo ""
echo "正在编译 qt_robot_control 包..."
colcon build --packages-select qt_robot_control

# 检查编译结果
if [ $? -eq 0 ]; then
    echo ""
    echo "✓ 编译成功!"
    
    # Source编译后的工作空间
    echo ""
    echo "Source工作空间..."
    source install/setup.bash
    
    # 提示用户如何运行
    echo ""
    echo "========================================="
    echo "编译完成! 使用以下命令运行:"
    echo "========================================="
    echo ""
    echo "方式1: ros2 launch qt_robot_control qt_robot_control.launch.py"
    echo "方式2: ros2 run qt_robot_control qt_robot_control"
    echo ""
    echo "========================================="
    
    # 询问是否立即运行
    read -p "是否立即运行? (y/n): " -n 1 -r
    echo ""
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        ros2 launch qt_robot_control qt_robot_control.launch.py
    fi
else
    echo ""
    echo "✗ 编译失败! 请检查错误信息。"
    exit 1
fi

