#!/bin/bash

# 测试脚本：验证GPIO控制节点是否正常工作

echo "=========================================="
echo "GPIO控制节点测试"
echo "=========================================="

# source环境
source /home/HwHiAiUser/wheeltec_ros2/install/setup.bash 2>/dev/null || true

echo ""
echo "1. 启动GPIO控制节点（使用launch文件）"
echo "请在新的终端窗口运行："
echo "   cd /home/HwHiAiUser/wheeltec_ros2"
echo "   source install/setup.bash"
echo "   ros2 launch wheeltec_gpio_control gpio_control.launch.py"
echo ""

echo "2. 查看节点是否运行："
echo "   ros2 node list | grep gpio"
echo ""

echo "3. 查看可用的话题："
echo "   ros2 topic list | grep gpio"
echo ""

echo "4. 查看GPIO状态："
echo "   ros2 topic echo /gpio_state"
echo ""

echo "5. 控制GPIO（设置为高电平）："
echo "   ros2 topic pub --once /gpio_control std_msgs/Int32 \"{data: 1}\""
echo ""

echo "6. 控制GPIO（设置为低电平）："
echo "   ros2 topic pub --once /gpio_control std_msgs/Int32 \"{data: 0}\""
echo ""

echo "7. 直接运行节点（支持键盘控制）："
echo "   ros2 run wheeltec_gpio_control gpio_control_node"
echo ""

echo "=========================================="


