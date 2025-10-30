#!/bin/bash

# GPIO权限修复脚本
# 用于解决GPIO控制节点的权限问题

echo "=== GPIO权限修复脚本 ==="

# 检查当前用户
echo "当前用户: $(whoami)"
echo "当前用户组: $(groups)"

# 检查GPIO设备权限
echo -e "\n=== 检查GPIO设备权限 ==="
ls -la /sys/class/gpio/

# 检查gpio_operate命令
echo -e "\n=== 检查gpio_operate命令 ==="
if [ -f "/usr/bin/gpio_operate" ]; then
    echo "gpio_operate命令存在"
    ls -la /usr/bin/gpio_operate
else
    echo "gpio_operate命令不存在"
fi

# 尝试添加用户到gpio组（如果存在）
echo -e "\n=== 尝试添加用户到gpio组 ==="
if getent group gpio > /dev/null 2>&1; then
    echo "gpio组存在，尝试添加用户..."
    sudo usermod -a -G gpio $(whoami)
    echo "用户已添加到gpio组，请重新登录生效"
else
    echo "gpio组不存在"
fi

# 检查sudo权限
echo -e "\n=== 检查sudo权限 ==="
if sudo -n true 2>/dev/null; then
    echo "用户有无密码sudo权限"
else
    echo "用户需要密码才能使用sudo"
    echo "建议配置无密码sudo权限："
    echo "sudo visudo"
    echo "添加行: $(whoami) ALL=(ALL) NOPASSWD: /usr/bin/gpio_operate"
fi

# 测试GPIO访问
echo -e "\n=== 测试GPIO访问 ==="
if [ -w "/sys/class/gpio/export" ]; then
    echo "用户可以直接访问/sys/class/gpio/export"
else
    echo "用户无法直接访问/sys/class/gpio/export，需要root权限"
fi

echo -e "\n=== 修复建议 ==="
echo "1. 如果使用gpio_operate命令，需要配置sudo权限"
echo "2. 如果使用Linux GPIO接口，需要用户有访问/sys/class/gpio的权限"
echo "3. 修改后的代码会自动尝试两种方法"
echo "4. 重新编译并运行节点测试"

echo -e "\n=== 重新编译节点 ==="
cd /home/HwHiAiUser/wheeltec_ros2
colcon build --packages-select wheeltec_gpio_control
source install/setup.bash

echo -e "\n=== 测试节点 ==="
echo "运行以下命令测试："
echo "ros2 run wheeltec_gpio_control gpio_control_node"
