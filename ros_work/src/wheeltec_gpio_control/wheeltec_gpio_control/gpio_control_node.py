#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import select
import termios
import tty
import subprocess
import threading
import time
import rclpy
from std_msgs.msg import Int32
from rclpy.qos import QoSProfile


class GPIOControlNode:
    def __init__(self):
        rclpy.init()
        
        # 创建节点
        self.node = rclpy.create_node('gpio_control_node')
        
        # GPIO配置
        self.gpio_group = 7
        self.gpio_pin = 5
        self.current_state = 0  # 0=低电平, 1=高电平
        
        # QoS配置
        qos = QoSProfile(depth=10)
        
        # 创建发布器
        self.state_publisher = self.node.create_publisher(
            Int32, 
            'gpio_state', 
            qos
        )
        
        # 创建订阅器
        self.control_subscriber = self.node.create_subscription(
            Int32,
            'gpio_control',
            self.control_callback,
            qos
        )
        
        # 检查是否支持键盘输入（stdin是tty设备）
        self.support_keyboard = False
        self.settings = None
        if os.name != 'nt' and sys.stdin.isatty():
            try:
                self.settings = termios.tcgetattr(sys.stdin)
                self.support_keyboard = True
            except (termios.error, AttributeError):
                self.support_keyboard = False
                self.settings = None
        
        # 键盘输入线程标志
        self.keyboard_running = True
        
        self.node.get_logger().info('GPIO控制节点已启动')
        self.node.get_logger().info(f'控制GPIO: {self.gpio_group}_{self.gpio_pin:02d}')
        
        if self.support_keyboard:
            self.node.get_logger().info('键盘控制: 1=拉高, 0=拉低, s=查看状态, q=退出')
        else:
            self.node.get_logger().info('键盘控制已禁用（非终端模式），仅支持ROS topic控制')
        
    def control_callback(self, msg):
        """ROS topic回调函数"""
        target_state = msg.data
        if target_state not in [0, 1]:
            self.node.get_logger().warn(f'无效的GPIO值: {target_state}, 应为0或1')
            return
        
        self.set_gpio_state(target_state, 'ROS topic')
    
    def set_gpio_state(self, state, source=''):
        """设置GPIO状态"""
        try:
            # 方法1：尝试使用gpio_operate命令（需要root权限）
            cmd = f'sudo gpio_operate set_value {self.gpio_group} {self.gpio_pin:02d} {state}'
            result = subprocess.run(
                cmd, 
                shell=True, 
                capture_output=True, 
                text=True, 
                timeout=2
            )
            
            if result.returncode == 0:
                self.current_state = state
                state_str = '高电平' if state == 1 else '低电平'
                self.node.get_logger().info(
                    f'[{source}] GPIO {self.gpio_group}_{self.gpio_pin:02d} 设置为: {state_str}'
                )
                return True
            else:
                # 如果gpio_operate失败，尝试使用标准Linux GPIO接口
                self.node.get_logger().warn(f'gpio_operate失败: {result.stderr}')
                return self.set_gpio_state_linux(state, source)
                
        except subprocess.TimeoutExpired:
            self.node.get_logger().error('GPIO设置超时')
            return False
        except Exception as e:
            self.node.get_logger().error(f'GPIO设置异常: {str(e)}')
            return False
    
    def set_gpio_state_linux(self, state, source=''):
        """使用标准Linux GPIO接口设置GPIO状态"""
        try:
            # 计算GPIO编号：对于Orange Pi，GPIO 7_05 对应 gpiochip508 + pin 5
            gpio_number = 508 + self.gpio_pin  # gpiochip508 + pin 5 = gpio513
            
            # 检查GPIO是否已导出
            gpio_path = f'/sys/class/gpio/gpio{gpio_number}'
            if not os.path.exists(gpio_path):
                # 导出GPIO
                with open('/sys/class/gpio/export', 'w') as f:
                    f.write(str(gpio_number))
                time.sleep(0.1)  # 等待导出完成
            
            # 设置方向为输出
            direction_path = f'{gpio_path}/direction'
            with open(direction_path, 'w') as f:
                f.write('out')
            
            # 设置GPIO值
            value_path = f'{gpio_path}/value'
            with open(value_path, 'w') as f:
                f.write(str(state))
            
            self.current_state = state
            state_str = '高电平' if state == 1 else '低电平'
            self.node.get_logger().info(
                f'[{source}] GPIO {self.gpio_group}_{self.gpio_pin:02d} (Linux gpio{gpio_number}) 设置为: {state_str}'
            )
            return True
            
        except PermissionError:
            self.node.get_logger().error('GPIO权限不足，请确保用户有访问/sys/class/gpio的权限')
            return False
        except Exception as e:
            self.node.get_logger().error(f'Linux GPIO设置失败: {str(e)}')
            return False
    
    def get_gpio_state(self):
        """读取GPIO当前状态（可选功能，如果gpio_operate支持读取）"""
        # 注意：如果gpio_operate不支持读取，可以尝试其他方法
        # 这里返回当前记录的状态
        return self.current_state
    
    def print_status(self):
        """打印当前GPIO状态"""
        state_str = '高电平' if self.current_state == 1 else '低电平'
        self.node.get_logger().info(
            f'GPIO {self.gpio_group}_{self.gpio_pin:02d} 当前状态: {state_str} ({self.current_state})'
        )
    
    def get_key(self):
        """获取键盘输入（非阻塞）"""
        if not self.support_keyboard or self.settings is None:
            return ''
        
        if os.name == 'nt':
            import msvcrt
            if msvcrt.kbhit():
                return msvcrt.getch().decode('utf-8')
            return ''
        
        try:
            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                key = sys.stdin.read(1)
            else:
                key = ''
            
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            return key
        except (termios.error, OSError, AttributeError):
            return ''
    
    def keyboard_thread(self):
        """键盘输入处理线程"""
        while self.keyboard_running:
            key = self.get_key()
            
            if key == '1':
                self.set_gpio_state(1, '键盘')
            elif key == '0':
                self.set_gpio_state(0, '键盘')
            elif key == 's':
                self.print_status()
            elif key == 'q':
                self.node.get_logger().info('接收到退出指令')
                self.keyboard_running = False
                break
            elif key == '\x03':  # Ctrl+C
                self.node.get_logger().info('接收到Ctrl+C')
                self.keyboard_running = False
                break
            
            time.sleep(0.01)  # 10ms延迟
    
    def publish_state_loop(self):
        """发布GPIO状态循环"""
        timer_rate = 0.1  # 10Hz = 0.1s
        
        while self.keyboard_running:
            msg = Int32()
            msg.data = self.current_state
            self.state_publisher.publish(msg)
            time.sleep(timer_rate)
    
    def run(self):
        """运行主循环"""
        try:
            # 启动键盘输入线程（仅当支持键盘输入时）
            if self.support_keyboard:
                keyboard_thread = threading.Thread(target=self.keyboard_thread)
                keyboard_thread.daemon = True
                keyboard_thread.start()
            
            # 启动状态发布线程
            publish_thread = threading.Thread(target=self.publish_state_loop)
            publish_thread.daemon = True
            publish_thread.start()
            
            # 主ROS循环
            while rclpy.ok():
                rclpy.spin_once(self.node, timeout_sec=0.1)
                
                # 如果没有键盘支持，不需要检查keyboard_running
                if self.support_keyboard and not self.keyboard_running:
                    break
                
        except KeyboardInterrupt:
            self.node.get_logger().info('接收到中断信号')
        except Exception as e:
            self.node.get_logger().error(f'运行异常: {str(e)}')
        finally:
            self.cleanup()
    
    def cleanup(self):
        """清理资源"""
        self.keyboard_running = False
        self.node.get_logger().info('正在关闭GPIO控制节点...')
        
        # 恢复终端设置（仅当支持键盘输入时）
        if self.support_keyboard and os.name != 'nt' and self.settings is not None:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            except (termios.error, AttributeError):
                pass
        
        # 可选：退出时重置GPIO（如果需要）
        # self.set_gpio_state(0, '清理')
        
        self.node.destroy_node()
        rclpy.shutdown()


def main(args=None):
    """主函数"""
    try:
        node = GPIOControlNode()
        node.run()
    except KeyboardInterrupt:
        print('程序被用户中断')
    except Exception as e:
        print(f'程序异常: {str(e)}')
    finally:
        # 确保ROS2正确关闭
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()

