#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
VR手柄摇杆控制机器人torso节点
通过订阅VR手柄的摇杆数据，实时控制机器人的torso关节运动
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from quest2_button_msg.msg import Quest2Button
from hdas_msg.msg import MotorControl
from std_msgs.msg import Bool
import numpy as np


class TorsoChassisVRControl(Node):
    def __init__(self):
        super().__init__('torso_chassis_vr_control')
        
        # 使用正确的QoS配置
        qos_torso = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        qos_brake = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        qos_button = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # 发布者
        self.torso_pub = self.create_publisher(
            MotorControl, '/motion_control/control_torso', qos_torso
        )
        self.chassis_pub = self.create_publisher(
            MotorControl, '/motion_control/control_chassis', qos_torso
        )
        self.brake_pub = self.create_publisher(
            Bool, '/motion_target/brake_mode', qos_brake
        )
        
        # 订阅VR手柄按钮数据
        self.create_subscription(
            Quest2Button, '/vr_button', self.button_callback, qos_button
        )
        
        # 当前torso关节位置 (3个关节)
        self.torso_joint_count = 3  # torso关节数量
        self.current_positions = [0.0, 0.0, 0.0]  # 存储3个关节的位置
        
        # 当前底盘状态
        # 底盘有3个轮子：[左前轮, 右前轮, 后轮]
        # p_des: [左前轮角度, 右前轮角度, 后轮角度]
        # v_des: [左前轮线速度, 右前轮线速度, 后轮线速度]
        self.chassis_wheel_count = 3  # 底盘轮子数量
        self.current_chassis_angles = [0.0, 0.0, 0.0]  # 当前轮子角度
        self.current_chassis_velocities = [0.0, 0.0, 0.0]  # 当前轮子线速度
        
        # 控制参数
        self.torso_max_speed = 0.1  # 最大速度 (rad/s) - 躯干控制
        self.chassis_max_speed = 0.5  # 底盘最大速度 (rad/s)
        self.dead_zone = 0.1  # 摇杆死区
        self.publish_rate = 50.0  # 发布频率 (Hz)
        
        # 控制增益 (与r1_lite_test_open_box.py保持一致)
        self.torso_v_des = [0.43, 0.7, 0.3, 0.3]  # 4个元素（消息格式要求）
        self.torso_kp = [140.0, 200.0, 120.0, 20.0]  # 4个元素（消息格式要求）
        self.torso_kd = [10.0, 50.0, 5.0, 1.0]  # 4个元素（消息格式要求）
        
        # 底盘控制增益（可根据实际情况调整）
        self.chassis_kp = [50.0, 50.0, 50.0]  # 角度控制增益
        self.chassis_kd = [5.0, 5.0, 5.0]  # 角度控制阻尼
        
        # 底盘控制参数
        self.chassis_max_angle_speed = 1.0  # 最大角度变化速度 (rad/s)
        self.chassis_max_linear_speed = 0.5  # 最大线速度 (m/s)
        
        # 关节限制 (3个关节，可根据实际情况调整)
        self.joint_limits_min = [-3.14, -3.14, -3.14]
        self.joint_limits_max = [3.14, 3.14, 3.14]
        
        # 底盘角度限制（可根据实际情况调整）
        self.chassis_angle_limits_min = [-1.57, -1.57, -1.57]  # -90度
        self.chassis_angle_limits_max = [1.57, 1.57, 1.57]  # +90度
        
        # 定时器用于持续发布控制命令
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        
        # 标志位：是否启用VR控制
        self.torso_control_enabled = False  # 躯干控制（按下右边G键时启用）
        self.chassis_control_enabled = False  # 底盘控制（按下左边G键时启用）
        
        # 上次接收到的摇杆数据
        self.last_joystick_left = [0.0, 0.0]
        self.last_joystick_right = [0.0, 0.0]
        
        # 上次的按键状态（用于检测按键状态变化）
        self.last_right_grip = False
        self.last_left_grip = False
        self.last_reset_triggered = False  # 防止零位复位重复触发
        
        self.get_logger().info('Torso VR控制节点已启动')
        self.get_logger().info('控制方式:')
        self.get_logger().info('  躯干控制: 按下右边G键 + 左右摇杆控制')
        self.get_logger().info('  摇杆映射:')
        self.get_logger().info('    左摇杆X -> 关节1')
        self.get_logger().info('    左摇杆Y -> 关节2')
        self.get_logger().info('    右摇杆Y -> 关节3')
        self.get_logger().info('  零位复位: 同时按下左右手柄的T+G键')
        self.get_logger().info('  底盘控制: 按下左边G键 + 左右摇杆控制')
        self.get_logger().info('    左摇杆Y -> 前进/后退速度')
        self.get_logger().info('    左摇杆X -> 转向角度')
        
    def button_callback(self, msg: Quest2Button):
        """处理VR手柄按钮和摇杆数据"""
        # 更新摇杆数据
        self.last_joystick_left = list(msg.left_joystick)
        self.last_joystick_right = list(msg.right_joystick)
        
        # 检测按键状态
        # lg: 左grip, rg: 右grip, ltr: 左trigger, rtr: 右trigger
        right_grip_pressed = msg.rg
        left_grip_pressed = msg.lg
        
        # 检测零位复位：同时按下左右手柄的T+G键
        left_tg_pressed = msg.ltr and msg.lg
        right_tg_pressed = msg.rtr and msg.rg
        reset_triggered = left_tg_pressed and right_tg_pressed
        
        if reset_triggered and not self.last_reset_triggered:
            # 同时按下左右手柄的T+G键，执行零位复位（只在刚按下时触发一次）
            self.current_positions = [0.0, 0.0, 0.0]
            self.get_logger().info('零位复位已执行 - 躯干已回到零位')
        
        # 更新零位复位触发状态
        self.last_reset_triggered = reset_triggered
        
        # 检测右边G键状态变化，控制躯干
        if right_grip_pressed and not self.last_right_grip:
            # 按下右边G键，启用躯干控制
            self.torso_control_enabled = True
            # 设置刹车模式
            brake_msg = Bool()
            brake_msg.data = True
            self.brake_pub.publish(brake_msg)
            self.get_logger().info('躯干控制已启用 - 使用左右摇杆控制躯干关节')
        elif not right_grip_pressed and self.last_right_grip:
            # 释放右边G键，禁用躯干控制
            self.torso_control_enabled = False
            self.get_logger().info('躯干控制已禁用')
        
        # 检测左边G键状态变化，控制底盘
        if left_grip_pressed and not self.last_left_grip:
            # 按下左边G键，启用底盘控制
            self.chassis_control_enabled = True
            # 重置底盘角度和速度为0，避免启用时立即移动
            self.current_chassis_angles = [0.0, 0.0, 0.0]
            self.current_chassis_velocities = [0.0, 0.0, 0.0]
            self.get_logger().info('底盘控制已启用 - 使用左右摇杆控制底盘')
        elif not left_grip_pressed and self.last_left_grip:
            # 释放左边G键，禁用底盘控制
            self.chassis_control_enabled = False
            # 重置底盘角度和速度为0
            self.current_chassis_angles = [0.0, 0.0, 0.0]
            self.current_chassis_velocities = [0.0, 0.0, 0.0]
            self.get_logger().info('底盘控制已禁用')
        
        # 更新上次按键状态
        self.last_right_grip = right_grip_pressed
        self.last_left_grip = left_grip_pressed
    
    def apply_dead_zone(self, value):
        """应用摇杆死区"""
        if abs(value) < self.dead_zone:
            return 0.0
        # 重新映射到 [dead_zone, 1.0] -> [0.0, 1.0]
        sign = 1.0 if value >= 0 else -1.0
        abs_value = abs(value)
        normalized = (abs_value - self.dead_zone) / (1.0 - self.dead_zone)
        return sign * normalized
    
    def timer_callback(self):
        """定时器回调，持续发布控制命令"""
        dt = 1.0 / self.publish_rate
        
        # 躯干控制
        if self.torso_control_enabled:
            # 应用死区并获取摇杆值
            left_x = self.apply_dead_zone(self.last_joystick_left[0])
            left_y = self.apply_dead_zone(self.last_joystick_left[1])
            right_y = self.apply_dead_zone(self.last_joystick_right[1])
            
            # 计算速度增量 (摇杆值 * 最大速度 * dt)
            # 3个关节：左摇杆X -> 关节1, 左摇杆Y -> 关节2, 右摇杆Y -> 关节3
            velocity_deltas = [
                -left_x * self.torso_max_speed * dt,   # 关节1: 左摇杆X
                left_y * self.torso_max_speed * dt,    # 关节2: 左摇杆Y
                right_y * self.torso_max_speed * dt,   # 关节3: 右摇杆Y
            ]
            
            # 更新位置（3个关节）
            new_positions = [
                self.current_positions[i] + velocity_deltas[i]
                for i in range(self.torso_joint_count)
            ]
            
            # 应用关节限制
            for i in range(self.torso_joint_count):
                new_positions[i] = np.clip(
                    new_positions[i],
                    self.joint_limits_min[i],
                    self.joint_limits_max[i]
                )
            
            # 更新当前位置
            self.current_positions = new_positions
            
            # 创建并发布躯干控制消息
            # 按照r1_lite_test_open_box.py的方式发送消息
            # p_des只有3个元素：[关节1位置, 关节2位置, 关节3位置]
            torso_msg = MotorControl()
            torso_msg.p_des = new_positions  # 3个元素：[关节1位置, 关节2位置, 关节3位置]
            torso_msg.v_des = self.torso_v_des  # 4个元素：[0.43, 0.7, 0.3, 0.3]
            torso_msg.kp = self.torso_kp  # 4个元素：[140.0, 200.0, 120.0, 20.0]
            torso_msg.kd = self.torso_kd  # 4个元素：[10.0, 50.0, 5.0, 1.0]
            torso_msg.t_ff = [0.0, 0.0, 0.0, 0.0]  # 4个元素
            
            self.torso_pub.publish(torso_msg)
        
        # 底盘控制
        if self.chassis_control_enabled:
            # 应用死区并获取摇杆值
            left_x = self.apply_dead_zone(self.last_joystick_left[0])
            left_y = self.apply_dead_zone(self.last_joystick_left[1])
            
            # 只有在摇杆有有效输入时才进行控制
            # 检查摇杆是否有有效输入（经过死区处理后不为0）
            has_input = abs(left_x) > 0.001 or abs(left_y) > 0.001
            
            if has_input:
                # 底盘控制映射：
                # 左摇杆Y: 前进/后退线速度（控制所有轮子的线速度）
                # 左摇杆X: 转向角度（控制前轮角度，实现差速转向）
                
                # 计算线速度（所有轮子使用相同的线速度）
                linear_velocity = left_y * self.chassis_max_linear_speed  # 前进/后退线速度 (m/s)
                
                # 计算转向角度（前轮转向，后轮保持0度）
                # 左摇杆X控制转向角度，负值表示左转，正值表示右转
                steering_angle = left_x * self.chassis_max_angle_speed * dt  # 角度增量 (rad)
                
                # 更新底盘角度
                # 左前轮和右前轮角度：根据转向输入计算（差速转向）
                # 后轮角度：保持0度（不转向）
                new_chassis_angles = list(self.current_chassis_angles)
                
                # 前轮转向角度更新（左前轮和右前轮同步转向）
                new_chassis_angles[0] += steering_angle  # 左前轮角度
                new_chassis_angles[1] += steering_angle  # 右前轮角度
                # 后轮角度保持0（不转向）
                new_chassis_angles[2] = 0.0
                
                # 应用角度限制
                for i in range(self.chassis_wheel_count):
                    new_chassis_angles[i] = np.clip(
                        new_chassis_angles[i],
                        self.chassis_angle_limits_min[i],
                        self.chassis_angle_limits_max[i]
                    )
                
                # 更新当前角度
                self.current_chassis_angles = new_chassis_angles
                
                # 计算线速度（所有轮子使用相同的线速度）
                # v_des: [左前轮线速度, 右前轮线速度, 后轮线速度]
                chassis_velocities = [linear_velocity, linear_velocity, linear_velocity]
                self.current_chassis_velocities = chassis_velocities
                
                # 创建并发布底盘控制消息
                # name, kp, kd, t_ff 标记为"-"，不需要发送（设置为空）
                chassis_msg = MotorControl()
                chassis_msg.p_des = new_chassis_angles  # [左前轮角度, 右前轮角度, 后轮角度]
                chassis_msg.v_des = chassis_velocities  # [左前轮线速度, 右前轮线速度, 后轮线速度]
                chassis_msg.name = ""  # 不需要发送，设置为空字符串
                chassis_msg.kp = []  # 不需要发送，设置为空数组
                chassis_msg.kd = []  # 不需要发送，设置为空数组
                chassis_msg.t_ff = []  # 不需要发送，设置为空数组
                
                self.chassis_pub.publish(chassis_msg)
            else:
                # 如果摇杆没有输入，发送零速度命令，保持当前角度
                chassis_msg = MotorControl()
                chassis_msg.p_des = self.current_chassis_angles  # 保持当前角度
                chassis_msg.v_des = [0.0, 0.0, 0.0]  # 零速度
                chassis_msg.name = ""  # 不需要发送，设置为空字符串
                chassis_msg.kp = []  # 不需要发送，设置为空数组
                chassis_msg.kd = []  # 不需要发送，设置为空数组
                chassis_msg.t_ff = []  # 不需要发送，设置为空数组
                
                self.chassis_pub.publish(chassis_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TorsoChassisVRControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

