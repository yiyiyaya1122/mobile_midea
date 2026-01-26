#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
VR 遥操作节点（底盘 + 躯干）
- 使用 motion_target 接口（TwistStamped）
- 实现底盘 / 躯干互锁
- Workspace 安全限制（防翻 / 防撞）
"""

import rclpy
from rclpy.node import Node
from enum import Enum

from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool
from robot_msgs.msg import Quest2Button


# =========================
# 控制模式定义
# =========================
class ControlMode(Enum):
    IDLE = 0
    CHASSIS = 1 
    TORSO = 2


# =========================
# 主节点
# =========================
class VRChassisTorsoControl(Node):

    def __init__(self):
        super().__init__('vr_chassis_torso_control')

        # -------------------------
        # Publisher
        # -------------------------
        self.chassis_pub = self.create_publisher(
            TwistStamped,
            '/motion_target/target_speed_chassis',
            10
        )

        self.chassis_acc_limit_pub = self.create_publisher(
            TwistStamped,
            '/motion_target/chassis_acc_limit',
            10
        )

        self.torso_pub = self.create_publisher(
            TwistStamped,
            '/motion_target/target_speed_torso',
            10
        )

        self.brake_pub = self.create_publisher(
            Bool,
            '/motion_target/brake_mode',
            10
        )

        # -------------------------
        # Subscriber
        # -------------------------
        self.create_subscription(
            Quest2Button,
            '/vr_button',
            self.button_callback,
            10
        )

        # -------------------------
        # 参数配置
        # -------------------------
        self.dead_zone = 0.1
        self.publish_rate = 50.0

        # 速度增益（用于缩放摇杆输入）
        self.torso_speed_gain = 0.5   # 躯干速度增益（降低以提高平滑度）
        self.chassis_speed_gain = 0.2  # 底盘速度增益（可调小以降低响应速度）
        
        # 低通滤波器参数（用于平滑控制）
        self.filter_alpha = 0.3  # 滤波系数：0.0-1.0，越小越平滑（但响应越慢）
        self.filtered_vx = 0.0
        self.filtered_vz = 0.0
        self.filtered_pitch = 0.0

        # 底盘速度限制
        self.max_vx = 1.0      # m/s (前进/后退最大速度)
        self.max_vy = 1.0      # m/s (侧向最大速度)
        self.max_wz = 2.0      # rad/s (角速度)

        # 底盘加速度限制（按接口要求）
        self.chassis_acc_limit_vx = 2.5  # m/s²
        self.chassis_acc_limit_vy = 1.0  # m/s²
        self.chassis_acc_limit_wz = 1.0  # rad/s²

        # 躯干速度限制（严格按接口要求）
        # v_x: [-0.2, 0.2] m/s，但实际范围是 -0.1 ≤ x ≤ 0.2
        self.max_torso_vx = 0.1   # x方向最大速度
        # v_z: [-0.2, 0.2] m/s，但实际范围是 0.1 ≤ z ≤ 0.7（高度范围）
        self.max_torso_vz = 0.1           # z方向最大速度
        self.max_pitch = 0.1              # pitch方向最大速度

        # Workspace 估计
        self.torso_height_est = 0.3   # 初始高度
        self.torso_height_min = 0.1
        self.torso_height_max = 0.7
        self.torso_height_slow = 0.6  # 高位阈值

        # -------------------------
        # 状态变量
        # -------------------------
        self.control_mode = ControlMode.IDLE

        self.chassis_enabled = False
        self.torso_enabled = False

        self.last_left_js = [0.0, 0.0]
        self.last_right_js = [0.0, 0.0]

        self.last_lg = False
        self.last_rg = False
        self.last_ltr = False
        self.last_rtr = False
        self.last_reset_triggered = False  # 防止零位复位重复触发
        self.torso_reset_active = False  # 复位激活标志
        
        # 当前按钮状态（用于timer_callback中检查）
        self.current_rtr = False
        self.current_rg = False
        

        # -------------------------
        # Timer
        # -------------------------
        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.timer_callback
        )

        # 发布底盘加速度限制（初始化时发布一次）
        self.publish_chassis_acc_limit()

        self.get_logger().info('✅ VR 底盘 + 躯干遥操作节点已启动')
        self.get_logger().info('控制方式:')
        self.get_logger().info('  躯干控制: 按下右边G键 + 左右摇杆控制')
        self.get_logger().info('    左摇杆X -> 关节1')
        self.get_logger().info('    左摇杆Y -> 关节2')
        self.get_logger().info('    右摇杆Y -> 关节3')
        self.get_logger().info('  躯干复位: 按下右边手柄的T+G键')
        self.get_logger().info('  底盘控制: 按下左边G键 + 左右摇杆控制')
        self.get_logger().info('    左摇杆Y -> 前进/后退速度')
        self.get_logger().info('    左摇杆X -> 左/右移动速度')
        self.get_logger().info('    右摇杆X -> 转向角度')
        

    # =========================
    # 工具函数
    # =========================
    def apply_dead_zone(self, v):
        if abs(v) < self.dead_zone:
            return 0.0
        return (abs(v) - self.dead_zone) / (1.0 - self.dead_zone) * (1 if v > 0 else -1)

    def expo(self, x, k=0.4):
        return x * abs(x) * k + x * (1.0 - k)

    # =========================
    # VR 输入回调
    # =========================
    def button_callback(self, msg: Quest2Button):
        self.last_left_js = list(msg.left_joystick)
        self.last_right_js = list(msg.right_joystick)

        lg = msg.lg
        rg = msg.rg
        ltr = msg.ltr  # 左trigger
        rtr = msg.rtr  # 右trigger

        # 更新当前按钮状态（供timer_callback使用）
        self.current_rtr = rtr
        self.current_rg = rg

        # 检测零位复位：按下右边手柄的T+G键
        # 注意：复位功能不受互锁影响，可以在任何模式下触发
        right_tg_pressed = rtr and rg
        reset_triggered = right_tg_pressed

        # 优先处理复位：复位优先级最高
        if reset_triggered:
            if not self.last_reset_triggered:
                # 刚按下T+G键，激活复位
                self.torso_reset_active = True
                # 复位时禁用其他控制
                self.torso_enabled = False
                self.chassis_enabled = False
                self.control_mode = ControlMode.IDLE
                self.get_logger().info(f'🔄 零位复位已激活 - rtr={rtr}, rg={rg}')
        else:
            if self.last_reset_triggered:
                # 刚释放T+G键，停止复位
                self.torso_reset_active = False
                self.get_logger().info('🔄 零位复位已停止')
        
        # 更新零位复位触发状态
        self.last_reset_triggered = reset_triggered
        self.last_ltr = ltr
        self.last_rtr = rtr

        # 左 G：底盘（复位时不触发，避免冲突）
        if lg and not self.last_lg and not reset_triggered:
            self.chassis_enabled = True
            self.torso_enabled = False
            self.control_mode = ControlMode.CHASSIS
            self.get_logger().info('🚗 底盘控制模式')
            self.brake_pub.publish(Bool(data=False))

        elif not lg and self.last_lg:
            self.chassis_enabled = False
            self.control_mode = ControlMode.IDLE
            self.brake_pub.publish(Bool(data=True))

        # 右 G：躯干（复位时不触发，避免冲突）
        if rg and not self.last_rg and not reset_triggered:
            self.torso_enabled = True
            self.chassis_enabled = False
            self.control_mode = ControlMode.TORSO
            self.get_logger().info('🧍 躯干控制模式')

        elif not rg and self.last_rg:
            self.torso_enabled = False
            self.control_mode = ControlMode.IDLE

        self.last_lg = lg
        self.last_rg = rg

    # =========================
    # 主循环
    # =========================
    def timer_callback(self):
        dt = 1.0 / self.publish_rate

        lx = self.apply_dead_zone(self.last_left_js[0])
        ly = self.apply_dead_zone(self.last_left_js[1])
        rx = self.apply_dead_zone(self.last_right_js[0])
        ry = self.apply_dead_zone(self.last_right_js[1])

        # ======================
        # 底盘控制（互锁）
        # ======================
        if self.control_mode == ControlMode.CHASSIS and self.chassis_enabled:
            # 摇杆映射：
            # 左摇杆X -> linear.y (侧向速度，方向已反转)
            # 左摇杆Y -> linear.x (前进/后退速度，方向已反转)
            # 右摇杆 -> angular.z (角速度)
            vx = -self.expo(ly) * self.max_vx * self.chassis_speed_gain  # 左摇杆Y控制前进/后退（方向反转）
            vy = -self.expo(lx) * self.max_vy * self.chassis_speed_gain  # 左摇杆X控制侧向速度（方向反转）
            wz = self.expo(rx) * self.max_wz * self.chassis_speed_gain  # 右摇杆X控制角速度

            # 高位限速（防翻）
            if self.torso_height_est > self.torso_height_slow:
                vx *= 0.3
                vy *= 0.3
                wz *= 0.3

            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.twist.linear.x = vx
            msg.twist.linear.y = vy
            msg.twist.angular.z = wz

            self.chassis_pub.publish(msg)
        else:
            self.publish_zero_chassis()

        # ======================
        # 躯干控制（互锁）
        # ======================
        # 优先处理复位：在timer_callback中直接检查复位状态（优先级最高）
        # 这样可以确保即使button_callback延迟，复位也能正常工作
        reset_active = self.current_rtr and self.current_rg
        
        if reset_active:
            # 复位时发送零速度命令到 /motion_target/target_speed_torso
            self.torso_reset_active = True
            # 复位时禁用其他控制
            self.torso_enabled = False
            self.chassis_enabled = False
            self.control_mode = ControlMode.IDLE
            # 发送零速度命令（复位）
            self.publish_zero_torso()
            # 调试信息（每1秒打印一次，避免日志过多）
            if not hasattr(self, '_last_reset_log_time'):
                self._last_reset_log_time = 0
            current_time = self.get_clock().now().nanoseconds
            if (current_time - self._last_reset_log_time) > 1e9:  # 1秒
                self.get_logger().info('🔄 复位中...发送零速度命令到 target_speed_torso')
                self._last_reset_log_time = current_time
        else:
            # 释放复位键时，清除复位标志
            if self.torso_reset_active:
                self.torso_reset_active = False
                if hasattr(self, '_last_reset_log_time'):
                    self._last_reset_log_time = 0
            
            # 正常躯干控制
            if self.control_mode == ControlMode.TORSO and self.torso_enabled:
                
                # v_x: x方向速度，范围 [-0.2, 0.2] m/s
                vx_target = self.expo(lx) * self.max_torso_vx * self.torso_speed_gain
                
                # v_z: z方向速度，范围 [-0.2, 0.2] m/s
                vz_target = -self.expo(ly) * self.max_torso_vz * self.torso_speed_gain
                
                # pitch: 俯仰角速度
                pitch_target = -self.expo(ry) * self.max_pitch * self.torso_speed_gain

                # 应用低通滤波器平滑速度命令
                self.filtered_vx = self.filter_alpha * vx_target + (1.0 - self.filter_alpha) * self.filtered_vx
                self.filtered_vz = self.filter_alpha * vz_target + (1.0 - self.filter_alpha) * self.filtered_vz
                self.filtered_pitch = self.filter_alpha * pitch_target + (1.0 - self.filter_alpha) * self.filtered_pitch

                # 若底盘在动，禁止升降
                # if abs(self.filtered_vx) > 0.2:
                #     self.filtered_vz = 0.0

                # # Workspace 高度限制（0.1 ≤ z ≤ 0.7）
                # self.torso_height_est += self.filtered_vz * dt
                # self.torso_height_est = max(
                #     self.torso_height_min,  # 0.1
                #     min(self.torso_height_est, self.torso_height_max)  # 0.7
                # )
                
                # # 如果达到高度限制，停止升降
                # if (self.torso_height_est <= self.torso_height_min and self.filtered_vz < 0) or \
                #    (self.torso_height_est >= self.torso_height_max and self.filtered_vz > 0):
                #     self.filtered_vz = 0.0

                msg = TwistStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'base_link'
                msg.twist.linear.x = self.filtered_vx
                msg.twist.linear.z = self.filtered_vz
                msg.twist.angular.y = self.filtered_pitch

                self.torso_pub.publish(msg)
            else:
                # 不在控制模式时，重置滤波器状态
                # 注意：如果正在复位，不要发送零速度命令，避免与位置命令冲突
                if not self.torso_reset_active:
                    self.filtered_vx = 0.0
                    self.filtered_vz = 0.0
                    self.filtered_pitch = 0.0
                    self.publish_zero_torso()

    # =========================
    # Zero Command
    # =========================
    def publish_zero_chassis(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        # 明确设置所有字段为0
        msg.twist.linear.x = 0.0
        msg.twist.linear.y = 0.0
        msg.twist.angular.z = 0.0
        self.chassis_pub.publish(msg)

    def publish_zero_torso(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        # 明确设置torso控制字段为0（只有x、z、pitch，没有yaw）
        msg.twist.linear.x = 0.0
        msg.twist.linear.z = 0.0
        msg.twist.angular.y = 0.0
        # 不设置angular.z (yaw)，因为torso控制没有yaw分量
        self.torso_pub.publish(msg)

    def publish_chassis_acc_limit(self):
        """发布底盘加速度限制"""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = self.chassis_acc_limit_vx  # 2.5 m/s²
        msg.twist.linear.y = self.chassis_acc_limit_vy  # 1.0 m/s²
        msg.twist.angular.z = self.chassis_acc_limit_wz  # 1.0 rad/s²
        self.chassis_acc_limit_pub.publish(msg)
        self.get_logger().info(f'📋 底盘加速度限制已发布: vx={self.chassis_acc_limit_vx}, vy={self.chassis_acc_limit_vy}, wz={self.chassis_acc_limit_wz}')



# =========================
# main
# =========================
def main(args=None):
    rclpy.init(args=args)
    node = VRChassisTorsoControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
