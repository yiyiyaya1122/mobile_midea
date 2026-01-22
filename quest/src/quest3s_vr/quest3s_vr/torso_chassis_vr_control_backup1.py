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
        self.torso_speed_gain = 1.0   # 躯干速度增益（可调小以降低响应速度）
        self.chassis_speed_gain = 0.5  # 底盘速度增益（可调小以降低响应速度）

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
        self.max_torso_vx_forward = 0.2   # 正向最大速度
        self.max_torso_vx_backward = 0.1  # 负向最大速度（限制为0.1）
        # v_z: [-0.2, 0.2] m/s，但实际范围是 0.1 ≤ z ≤ 0.7（高度范围）
        self.max_torso_vz = 0.2           # 升降速度
        self.max_pitch = 0.5              # rad/s
        self.max_yaw = 0.5                # rad/s

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
        self.get_logger().info('  摇杆映射:')
        self.get_logger().info('    左摇杆X -> 关节1')
        self.get_logger().info('    左摇杆Y -> 关节2')
        self.get_logger().info('    右摇杆Y -> 关节3')
        self.get_logger().info('  零位复位: 同时按下左右手柄的T+G键')
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

        # 左 G：底盘
        if lg and not self.last_lg:
            self.chassis_enabled = True
            self.torso_enabled = False
            self.control_mode = ControlMode.CHASSIS
            self.get_logger().info('🚗 底盘控制模式')
            self.brake_pub.publish(Bool(data=False))

        elif not lg and self.last_lg:
            self.chassis_enabled = False
            self.control_mode = ControlMode.IDLE
            self.brake_pub.publish(Bool(data=True))

        # 右 G：躯干
        if rg and not self.last_rg:
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
        if self.control_mode == ControlMode.TORSO and self.torso_enabled:
            # v_x: -0.1 ≤ x ≤ 0.2 (负向限制为0.1，正向最大0.2)
            vx_raw = self.expo(lx)
            if vx_raw > 0:
                vx = vx_raw * self.max_torso_vx_forward * self.torso_speed_gain
            else:
                vx = vx_raw * self.max_torso_vx_backward * self.torso_speed_gain
            
            # v_z: 升降速度，范围 [-0.2, 0.2] m/s
            # 但高度范围是 0.1 ≤ z ≤ 0.7
            vz = self.expo(ly) * self.max_torso_vz * self.torso_speed_gain
            
            pitch = self.expo(ry) * self.max_pitch * self.torso_speed_gain
            yaw = self.expo(rx) * self.max_yaw * self.torso_speed_gain

            # 若底盘在动，禁止升降
            if abs(vx) > 0.2:
                vz = 0.0

            # Workspace 高度限制（0.1 ≤ z ≤ 0.7）
            self.torso_height_est += vz * dt
            self.torso_height_est = max(
                self.torso_height_min,  # 0.1
                min(self.torso_height_est, self.torso_height_max)  # 0.7
            )
            
            # 如果达到高度限制，停止升降
            if (self.torso_height_est <= self.torso_height_min and vz < 0) or \
               (self.torso_height_est >= self.torso_height_max and vz > 0):
                vz = 0.0

            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.twist.linear.x = vx
            msg.twist.linear.z = vz
            msg.twist.angular.y = pitch
            msg.twist.angular.z = yaw

            self.torso_pub.publish(msg)
        else:
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
        # 明确设置所有字段为0
        msg.twist.linear.x = 0.0
        msg.twist.linear.z = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0
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
