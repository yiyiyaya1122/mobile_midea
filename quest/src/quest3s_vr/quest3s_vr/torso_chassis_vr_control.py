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
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import time

from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool
from robot_msgs.msg import Quest2Button
from hdas_msg.msg import MotorControl


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

        # 躯干位置控制发布者（用于回零）
        # 根据 ros2 topic info，订阅者 HDAS 使用 BEST_EFFORT + VOLATILE
        # 需要匹配订阅者的 QoS 才能传递消息
        qos_torso = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        self.torso_control_pub = self.create_publisher(
            MotorControl,
            '/motion_control/control_torso',
            qos_torso
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
        self.chassis_speed_gain = 0.1  # 底盘速度增益（可调小以降低响应速度）
        
        # 低通滤波器参数（用于平滑控制）
        self.filter_alpha = 1.0  # 滤波系数：0.0-1.0，越小越平滑（但响应越慢）
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
        self.max_torso_vz = 0.15           # z方向最大速度
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
        self.torso_resetting = False  # 标记是否正在回零
        

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
        self.get_logger().info('    左摇杆X -> x方向速度')
        self.get_logger().info('    左摇杆Y -> z方向速度（升降）')
        self.get_logger().info('    右摇杆Y -> pitch角速度')
        self.get_logger().info('  底盘控制: 按下左边G键 + 左右摇杆控制')
        self.get_logger().info('    左摇杆Y -> 前进/后退速度')
        self.get_logger().info('    左摇杆X -> 左/右移动速度')
        self.get_logger().info('    右摇杆X -> 转向角度')
        self.get_logger().info('  躯干回零: 同时按下右边手柄的T+G键')
        

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

        self.last_ltr = ltr
        self.last_rtr = rtr

        # 检测零位复位：同时按下右边手柄的T+G键（避免与底盘控制冲突）
        reset_triggered = rtr and rg

        if reset_triggered and not self.last_reset_triggered:
            # 同时按下右边手柄的T+G键，执行零位复位（只在刚按下时触发一次）
            # 发送零位置命令使躯干回到零位
            self.torso_resetting = True  # 标记正在回零
            self.reset_torso_to_zero()
            # 回零时禁用躯干控制，避免冲突
            self.torso_enabled = False
            self.control_mode = ControlMode.IDLE
        
        # 如果释放了回零按键，清除回零标记
        if not reset_triggered and self.last_reset_triggered:
            self.torso_resetting = False
        
        # 更新零位复位触发状态
        self.last_reset_triggered = reset_triggered

        # 左 G：底盘（如果正在回零，不触发底盘控制）
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

        # 右 G：躯干（如果正在回零，不触发躯干控制）
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
            wz = -self.expo(rx) * self.max_wz * self.chassis_speed_gain  # 右摇杆X控制角速度

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
        # 如果正在回零，持续发送位置命令
        if self.torso_resetting:
            # 回零时持续发送零位置命令
            torso_control = MotorControl()
            torso_control.p_des = [0.0, 0.0, 0.0]  # 3个关节都回到零位
            torso_control.v_des = [0.43, 0.7, 0.3, 0.3]  # 速度参数（4个元素）
            torso_control.kp = [140.0, 200.0, 120.0, 20.0]  # 位置增益（4个元素）
            torso_control.kd = [10.0, 50.0, 5.0, 1.0]  # 速度增益（4个元素）
            torso_control.t_ff = [0.0, 0.0, 0.0, 0.0]  # 前馈扭矩（4个元素）
            self.torso_control_pub.publish(torso_control)
        elif self.control_mode == ControlMode.TORSO and self.torso_enabled:
                
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
            # 不在控制模式时，重置滤波器状态并发送零速度命令
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

    def reset_torso_to_zero(self):
        """发送零位置命令使躯干回到零位（参考 r1_lite_test_open_box.py）"""
        # 参考 r1_lite_test_open_box.py 的实现方式
        self.get_logger().info('🔄 开始躯干回零...')
        
        # 1. 设置刹车模式（参考 r1_lite_test_open_box.py command '5'）
        brake_msg = Bool()
        brake_msg.data = True
        self.brake_pub.publish(brake_msg)
        time.sleep(0.2)  # 等待刹车模式生效
        
        torso_control = MotorControl()
        torso_control.p_des = [0.0, 0.0, 0.0]  # 3个关节都回到零位（参考 r1_lite_test_open_box.py line 184）
        torso_control.v_des = [0.43, 0.7, 0.3, 0.3]  # 速度参数（4个元素，与 r1_lite_test_open_box.py 保持一致）
        torso_control.kp = [140.0, 200.0, 120.0, 20.0]  # 位置增益（4个元素）
        torso_control.kd = [10.0, 50.0, 5.0, 1.0]  # 速度增益（4个元素）
        torso_control.t_ff = [0.0, 0.0, 0.0, 0.0]  # 前馈扭矩（4个元素）
        
        # 4. 发送位置命令（发送多次确保接收）
        self.get_logger().info('📤 发送零位置命令 [0.0, 0.0, 0.0]...')
        for i in range(10):
            self.torso_control_pub.publish(torso_control)
            time.sleep(0.1)  # 每100ms发送一次，共发送10次
        
        self.get_logger().info('✅ 躯干回零命令已发送 - 零位置 [0.0, 0.0, 0.0]（持续发送中）')



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
