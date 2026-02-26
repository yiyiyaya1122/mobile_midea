#!/usr/bin/env python3
"""
R1 Lite YAM 手臂开箱测试。

依赖：robot_msgs (YamCmd, YamStatus)。无需 hdas_msg。
  source /opt/ros/<distro>/setup.bash
  source <含 robot_msgs 的 install>/setup.bash
  python3 r1_lite_yam_arm_test.py

菜单：1=手臂回零  2=手臂小动  3=夹爪张开  4=夹爪闭合  q=退出
"""

import sys
import threading
import time

import numpy as np
import rclpy
from robot_msgs.msg import YamCmd, YamStatus
from std_msgs.msg import Bool
from rclpy.node import Node

MODE_JOINT = 5
GRIPPER_CLOSED = 0.0
GRIPPER_OPEN = 5.0
HOME_JOINT = np.array([0.0] * 6)
# 回零：开环插值，不依赖 status，固定时长后恢复控制；单步关节变化不超过此值
RAMP_STEP = np.deg2rad(360) / 50.0
HOMING_PERIOD_S = 0.02
HOMING_DURATION_S = 2.0  # 回零总时长，到点后自动结束
# 手臂小动：相对零位的偏移 (rad)，左臂 + 右臂 -，目标姿势更大
ARM_SMALL_OFFSET = np.array([0.9, 0.5, 0.2, 0.0, 0.0, 0.0])
ARM_SMALL_STEPS = 20  # 分多步发送，每步 joint delta 更小，避免超限
ARM_SMALL_STEP_DT = 0.05  # 每步间隔 (s)

# 命令发布：YamController 订阅端为默认 VOLATILE，必须一致否则收不到
QOS_CMD = rclpy.qos.QoSProfile(
    reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
    history=rclpy.qos.HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=rclpy.qos.DurabilityPolicy.VOLATILE,
)
QOS_STATUS = rclpy.qos.QoSProfile(
    reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
    history=rclpy.qos.HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=rclpy.qos.DurabilityPolicy.VOLATILE,
)
QOS = rclpy.qos.QoSProfile(
    reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
    history=rclpy.qos.HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
)


def print_menu():
    print("\n1: 手臂回零 (arm homing)")
    print("2: 手臂小动 (arm small motion)")
    print("3: 夹爪张开 (gripper open)")
    print("4: 夹爪闭合 (gripper closed)")
    print("q: 退出 (quit)")
    print("---")


_print_lock = threading.Lock()


class YamArmTestNode(Node):
    def __init__(self):
        super().__init__("yam_arm_test")
        self.left_cmd_pub = self.create_publisher(YamCmd, "/YAM_VR_L", QOS_CMD)
        self.right_cmd_pub = self.create_publisher(YamCmd, "/YAM_VR_R", QOS_CMD)
        self.breaking_mode_pub = self.create_publisher(Bool, "/motion_target/brake_mode", QOS)
        self._last_left = list(HOME_JOINT)
        self._last_right = list(HOME_JOINT)
        self._left_status = None
        self._right_status = None
        self._received_left = False
        self._received_right = False
        self._status_lock = threading.Lock()
        self._command = None
        self._cmd_lock = threading.Lock()
        self.create_subscription(YamStatus, "/arm_l_status", self._cb_left_status, QOS_STATUS)
        self.create_subscription(YamStatus, "/arm_r_status", self._cb_right_status, QOS_STATUS)

    def _cb_left_status(self, msg):
        with self._status_lock:
            self._left_status = msg
            self._received_left = True

    def _cb_right_status(self, msg):
        with self._status_lock:
            self._right_status = msg
            self._received_right = True

    def _get_current_joints(self):
        with self._status_lock:
            left = list(self._left_status.joint_pos[:6]) if self._left_status else list(self._last_left)
            right = list(self._right_status.joint_pos[:6]) if self._right_status else list(self._last_right)
        return np.array(left), np.array(right)

    def _make_yam_cmd(self, joint_6, gripper=0.0):
        msg = YamCmd()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.mode = MODE_JOINT
        msg.joint_pos = list(joint_6)[:6]
        msg.gripper = max(GRIPPER_CLOSED, min(GRIPPER_OPEN, float(gripper)))
        msg.end_pose = [0.0] * 7
        return msg

    def _publish_arm_cmd(self, left_6, right_6, gripper_left=0.0, gripper_right=0.0):
        self._last_left = list(left_6)[:6]
        self._last_right = list(right_6)[:6]
        self.left_cmd_pub.publish(self._make_yam_cmd(self._last_left, gripper_left))
        self.right_cmd_pub.publish(self._make_yam_cmd(self._last_right, gripper_right))

    def _publish_arm_cmd_repeat(self, left_6, right_6, gripper_left=0.0, gripper_right=0.0, times=5, dt=0.02):
        """连续发多次，确保控制器收到（部分控制器需周期性指令）。"""
        for _ in range(times):
            self._publish_arm_cmd(left_6, right_6, gripper_left, gripper_right)
            time.sleep(dt)

    def run_arm_homing(self):
        """开环回零：从当前 _last 插值到零位，固定时长，每步限制 joint delta。可按键中止。"""
        start_left = np.array(self._last_left)
        start_right = np.array(self._last_right)
        n = max(1, int(HOMING_DURATION_S / HOMING_PERIOD_S))
        t0 = time.monotonic()
        for step in range(1, n + 1):
            if not rclpy.ok():
                return
            cmd = self.take_command()
            if cmd is not None:
                self.set_command(cmd)
                self.get_logger().info("Homing aborted.")
                return
            t = step / n
            # 线性插值到零位，夹爪最后一段再闭合
            left = (start_left * (1 - t) + HOME_JOINT * t).tolist()
            right = (start_right * (1 - t) + HOME_JOINT * t).tolist()
            gripper = GRIPPER_OPEN if t < 0.9 else GRIPPER_CLOSED
            for _ in range(3):
                self._publish_arm_cmd(left, right, gripper, gripper)
                time.sleep(0.01)
            time.sleep(HOMING_PERIOD_S)
        for _ in range(10):
            self._publish_arm_cmd(HOME_JOINT, HOME_JOINT, GRIPPER_CLOSED, GRIPPER_CLOSED)
            time.sleep(HOMING_PERIOD_S)
        self.get_logger().info("Arm homing done.")

    def run_arm_small_motion(self):
        n = ARM_SMALL_STEPS
        dt = ARM_SMALL_STEP_DT
        # 分步到目标姿势，控制每步 joint delta；保持 2s，不回零
        for i in range(1, n + 1):
            t = i / n
            left = (HOME_JOINT + ARM_SMALL_OFFSET * t).tolist()
            right = (HOME_JOINT - ARM_SMALL_OFFSET * t).tolist()
            self._publish_arm_cmd(left, right, GRIPPER_CLOSED, GRIPPER_CLOSED)
            time.sleep(dt)
        

    def run_gripper_open(self):
        self._publish_arm_cmd(self._last_left, self._last_right, GRIPPER_OPEN, GRIPPER_OPEN)
        time.sleep(0.3)

    def run_gripper_closed(self):
        self._publish_arm_cmd(self._last_left, self._last_right, GRIPPER_CLOSED, GRIPPER_CLOSED)
        time.sleep(0.3)

    def send_brake(self, enable):
        msg = Bool()
        msg.data = bool(enable)
        self.breaking_mode_pub.publish(msg)

    def set_command(self, cmd):
        with self._cmd_lock:
            self._command = cmd

    def take_command(self):
        with self._cmd_lock:
            c = self._command
            self._command = None
            return c

    def _input_thread_fn(self):
        while rclpy.ok():
            try:
                with _print_lock:
                    line = input().strip().lower()
            except (EOFError, KeyboardInterrupt):
                self.set_command("q")
                break
            if line:
                self.set_command(line)

    def run(self):
        with _print_lock:
            print_menu()
            print("输入 1-4 或 q: ", end="", flush=True)
        threading.Thread(target=rclpy.spin, args=(self,), daemon=True).start()
        threading.Thread(target=self._input_thread_fn, daemon=True).start()
        rate = self.create_rate(20)
        try:
            while rclpy.ok():
                cmd = self.take_command()
                if not cmd:
                    rate.sleep()
                    continue
                self.send_brake(True)
                if cmd == "1":
                    self.run_arm_homing()
                elif cmd == "2":
                    self.run_arm_small_motion()
                elif cmd == "3":
                    self.run_gripper_open()
                elif cmd == "4":
                    self.run_gripper_closed()
                elif cmd == "q":
                    self.send_brake(False)
                    self.get_logger().info("Quit.")
                    break
                else:
                    self.get_logger().warn(f"Unknown command: {cmd}")
                with _print_lock:
                    print_menu()
                    print("输入 1-4 或 q: ", end="", flush=True)
                rate.sleep()
        except KeyboardInterrupt:
            self.send_brake(False)
        finally:
            with _print_lock:
                print("Shutting down.")


def main(args=None):
    rclpy.init(args=args)
    node = YamArmTestNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
