#!/usr/bin/env python3
"""
R1 Lite 躯干开箱测试。

依赖：hdas_msg (MotorControl)。无需 robot_msgs。
  source /opt/ros/<distro>/setup.bash
  source ~/galaxea/install/setup.bash
  python3 r1_lite_torso_test.py

菜单：1=躯干跪姿  2=躯干回零  q=退出
"""

import sys
import threading
import time

import rclpy
from hdas_msg.msg import MotorControl
from std_msgs.msg import Bool
from rclpy.node import Node

TORSO_ZERO = [0.0, 0.0, 0.0, 0.0]
TORSO_KNEE = [-1.39, 2.53, 1.14, 0.0]

QOS = rclpy.qos.QoSProfile(
    reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
    history=rclpy.qos.HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
)


def print_menu():
    print("\n1: 躯干跪姿 (torso knees)")
    print("2: 躯干回零 (torso zero)")
    print("q: 退出 (quit)")
    print("---")


# 统一打印顺序，避免主线程与输入线程交叉输出
_print_lock = threading.Lock()


class TorsoTestNode(Node):
    def __init__(self):
        super().__init__("torso_test")
        self.torso_control_pub = self.create_publisher(MotorControl, "/motion_control/control_torso", QOS)
        self.breaking_mode_pub = self.create_publisher(Bool, "/motion_target/brake_mode", QOS)
        self._command = None
        self._cmd_lock = threading.Lock()

    def _publish_torso(self, position_torso):
        p = list(position_torso)
        if len(p) < 4:
            p.extend([0.0] * (4 - len(p)))
        msg = MotorControl()
        msg.p_des = [float(x) for x in p[:4]]
        msg.v_des = [0.43, 0.7, 0.3, 0.3]
        msg.kp = [140.0, 200.0, 120.0, 20.0]
        msg.kd = [10.0, 50.0, 5.0, 1.0]
        msg.t_ff = [0.0, 0.0, 0.0, 0.0]
        self.torso_control_pub.publish(msg)

    def run_torso_knees(self):
        self._publish_torso(TORSO_KNEE)
        time.sleep(1.0)

    def run_torso_zero(self):
        self._publish_torso(TORSO_ZERO)
        time.sleep(1.0)

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
            print("输入 1、2 或 q: ", end="", flush=True)
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
                    self.run_torso_knees()
                elif cmd == "2":
                    self.run_torso_zero()
                elif cmd == "q":
                    self.send_brake(False)
                    self.get_logger().info("Quit.")
                    break
                else:
                    self.get_logger().warn(f"Unknown command: {cmd}")
                with _print_lock:
                    print_menu()
                    print("输入 1、2 或 q: ", end="", flush=True)
                rate.sleep()
        except KeyboardInterrupt:
            self.send_brake(False)
        finally:
            with _print_lock:
                print("Shutting down.")


def main(args=None):
    rclpy.init(args=args)
    node = TorsoTestNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
