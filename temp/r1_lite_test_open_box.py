#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import sys
import threading
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from hdas_msg.msg import MotorControl
import time
from std_msgs.msg import Bool
from rclpy.duration import Duration

qos_profile = rclpy.qos.QoSProfile(
    reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,  
    history=rclpy.qos.HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
)
class JointStateSender(Node):
    def __init__(self):
        super().__init__('joint_state_sender')
        self.left_joint_state_pub = self.create_publisher(JointState, '/motion_target/target_joint_state_arm_left', qos_profile)
        self.right_joint_state_pub = self.create_publisher(JointState, '/motion_target/target_joint_state_arm_right', qos_profile)
        self.torso_control_pub = self.create_publisher(MotorControl, '/motion_control/control_torso', qos_profile)
        
        self.left_gripper_joint_state_pub = self.create_publisher(JointState, '/motion_target/target_position_gripper_left', qos_profile)
        self.right_gripper_joint_state_pub = self.create_publisher(JointState, '/motion_target/target_position_gripper_right', qos_profile)
       
        self.chassis_command_pub = self.create_publisher(Twist, '/motion_target/target_speed_chassis', qos_profile)
        self.acc_limit_pub = self.create_publisher(Twist, '/motion_target/chassis_acc_limit', qos_profile)
        self.breaking_mode_pub = self.create_publisher(Bool, '/motion_target/brake_mode', qos_profile)
        self.command = None
        self.feedback_running = False



    def send_joint_state_real(self, position_left, position_right, position_torso):
        left_joint_state = JointState()
        left_joint_state.position = position_left
        right_joint_state = JointState()
        right_joint_state.position = position_right
        torso_control = MotorControl()
        torso_control.p_des = position_torso
        torso_control.v_des = [0.43,0.7,0.3,0.3]
        torso_control.kp = [140.0,200.0,120.0,20.0]
        torso_control.kd = [10.0,50.0,5.0,1.0]
        torso_control.t_ff = [0.0,0.0,0.0,0.0]

        self.left_joint_state_pub.publish(left_joint_state)
        self.right_joint_state_pub.publish(right_joint_state)
        self.torso_control_pub.publish(torso_control)

    def send_gripper_joint_state(self, position_left, position_right):
        left_joint_state = JointState()
        left_joint_state.position = position_left
        right_joint_state = JointState()
        right_joint_state.position = position_right

        self.left_gripper_joint_state_pub.publish(left_joint_state)
        self.right_gripper_joint_state_pub.publish(right_joint_state)

    def send_vel_limit(self, vel_limit_left, vel_limit_right):
        left_joint_state = JointState()
        left_joint_state.velocity = vel_limit_left
        right_joint_state = JointState()
        right_joint_state.velocity = vel_limit_right

        self.left_joint_state_pub.publish(left_joint_state)
        self.right_joint_state_pub.publish(right_joint_state)

    def send_chassis(self, chassis_command):
        chassis_command_msg = Twist()
        chassis_command_msg.linear.x = chassis_command[0]
        chassis_command_msg.linear.y = chassis_command[1]
        chassis_command_msg.angular.z = chassis_command[2]
        self.chassis_command_pub.publish(chassis_command_msg)
    def send_acc_limit(self,acc_limit):
        acc_limit_msg = Twist()
        acc_limit_msg.linear.x = acc_limit[0]
        acc_limit_msg.linear.y = acc_limit[1]
        acc_limit_msg.angular.z = acc_limit[2]
        self.acc_limit_pub.publish(acc_limit_msg)
    def send_breaking_mode(self,breaking_mode_signal):
        breaking_mode_msg = Bool()
        breaking_mode_msg.data = bool(breaking_mode_signal[0])
        self.breaking_mode_pub.publish(breaking_mode_msg)


    # def start_fake_feedback_blocking(self):
    #     joint_state = JointState()
    #     joint_state.position = [0.0] * 7

    #     start_time = self.get_clock().now().nanoseconds
    #     duration_ns = int(5 * 1e9)

    #     rate = self.create_rate(10)  # 10Hz
    #     while self.get_clock().now().nanoseconds - start_time < duration_ns:
    #         joint_state.header.stamp = self.get_clock().now().to_msg()
    #         self.fake_pub_left.publish(joint_state)
    #         self.fake_pub_right.publish(joint_state)
    #         rate.sleep()

    #     self.get_logger().info("Fake feedback publishing finished.")

    def input_thread(self):
        while True:
            print('1: arm_test1')
            print('2: arm_test2')
            print('3: arm_test3')
            print('4: torso_test1')
            print('5: torso_test2')
            print('6: gripper_test1')
            print('7: gripper_test2')
            print('q: quit brake mode')
            self.command = input("Press '1' or '2' or '3' or '4' or '5' or '6' or '7' or 'q': ")

    def run(self):
        threading.Thread(target=rclpy.spin, args=(self,), daemon=True).start()

        # while True:
        #     print("\033[33m双臂是否已安装？输入 1 表示已安装，0 表示未安装：\033[0m")
        #     choice = input("\033[33mHas the dual-arm assembly been completed? Enter 1 for Yes, 0 for No:\033[0m ")
        #     if choice == '1':
        #         break
        #     elif choice == '0':
        #         print("Publishing simulated feedback to /hdas/feedback_arm_left and /hdas/feedback_arm_right to complete initialization...")
        #         self.start_fake_feedback_blocking()  # <-- 主线程阻塞执行
        #         break
        #     else:
        #         print("Invalid input. Please enter 1 or 0.")
        input_thread = threading.Thread(target=self.input_thread)
        input_thread.daemon = True
        input_thread.start()
        try:

            while rclpy.ok():
                try:
                    command = self.command
                    self.command = None
                    if command == '1': # hand up test, end up with standing
                        # 按下2时发送第二个JointState
                        self.send_vel_limit([1.6,1.6,1.6,4.0,4.0,4.0],[1.6,1.6,1.6,4.0,4.0,4.0])
                        self.send_breaking_mode([1])
                        self.send_joint_state_real([0.0, 0.0, -0.6, 0.0, 0.0, 0.0],[0.0, 0.0, -0.6, 0.0, 0.0, 0.0],[0.0,0.0,0.0,0.0])
                        time.sleep(0.5)  # 防止重复发送
                        self.send_joint_state_real([0.0, 0.0, 0.0, 0.0, 0.0, 0.0],[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],[0.0,0.0,0.0,0.0])
                        time.sleep(0.5)  # 防止重复发送
                    elif command == '2': # kongfu standing
                        self.send_breaking_mode([1])
                        self.send_vel_limit([1.6,1.6,1.6,4.0,4.0,4.0],[1.6,1.6,1.6,4.0,4.0,4.0])
                        self.send_joint_state_real([0.0, 0.0, -0.6, 0.0, 0.0, 0.0],[0.0, 0.0, -0.6, 0.0, 0.0, 0.0],[0.0, 0.0, 0.0])
                        time.sleep(0.4)
                        self.send_joint_state_real([0.3, 2.24, -1.5, 0.2, 0.0, 0.0],[0.3, 2.24, -1.5, 0.2, 0.0, 0.0],[0.0, 0.0, 0.0])
                        time.sleep(1.0)
                        self.send_joint_state_real([0.0, 0.0, 0.0, 0.0, 0.0, 0.0],[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],[0.0, 0.0, 0.0])
                        time.sleep(1.0)
                    elif command == '3': # hand dance
                        self.send_breaking_mode([1])
                        j5_bias=0.00 #left arm
                        j1_bias=0.00
                        # self.send_vel_limit([3,3,3,5,5,5],[3,3,3,5,5,5], [0.5,0.79,0.7,1.0])
                        self.send_vel_limit([2.4,2.4,2.4,4.0,4.0,4.0],[2.4,2.4,2.4,4.0,4.0,4.0])
                        self.send_joint_state_real([0.0,1.15,-1.19,0.0,0.0,0.0],[0.0,1.15,-1.19,0.0,0.0,0.0],[0.0,0.0,0.0])
                        time.sleep(2)  # 防止重复发送
                        self.send_joint_state_real([0.8,1.15,-1.19,0.0,0.0,0.0],[0.8,1.15,-1.19,0.0,0.0,0.0],[0.0,0.0,0.0])
                        time.sleep(2)  # 防止重复发送
                        self.send_joint_state_real([-0.8,1.15,-1.19,0.0,0.0,0.0],[-0.8,1.15,-1.19,0.0,0.0,0.0],[0.0,0.0,0.0])
                        time.sleep(2)  # 防止重复发送
                        self.send_joint_state_real([0.0,1.15,-1.19,0.0,0.0,0.0],[0.0,1.15,-1.19,0.0,0.0,0.0],[0.0,0.0,0.0] )
                        time.sleep(2)  # 防止重复发送
                        self.send_joint_state_real([-1.5703,0.9,-1.18,0.59,0.0,0.0],[1.5703,0.9,-1.18,0.59,0.0,0.0],[0.0,0.0,0.0])
                        time.sleep(3)  # 防止重复发送
                        self.send_joint_state_real([0.0, 0.0, 0.0, 0.0, 0.0, 0.0],[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],[0.0,0.0,0.0])
                        time.sleep(3)  # 防止重复发送
                    elif command == '4': # on your knees  
                        self.send_breaking_mode([1])     
                        self.send_vel_limit([1.6,1.6,1.6,4.0,4.0,4.0],[1.6,1.6,1.6,4.0,4.0,4.0])      
                        self.send_joint_state_real([0.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0,0.0],[-1.39, 2.53, 1.14])
                        time.sleep(1)
                    elif command == '5': # t4 test
                        self.send_breaking_mode([1])
                        self.send_vel_limit([1.6,1.6,1.6,4.0,4.0,4.0],[1.6,1.6,1.6,4.0,4.0,4.0])
                        self.send_joint_state_real([0.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0])
                        time.sleep(1)
                    elif command == '6':
                        self.send_breaking_mode([1])
                        # print("hahaha")
                        self.send_gripper_joint_state([100.0],[100.0])
                        time.sleep(1)
                    elif command == '7':
                        self.send_breaking_mode([1])
                        # print("hahaha111")
                        self.send_gripper_joint_state([0.0],[0.0])
                        time.sleep(1)
                    elif command == 'q':
                        self.send_breaking_mode([0]) 
                        rclpy.shutdown()
                        sys.exit(0)
                        break
                except KeyboardInterrupt:
                    self.get_logger().info("Keyboard interrupt received, shutting down...")
                    rclpy.shutdown()
                    sys.exit(0)
                    break
        except KeyboardInterrupt:
            print("Exit the program")
        finally:
            # rospy.signal_shutdown()
            print("shutting the ROS node")
            sys.exit(0)     


def main(args=None):
    rclpy.init(args=args)
    sender = JointStateSender()
    try:
        sender.run()
    except KeyboardInterrupt:
        pass
    finally:
        sender.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
