# control_torso_complete.py
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from hdas_msg.msg import MotorControl
from std_msgs.msg import Bool
import sys
import time

class TorsoCommander:
    def __init__(self):
        rclpy.init()
        self.node = Node('torso_commander')
        
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
        
        self.torso_pub = self.node.create_publisher(
            MotorControl, '/motion_control/control_torso', qos_torso
        )
        self.brake_pub = self.node.create_publisher(
            Bool, '/motion_target/brake_mode', qos_brake
        )
        
        time.sleep(0.5)  # 等待发布者建立
        
    def set_brake(self, enabled=True):
        """设置刹车模式"""
        msg = Bool()
        msg.data = enabled
        self.brake_pub.publish(msg)
        print(f"刹车模式: {'开启' if enabled else '关闭'}")
        time.sleep(0.2)
        return self
    
    def send_position(self, positions, wait=1.0):
        """发送躯干位置"""
        msg = MotorControl()
        msg.p_des = positions
        msg.v_des = [0.43, 0.7, 0.3, 0.3]
        msg.kp = [140.0, 200.0, 120.0, 20.0]
        msg.kd = [10.0, 50.0, 5.0, 1.0]
        msg.t_ff = [0.0, 0.0, 0.0, 0.0]
        
        # 发送多次确保接收
        for i in range(5):
            self.torso_pub.publish(msg)
            time.sleep(0.05)
        
        print(f"位置指令发送: {positions}")
        if wait > 0:
            time.sleep(wait)
        
        return self
    
    def shutdown(self):
        self.node.destroy_node()
        rclpy.shutdown()

def main():
    commander = TorsoCommander()
    
    try:
        if len(sys.argv) == 1:
            print("躯干控制命令")
            print("=" * 40)
            print("用法:")
            print("  1. 预设动作: python3 control_torso_complete.py <动作名>")
            print("  2. 自定义: python3 control_torso_complete.py <j1> <j2> <j3> <j4>")
            print("\n预设动作:")
            print("  stand  - 站立 (0,0,0,0)")
            print("  kneel  - 跪下 (-1.39,2.53,1.14,0)")
            print("  zero   - 归零")
            print("  test1  - 关节1测试 (0.5,0,0,0)")
            print("  test2  - 关节2测试 (0,0.5,0,0)")
            print("  test3  - 关节3测试 (0,0,0.5,0)")
            print("  test4  - 关节4测试 (0,0,0,0.5)")
            return
        
        arg = sys.argv[1]
        
        # 预设动作
        presets = {
            'stand': [0.0, 0.0, 0.0, 0.0],
            'kneel': [-1.39, 2.53, 1.14, 0.0],
            'zero': [0.0, 0.0, 0.0, 0.0],
            'test1': [0.5, 0.0, 0.0, 0.0],
            'test2': [0.0, 0.5, 0.0, 0.0],
            'test3': [0.0, 0.0, 0.5, 0.0],
            'test4': [0.0, 0.0, 0.0, 0.5],
        }
        
        if arg in presets:
            print(f"执行预设动作: {arg}")
            commander.set_brake(True).send_position(presets[arg], wait=2.0)
        
        elif len(sys.argv) == 5:
            try:
                positions = [float(x) for x in sys.argv[1:5]]
                commander.set_brake(True).send_position(positions, wait=2.0)
            except ValueError:
                print("错误: 参数必须是数字")
        
        else:
            print("错误: 参数数量不正确")
        
        print("\n操作完成!")
        
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        commander.shutdown()

if __name__ == '__main__':
    main()