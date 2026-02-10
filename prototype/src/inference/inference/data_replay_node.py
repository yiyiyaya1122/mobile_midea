import rclpy
from rclpy.node import Node
import h5py
import numpy as np
import os
from typing import Any
import yaml
from std_srvs.srv import Trigger
from geometry_msgs.msg import TwistStamped
from robot_msgs.msg import YamCmd, RobotCmd
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

def load_topics_config(config_path):
    with open(config_path) as f:
        config = yaml.safe_load(f)
    return config

MODE_IK = 4
MODE_JOINT = 5
# 夹爪指令范围：0=闭合, 5=张开；HDF5 中存的是归一化 [0,1]，replay 时需恢复到此范围
GRIPPER_OPEN = 5.0

class DataReplayNode(Node):
    def __init__(self):
        super().__init__('data_replay')

        # 参数
        self.declare_parameter('hdf5_file', '')
        self.declare_parameter('config_path', '')
        self.declare_parameter('loop', False)
        self.declare_parameter('arm_type', 'yam')
        self.declare_parameter('arm_replay_mode', 'end_pose')
        self.declare_parameter('replay_frequency', 0)  # Hz; 0 = use config's collection_frequency (int from launch)

        self.hdf5_file = self.get_parameter('hdf5_file').value
        self.config_path = self.get_parameter('config_path').value
        self.loop = self.get_parameter('loop').value
        self.arm_type = self.get_parameter('arm_type').value
        self.arm_replay_mode = self.get_parameter('arm_replay_mode').value

        if self.arm_type == 'arx' and self.arm_replay_mode == 'end_pose':
            self.get_logger().warn("end_pose only supported on yam, fallback to joint")
            self.arm_replay_mode = 'joint'

        # 指令消息类型和话题
        if self.arm_type == 'yam':
            self.CmdMsg = YamCmd
            self.left_cmd_topic = '/YAM_VR_L'
            self.right_cmd_topic = '/YAM_VR_R'
        else:
            self.CmdMsg = RobotCmd
            self.left_cmd_topic = '/ARX_INFER_L'
            self.right_cmd_topic = '/ARX_INFER_R'

        # 加载配置和HDF5
        self.config = load_topics_config(self.config_path)
        self.hf = h5py.File(self.hdf5_file, 'r')
        self.data = {}
        self.num_frames = 0

        for name in self.config['datasets']:
            if name in self.hf:
                self.data[name] = self.hf[name][:]
                self.num_frames = max(self.num_frames, len(self.data[name]))

        freq_param = self.get_parameter('replay_frequency').value
        self.frequency = float(freq_param) if (isinstance(freq_param, (int, float)) and float(freq_param) > 0) else self.config.get('collection_frequency', 50.0)
        self.get_logger().info(f"Replay frequency: {self.frequency} Hz")

        # 发布器
        self.topic_publishers = {}
        self.setup_publishers()

        # 回放状态
        self.current_frame = 0
        self.is_replaying = False
        self.timer = None

        # 服务
        self.create_service(Trigger, 'start_replay', self.start_callback)
        self.create_service(Trigger, 'stop_replay', self.stop_callback)

    def setup_publishers(self):
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.topic_publishers[self.left_cmd_topic] = self.create_publisher(
            self.CmdMsg, self.left_cmd_topic, qos)
        self.topic_publishers[self.right_cmd_topic] = self.create_publisher(
            self.CmdMsg, self.right_cmd_topic, qos)

    def start_callback(self, req, res):
        if self.is_replaying:
            res.success = False
            res.message = "Already replaying"
            return res
        self.is_replaying = True
        self.current_frame = 0
        self.timer = self.create_timer(1.0 / self.frequency, self.replay_frame)
        res.success = True
        res.message = "Replay started"
        self.get_logger().info(f"▶️  数据回放启动, 模式: {self.arm_replay_mode}, 循环: {self.loop}")
        return res

    def stop_callback(self, req, res):
        if not self.is_replaying:
            res.success = False
            res.message = "Not replaying"
            return res
        self.is_replaying = False
        if self.timer:
            self.timer.cancel()
        res.success = True
        res.message = "Replay stopped"
        self.get_logger().info(f"⏸️  数据回放停止, 已回放帧: {self.current_frame}/{self.num_frames}")
        return res

    def replay_frame(self):
        if self.current_frame >= self.num_frames:
            if self.loop:
                self.current_frame = 0
                self.get_logger().info("🔁 循环回放，从头开始")
            else:
                self.is_replaying = False
                if self.timer:
                    self.timer.cancel()
                self.get_logger().info("✅ 回放完成")
                return

        # 数据准备
        left_joint_pos = None
        right_joint_pos = None
        left_end_pose = None
        right_end_pose = None
        left_gripper_cmd = None
        right_gripper_cmd = None

        for dataset_name, cfg in self.config['datasets'].items():
            if dataset_name not in self.data:
                continue

            data = self.data[dataset_name][self.current_frame]
            proc = cfg['processor'].split('.')[-1]

            if proc == 'JointPositionProcessor':
                if 'left' in dataset_name:
                    left_joint_pos = data
                else:
                    right_joint_pos = data
            elif proc == 'YamCmdEndPoseProcessor':
                pose = np.asarray(data).flatten()
                if 'left' in dataset_name:
                    left_end_pose = pose
                else:
                    right_end_pose = pose
            elif proc == 'YamCmdGripperProcessor':
                # 从归一化 [0,1] 恢复为指令范围 [0, 5]；若值已在 (1,5] 则视为未归一化，直接裁剪
                val = float(np.asarray(data).flatten()[0])
                if 0 <= val <= 1:
                    val = val * GRIPPER_OPEN
                else:
                    val = max(0.0, min(GRIPPER_OPEN, val))
                if '/state/gripper_cmd/left' in dataset_name:
                    left_gripper_cmd = val
                elif '/state/gripper_cmd/right' in dataset_name:
                    right_gripper_cmd = val

        # 决策
        use_end_pose = (
            self.arm_replay_mode == 'end_pose'
            and self.arm_type == 'yam'
            and (left_end_pose is not None or right_end_pose is not None)
        )

        if self.current_frame == 0:
            self.get_logger().warn(
                f"[DECISION] use_end_pose={use_end_pose}, L={left_end_pose is not None}, R={right_end_pose is not None}"
            )

        # 发布指令并打印日志（end_pose 模式下夹爪用 /state/gripper_cmd/left、/state/gripper_cmd/right，已从归一化恢复）
        if use_end_pose:
            if left_end_pose is not None:
                gripper = left_gripper_cmd if left_gripper_cmd is not None else GRIPPER_OPEN
                cmd = self.create_arm_cmd_from_end_pose(left_end_pose, gripper=gripper)
                if cmd is not None:
                    self.topic_publishers[self.left_cmd_topic].publish(cmd)
                    if self.current_frame % 100 == 0:
                        self.get_logger().info(
                            f"[END_POSE] L frame {self.current_frame}: pos={cmd.end_pose[:3]}, gripper={cmd.gripper}, mode={cmd.mode}"
                        )
            if right_end_pose is not None:
                gripper = right_gripper_cmd if right_gripper_cmd is not None else GRIPPER_OPEN
                cmd = self.create_arm_cmd_from_end_pose(right_end_pose, gripper=gripper)
                if cmd is not None:
                    self.topic_publishers[self.right_cmd_topic].publish(cmd)
                    if self.current_frame % 100 == 0:
                        self.get_logger().info(
                            f"[END_POSE] R frame {self.current_frame}: pos={cmd.end_pose[:3]}, gripper={cmd.gripper}, mode={cmd.mode}"
                        )
        else:
            if left_joint_pos is not None:
                cmd = self.create_arm_cmd(left_joint_pos)
                self.topic_publishers[self.left_cmd_topic].publish(cmd)
                if self.current_frame % 100 == 0:
                    self.get_logger().info(
                        f"[JOINT] L frame {self.current_frame}: joints={cmd.joint_pos}, gripper={cmd.gripper}, mode={cmd.mode}"
                    )
            if right_joint_pos is not None:
                cmd = self.create_arm_cmd(right_joint_pos)
                self.topic_publishers[self.right_cmd_topic].publish(cmd)
                if self.current_frame % 100 == 0:
                    self.get_logger().info(
                        f"[JOINT] R frame {self.current_frame}: joints={cmd.joint_pos}, gripper={cmd.gripper}, mode={cmd.mode}"
                    )

        self.current_frame += 1

    def create_arm_cmd(self, joint):
        msg = self.CmdMsg()
        msg.joint_pos = joint[:6].tolist()
        msg.gripper = float(joint[6])
        msg.mode = MODE_JOINT
        if hasattr(msg, 'end_pose'):
            msg.end_pose = [0.0]*7
        return msg

    def create_arm_cmd_from_end_pose(self, pose, gripper=None):
        if gripper is None:
            gripper = GRIPPER_OPEN
        # HDF5 stores end_pose as (x y z x y z w); YamCmd expects (x y z w x y z)
        p = np.asarray(pose).flatten()[:7]
        if len(p) < 7:
            return None
        msg = YamCmd()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.end_pose = [float(p[0]), float(p[1]), float(p[2]), float(p[6]), float(p[3]), float(p[4]), float(p[5])]
        msg.gripper = float(gripper)
        msg.mode = MODE_IK
        msg.joint_pos = [0.0] * 6
        return msg

def main():
    rclpy.init()
    node = DataReplayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
