import rclpy
from rclpy.node import Node
import h5py
import numpy as np
import os
import time
from typing import Dict, Any
import yaml
from std_srvs.srv import Trigger
from geometry_msgs.msg import TwistStamped
from robot_msgs.msg import YamCmd, RobotCmd
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


def load_topics_config(config_path):
    with open(config_path) as f:
        config = yaml.safe_load(f)
    return config


class DataReplayNode(Node):
    def __init__(self):
        super().__init__('data_replay')
        
        # Declare parameters
        self.declare_parameter('hdf5_file', '')
        self.declare_parameter('config_path', '')
        self.declare_parameter('loop', False)  # Whether to loop the replay
        self.declare_parameter('arm_type', 'yam')  # Arm type: 'yam' or 'arx'
        
        hdf5_file = self.get_parameter('hdf5_file').value
        config_path = self.get_parameter('config_path').value
        self.loop = self.get_parameter('loop').value
        self.arm_type = self.get_parameter('arm_type').value
        
        # Set command topics based on arm type
        if self.arm_type == 'arx':
            self.CmdMsg = RobotCmd
            self.left_cmd_topic = '/ARX_INFER_L'
            self.right_cmd_topic = '/ARX_INFER_R'
        elif self.arm_type == 'yam':
            self.CmdMsg = YamCmd
            self.left_cmd_topic = '/YAM_VR_L'
            self.right_cmd_topic = '/YAM_VR_R'
        else:
            self.get_logger().fatal(f"Invalid arm_type: {self.arm_type}. Must be 'yam' or 'arx'.")
            raise ValueError(f"Invalid arm_type: {self.arm_type}")
        
        if not hdf5_file or not os.path.exists(hdf5_file):
            self.get_logger().error(f"HDF5 file not found: {hdf5_file}")
            raise FileNotFoundError(f"HDF5 file not found: {hdf5_file}")
        
        if not config_path or not os.path.exists(config_path):
            self.get_logger().error(f"Config file not found: {config_path}")
            raise FileNotFoundError(f"Config file not found: {config_path}")
        
        self.config = load_topics_config(config_path)
        self.hdf5_file = hdf5_file
        
        # Load HDF5 data
        self.hf = h5py.File(hdf5_file, 'r')
        self.data = {}
        self.num_frames = None
        
        # Load all datasets
        for dataset_name in self.config['datasets'].keys():
            if dataset_name in self.hf.keys():
                self.data[dataset_name] = self.hf[dataset_name][:]
                if self.num_frames is None:
                    self.num_frames = len(self.data[dataset_name])
                elif len(self.data[dataset_name]) != self.num_frames:
                    self.get_logger().warn(
                        f"Dataset {dataset_name} has {len(self.data[dataset_name])} frames, "
                        f"expected {self.num_frames}"
                    )
                # Debug: log loaded datasets
                if 'torso' in dataset_name.lower() or 'chassis' in dataset_name.lower():
                    sample_data = self.data[dataset_name][0] if len(self.data[dataset_name]) > 0 else None
                    self.get_logger().info(
                        f"Loaded dataset {dataset_name}: {len(self.data[dataset_name])} frames, "
                        f"sample data: {sample_data}"
                    )
                # Debug: log joint position datasets
                if 'joint_position' in dataset_name.lower():
                    sample_data = self.data[dataset_name][0] if len(self.data[dataset_name]) > 0 else None
                    self.get_logger().info(
                        f"Loaded dataset {dataset_name}: {len(self.data[dataset_name])} frames, "
                        f"sample data shape: {sample_data.shape if hasattr(sample_data, 'shape') else type(sample_data)}, "
                        f"sample values: {sample_data}"
                    )
            else:
                # Warn if expected dataset is missing
                if 'torso' in dataset_name.lower() or 'chassis' in dataset_name.lower():
                    self.get_logger().warn(f"Dataset {dataset_name} not found in HDF5 file")
                elif 'joint_position' in dataset_name.lower():
                    self.get_logger().warn(f"Dataset {dataset_name} not found in HDF5 file")
        
        # Get collection frequency from HDF5 or config
        if 'collection_frequency' in self.hf.keys():
            self.frequency = float(self.hf['collection_frequency'][()])
        else:
            self.frequency = self.config.get('collection_frequency', 50.0)
        
        self.get_logger().info(f"Loaded {self.num_frames} frames from {hdf5_file}")
        self.get_logger().info(f"Replay frequency: {self.frequency} Hz")
        
        # Setup publishers
        self.topic_publishers = {}
        self.setup_publishers()
        
        # Replay state
        self.current_frame = 0
        self.is_replaying = False
        self.timer = None
        
        # Service for control
        self.start_srv = self.create_service(Trigger, 'start_replay', self.start_callback)
        self.stop_srv = self.create_service(Trigger, 'stop_replay', self.stop_callback)
        
    def setup_publishers(self):
        """Setup ROS2 publishers for command topics (not status/camera topics)"""
        # Create QoS profile for arm commands (RELIABLE to match C++ controller subscribers)
        qos_profile_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Create publishers for arm command topics
        self.topic_publishers[self.left_cmd_topic] = self.create_publisher(
            self.CmdMsg,
            self.left_cmd_topic,
            qos_profile_reliable
        )
        self.get_logger().info(f"Created publisher for {self.left_cmd_topic} ({self.CmdMsg.__name__}, RELIABLE)")
        
        self.topic_publishers[self.right_cmd_topic] = self.create_publisher(
            self.CmdMsg,
            self.right_cmd_topic,
            qos_profile_reliable
        )
        self.get_logger().info(f"Created publisher for {self.right_cmd_topic} ({self.CmdMsg.__name__}, RELIABLE)")
        
        # Check for subscribers after a short delay
        self.create_timer(2.0, self.check_subscribers)
        
        # Create publishers for chassis and torso target topics (if data exists)
        chassis_topic = '/motion_target/target_speed_chassis'
        torso_topic = '/motion_target/target_speed_torso'
        
        # Check if chassis/torso data exists in datasets
        for dataset_name, dataset_cfg in self.config['datasets'].items():
            if dataset_name not in self.data:
                continue
            topic_name = dataset_cfg['topic']
            if topic_name == chassis_topic and topic_name not in self.topic_publishers:
                self.topic_publishers[chassis_topic] = self.create_publisher(
                    TwistStamped,
                    chassis_topic,
                    qos_profile_reliable
                )
                self.get_logger().info(f"Created publisher for {chassis_topic} (RELIABLE)")
            elif topic_name == torso_topic and topic_name not in self.topic_publishers:
                self.topic_publishers[torso_topic] = self.create_publisher(
                    TwistStamped,
                    torso_topic,
                    qos_profile_reliable
                )
                self.get_logger().info(f"Created publisher for {torso_topic} (RELIABLE)")
    
    def start_callback(self, request, response):
        if self.is_replaying:
            response.success = False
            response.message = "Already replaying data"
            self.get_logger().warn("⚠️  Replay already in progress, cannot start again")
            return response
        
        self.is_replaying = True
        self.current_frame = 0
        interval = 1.0 / self.frequency
        self.timer = self.create_timer(interval, self.replay_frame)
        
        response.success = True
        response.message = f"Started replaying at {self.frequency}Hz"
        self.get_logger().info("=" * 60)
        self.get_logger().info("▶️  数据回放已启动")
        self.get_logger().info(f"   频率: {self.frequency} Hz")
        self.get_logger().info(f"   总帧数: {self.num_frames}")
        self.get_logger().info(f"   循环播放: {'是' if self.loop else '否'}")
        self.get_logger().info("=" * 60)
        return response
    
    def stop_callback(self, request, response):
        if not self.is_replaying:
            response.success = False
            response.message = "Not currently replaying"
            self.get_logger().warn("⚠️  当前未在回放，无法停止")
            return response
        
        self.is_replaying = False
        if self.timer:
            self.timer.cancel()
        
        response.success = True
        response.message = "Stopped replaying"
        self.get_logger().info("=" * 60)
        self.get_logger().info("⏸️  数据回放已停止")
        self.get_logger().info(f"   已回放帧数: {self.current_frame}/{self.num_frames}")
        self.get_logger().info(f"   进度: {self.current_frame/self.num_frames*100:.1f}%")
        self.get_logger().info("=" * 60)
        return response
    
    def replay_frame(self):
        """Publish one frame of data as commands (not status/camera)"""
        if self.current_frame >= self.num_frames:
            if self.loop:
                self.current_frame = 0
                self.get_logger().info("Looping replay from beginning")
            else:
                self.is_replaying = False
                if self.timer:
                    self.timer.cancel()
                self.get_logger().info("Replay completed")
                return
        
        # Process datasets and publish commands
        # Skip camera data (ImageProcessor, DepthProcessor)
        # Convert joint position data to command messages
        
        # Extract joint position data
        left_joint_pos = None
        right_joint_pos = None
        
        for dataset_name, dataset_cfg in self.config['datasets'].items():
            if dataset_name not in self.data:
                continue
            
            processor_name = dataset_cfg['processor'].split('.')[-1]
            
            # Skip camera data
            if processor_name in ['ImageProcessor', 'DepthProcessor']:
                continue
            
            # Extract joint position data
            if processor_name == 'JointPositionProcessor':
                data = self.data[dataset_name][self.current_frame]
                if '/state/joint_position/left' in dataset_name:
                    left_joint_pos = data
                    # Debug: log first frame and every 100 frames
                    if self.current_frame == 0 or self.current_frame % 100 == 0:
                        self.get_logger().info(
                            f"Extracted left joint data frame {self.current_frame}: "
                            f"shape={data.shape if hasattr(data, 'shape') else 'N/A'}, "
                            f"values={data}"
                        )
                elif '/state/joint_position/right' in dataset_name:
                    right_joint_pos = data
                    # Debug: log first frame and every 100 frames
                    if self.current_frame == 0 or self.current_frame % 100 == 0:
                        self.get_logger().info(
                            f"Extracted right joint data frame {self.current_frame}: "
                            f"shape={data.shape if hasattr(data, 'shape') else 'N/A'}, "
                            f"values={data}"
                        )
            
            # Publish chassis and torso target data
            elif processor_name in ['TwistProcessor', 'OdomProcessor']:
                topic_name = dataset_cfg['topic']
                if topic_name in self.topic_publishers:
                    data = self.data[dataset_name][self.current_frame]
                    msg = self.convert_twist(data, TwistStamped)
                    if msg is not None:
                        self.topic_publishers[topic_name].publish(msg)
                        # Debug: log first frame and every 100 frames for torso/chassis
                        if self.current_frame == 0 or self.current_frame % 100 == 0:
                            if 'torso' in topic_name.lower():
                                self.get_logger().info(
                                    f"Published torso data frame {self.current_frame}: "
                                    f"linear=({msg.twist.linear.x:.3f}, {msg.twist.linear.y:.3f}, {msg.twist.linear.z:.3f}), "
                                    f"angular=({msg.twist.angular.x:.3f}, {msg.twist.angular.y:.3f}, {msg.twist.angular.z:.3f})"
                                )
                            elif 'chassis' in topic_name.lower():
                                self.get_logger().info(
                                    f"Published chassis data frame {self.current_frame}: "
                                    f"linear=({msg.twist.linear.x:.3f}, {msg.twist.linear.y:.3f}, {msg.twist.linear.z:.3f}), "
                                    f"angular=({msg.twist.angular.x:.3f}, {msg.twist.angular.y:.3f}, {msg.twist.angular.z:.3f})"
                                )
        
        # Publish arm commands
        if left_joint_pos is not None:
            left_cmd = self.create_arm_cmd(left_joint_pos)
            if left_cmd is not None:
                self.topic_publishers[self.left_cmd_topic].publish(left_cmd)
                # Debug: log first frame and every 100 frames
                if self.current_frame == 0 or self.current_frame % 100 == 0:
                    self.get_logger().info(
                        f"Published left arm command frame {self.current_frame} to {self.left_cmd_topic}: "
                        f"joint_pos={left_cmd.joint_pos}, gripper={left_cmd.gripper:.3f}, mode={left_cmd.mode}"
                    )
            else:
                if self.current_frame == 0 or self.current_frame % 100 == 0:
                    self.get_logger().warn(f"Failed to create left arm command for frame {self.current_frame}")
        else:
            if self.current_frame == 0 or self.current_frame % 100 == 0:
                self.get_logger().warn(f"No left joint position data for frame {self.current_frame}")
        
        if right_joint_pos is not None:
            right_cmd = self.create_arm_cmd(right_joint_pos)
            if right_cmd is not None:
                self.topic_publishers[self.right_cmd_topic].publish(right_cmd)
                # Debug: log first frame and every 100 frames
                if self.current_frame == 0 or self.current_frame % 100 == 0:
                    self.get_logger().info(
                        f"Published right arm command frame {self.current_frame} to {self.right_cmd_topic}: "
                        f"joint_pos={right_cmd.joint_pos}, gripper={right_cmd.gripper:.3f}, mode={right_cmd.mode}"
                    )
            else:
                if self.current_frame == 0 or self.current_frame % 100 == 0:
                    self.get_logger().warn(f"Failed to create right arm command for frame {self.current_frame}")
        else:
            if self.current_frame == 0 or self.current_frame % 100 == 0:
                self.get_logger().warn(f"No right joint position data for frame {self.current_frame}")
        
        self.current_frame += 1
        
        if self.current_frame % 100 == 0:
            self.get_logger().info(f"Replayed {self.current_frame}/{self.num_frames} frames")
    
    def check_subscribers(self):
        """Check if there are subscribers for arm command topics"""
        left_count = self.topic_publishers[self.left_cmd_topic].get_subscription_count()
        right_count = self.topic_publishers[self.right_cmd_topic].get_subscription_count()
        
        if left_count == 0:
            self.get_logger().warn("=" * 60)
            self.get_logger().warn(f"⚠️  警告: 没有订阅者订阅 {self.left_cmd_topic}")
            self.get_logger().warn("   可能原因:")
            self.get_logger().warn("   1. yam_damiao_controller 未运行")
            self.get_logger().warn("   2. 控制器还在 INIT 状态（需要完成归零）")
            self.get_logger().warn("   3. 控制器在 SOFT 状态（已禁用）")
            self.get_logger().warn("=" * 60)
        else:
            self.get_logger().info(f"✅ 找到 {left_count} 个订阅者订阅 {self.left_cmd_topic}")
        
        if right_count == 0:
            self.get_logger().warn("=" * 60)
            self.get_logger().warn(f"⚠️  警告: 没有订阅者订阅 {self.right_cmd_topic}")
            self.get_logger().warn("   可能原因:")
            self.get_logger().warn("   1. yam_damiao_controller 未运行")
            self.get_logger().warn("   2. 控制器还在 INIT 状态（需要完成归零）")
            self.get_logger().warn("   3. 控制器在 SOFT 状态（已禁用）")
            self.get_logger().warn("=" * 60)
        else:
            self.get_logger().info(f"✅ 找到 {right_count} 个订阅者订阅 {self.right_cmd_topic}")
    
    def create_arm_cmd(self, joint_data: np.ndarray) -> Any:
        """Create arm command message from joint position data"""
        # joint_data is 7 values: [j1, j2, j3, j4, j5, j6, gripper]
        if len(joint_data) < 7:
            self.get_logger().warn(f"Joint data has {len(joint_data)} values, expected 7")
            return None
        
        msg = self.CmdMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        # Set joint positions (first 6 joints)
        msg.joint_pos = joint_data[:6].tolist()
        
        # Set gripper (7th value)
        msg.gripper = float(joint_data[6])
        
        # Set mode to 5 (JOINT_CONTROL)
        msg.mode = 5
        
        # Initialize end_pose if it exists (for YamCmd)
        if hasattr(msg, 'end_pose'):
            # Set a default end_pose (will be ignored in JOINT_CONTROL mode)
            msg.end_pose = [0.0] * 7
        
        return msg
    
    def convert_twist(self, data: np.ndarray, msg_class) -> TwistStamped:
        """Convert numpy array to TwistStamped message"""
        msg = msg_class()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        # Data format: [angular.x, angular.y, angular.z, linear.x, linear.y, linear.z]
        if len(data) >= 6:
            msg.twist.angular.x = float(data[0])
            msg.twist.angular.y = float(data[1])
            msg.twist.angular.z = float(data[2])
            msg.twist.linear.x = float(data[3])
            msg.twist.linear.y = float(data[4])
            msg.twist.linear.z = float(data[5])
        
        return msg
    
    def destroy_node(self):
        """Clean up resources"""
        if hasattr(self, 'hf') and self.hf:
            self.hf.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = DataReplayNode()
        rclpy.spin(node)
    except FileNotFoundError as e:
        print(f"Error: {e}")
        print("Usage: ros2 run inference data_replay_node --ros-args "
              "-p hdf5_file:=<path_to_hdf5> -p config_path:=<path_to_config>")
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
