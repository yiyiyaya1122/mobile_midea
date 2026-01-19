import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from transforms3d.quaternions import mat2quat
from quest2_button_msg.msg import Quest2Button
from quest3s_vr.controllers import ControllerVuer
from yam_arm_msg.msg import YamStatus
from multiprocessing import Array, Process
from pathlib import Path
from ament_index_python.packages import get_package_share_directory


def load_cert(package_name='quest3s_vr'):
    cert_path = Path(get_package_share_directory(package_name)) / 'config/cert.pem'
    key_path = Path(get_package_share_directory(package_name)) / 'config/key.pem'

    return cert_path, key_path


class VRPublisher(Node):
    def __init__(self):
        super().__init__('quest3s_node')
        # Create publishers
        self.left_pose_pub = self.create_publisher(PoseStamped, '/vr_pose_left', 5)
        self.right_pose_pub = self.create_publisher(PoseStamped, '/vr_pose_right', 5)
        self.button_pub = self.create_publisher(Quest2Button, '/vr_button', 5)
        self.head_pose_pub = self.create_publisher(PoseStamped, '/vr_pose_head', 5)

        # Create subscribers for robot status
        self.left_status_sub = self.create_subscription(
            YamStatus,
            '/arm_l_status',
            self.left_status_callback,
            5)
        self.right_status_sub = self.create_subscription(
            YamStatus,
            '/arm_r_status',
            self.right_status_callback,
            5)

        self.timer = self.create_timer(0.02, self.timer_callback)

        cert_path, key_path = load_cert()
        self.vuer = ControllerVuer(cert=cert_path, key=key_path)
        self.process = Process(target=self.vuer.run)
        self.process.daemon = True
        self.process.start()

    def left_status_callback(self, msg: YamStatus):
        """Callback for the left arm's status."""
        try:
            self.vuer.left_arm_joints = msg.joint_pos[:6]
        except Exception as e:
            self.get_logger().error(f"Error in left_status_callback: {e}")

    def right_status_callback(self, msg: YamStatus):
        """Callback for the right arm's status."""
        try:
            self.vuer.right_arm_joints = msg.joint_pos[:6]
        except Exception as e:
            self.get_logger().error(f"Error in right_status_callback: {e}")

    def timer_callback(self):
        self.publish_pose(self.vuer.left_controller_matrix, 'left')
        self.publish_pose(self.vuer.right_controller_matrix, 'right')
        self.publish_pose(self.vuer.head_matrix, 'head')
        self.publish_buttons(self.vuer.left_controller_state, self.vuer.right_controller_state)

    def publish_pose(self, pose_data, name):
        """Convert and publish pose data"""
        try:
            matrix = np.array(pose_data).reshape((4, 4))
            position = matrix[:3, 3]
            rotation_matrix = matrix[:3, :3]
            q = mat2quat(rotation_matrix)

            # Create PoseStamped message
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            msg.pose.position.x = float(position[0])
            msg.pose.position.y = float(position[1])
            msg.pose.position.z = float(position[2])

            # t3d mat2quat
            msg.pose.orientation.x = float(q[1])
            msg.pose.orientation.y = float(q[2])
            msg.pose.orientation.z = float(q[3])
            msg.pose.orientation.w = float(q[0])

            if name == 'left':
                self.left_pose_pub.publish(msg)
            if name == 'right':
                self.right_pose_pub.publish(msg)
            if name == 'head':
                self.head_pose_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Error converting pose: {str(e)}")

    def publish_buttons(self, left_state, right_state):
        """Convert and publish button states"""
        try:
            msg = Quest2Button()

            # Digital buttons (assuming X/Y on left, A/B on right)
            msg.x = left_state.get('aButton', False)
            msg.y = left_state.get('bButton', False)
            msg.a = right_state.get('aButton', False)
            msg.b = right_state.get('bButton', False)

            # Right controller
            msg.r_thu = abs(right_state.get('thumbstickValue', [0, 0])[1]) > 0.7
            msg.rj = right_state.get('thumbstick', False)
            msg.rg = right_state.get('squeeze', False)
            msg.rtr = right_state.get('trigger', False)

            # Left controller
            msg.l_thu = abs(left_state.get('thumbstickValue', [0, 0])[1]) > 0.7
            msg.lj = left_state.get('thumbstick', False)
            msg.lg = left_state.get('squeeze', False)
            msg.ltr = left_state.get('trigger', False)

            # Analog values
            msg.left_joystick = np.array(left_state.get('thumbstickValue', [0.0, 0.0]), dtype=np.float32)
            msg.left_trigger = float(left_state.get('triggerValue', 0.0))
            msg.left_grip = float(left_state.get('squeezeValue', 0.0))
            msg.right_joystick = np.array(right_state.get('thumbstickValue', [0.0, 0.0]), dtype=np.float32)
            msg.right_trigger = float(right_state.get('triggerValue', 0.0))
            msg.right_grip = float(right_state.get('squeezeValue', 0.0))

            self.button_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Error converting buttons: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = VRPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()