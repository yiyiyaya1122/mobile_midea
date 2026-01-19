import numpy as np
import rclpy
from rclpy.node import Node
from transforms3d.quaternions import mat2quat
from quest3s_vr.hands import HandVuer
from geometry_msgs.msg import PoseArray, Pose, Quaternion, PoseStamped, Point
from multiprocessing import Array, Process
from pathlib import Path
from ament_index_python.packages import get_package_share_directory


def load_cert(package_name='quest3s_vr'):
    cert_path = Path(get_package_share_directory(package_name)) / 'config/cert.pem'
    key_path = Path(get_package_share_directory(package_name)) / 'config/key.pem'

    return cert_path, key_path


class VRPublisher(Node):
    def __init__(self):
        super().__init__('quest3s_hand_publisher')
        # Create publishers
        self.left_pose_pub = self.create_publisher(PoseArray, '/vr_hand_left', 5)
        self.right_pose_pub = self.create_publisher(PoseArray, '/vr_hand_right', 5)
        self.head_pose_pub = self.create_publisher(PoseStamped, '/vr_pose_head', 5)
        self.timer = self.create_timer(0.02, self.timer_callback)

        cert_path, key_path = load_cert()
        self.vuer = HandVuer(cert=cert_path, key=key_path)
        self.process = Process(target=self.vuer.run)
        self.process.daemon = True
        self.process.start()

    def timer_callback(self):
        self.publish_hand_array(self.vuer.left_hand_matrices, 'left')
        self.publish_hand_array(self.vuer.right_hand_matrices, 'right')
        self.publish_pose(self.vuer.head_matrix)
        # self.publish_hand_state(self.vuer.left_controller_state, self.vuer.right_controller_state)

    def publish_hand_array(self, matrices, name):
        try:
            msg = PoseArray()
            for index, mat in enumerate(matrices):
                position = mat[:3, 3]
                rot = mat[:3, :3]
                q = mat2quat(rot)

                pose = Pose()
                # Position
                pose.position = Point(
                    x=float(position[0]),
                    y=float(position[1]),
                    z=float(position[2])
                )
                # Orientation (convert from [w, x, y, z] to [x, y, z, w])
                pose.orientation = Quaternion(
                    x=float(q[1]),
                    y=float(q[2]),
                    z=float(q[3]),
                    w=float(q[0])
                )

                msg.poses.append(pose)

            if name == 'left':
                self.left_pose_pub.publish(msg)
            elif name == 'right':
                self.right_pose_pub.publish(msg)
            else:
                raise RuntimeError(f'no name: {name}')

            msg.header.stamp = self.get_clock().now().to_msg()
        except Exception as e:
            self.get_logger().error(f"Error converting hand array: {str(e)}")


    def publish_pose(self, pose_data):
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

            # Position
            msg.pose.position = Point(
                x=float(position[0]),
                y=float(position[1]),
                z=float(position[2])
            )

            # Orientation (convert from [w, x, y, z] to [x, y, z, w])
            msg.pose.orientation = Quaternion(
                x=float(q[1]),
                y=float(q[2]),
                z=float(q[3]),
                w=float(q[0])
            )

            # print(f"matrix: {matrix}")
            # print(f"position: {position}")
            # print(f"rotation_matrix: {rotation_matrix}")
            # print(f"q: {q}")
            # print(f"msg: {msg}")

            self.head_pose_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Error converting pose: {str(e)}")

    # def publish_hand_state(self, left_state, right_state):
    #     """Convert and publish button states"""
    #     try:
    #         msg = Quest2Button()
    #         # print(f"left_state: {left_state}")
    #         # print(f"right_state: {right_state}")
    #
    #         # Digital buttons (assuming X/Y on left, A/B on right)
    #         msg.x = left_state.get('aButton', False)
    #         msg.y = left_state.get('bButton', False)
    #         msg.a = right_state.get('aButton', False)
    #         msg.b = right_state.get('bButton', False)
    #
    #         # Right controller
    #         msg.r_thu = abs(right_state.get('thumbstickValue', [0, 0])[1]) > 0.7
    #         msg.rj = right_state.get('thumbstick', False)
    #         msg.rg = right_state.get('squeeze', False)
    #         msg.rtr = right_state.get('trigger', False)
    #
    #         # Left controller
    #         msg.l_thu = abs(left_state.get('thumbstickValue', [0, 0])[1]) > 0.7
    #         msg.lj = left_state.get('thumbstick', False)
    #         msg.lg = left_state.get('squeeze', False)
    #         msg.ltr = left_state.get('trigger', False)
    #
    #         # Analog values
    #         msg.left_joystick = np.array(left_state.get('thumbstickValue', [0.0, 0.0]), dtype=np.float32)
    #         msg.left_trigger = float(left_state.get('triggerValue', 0.0))
    #         msg.left_grip = float(left_state.get('squeezeValue', 0.0))
    #         msg.right_joystick = np.array(right_state.get('thumbstickValue', [0.0, 0.0]), dtype=np.float32)
    #         msg.right_trigger = float(right_state.get('triggerValue', 0.0))
    #         msg.right_grip = float(right_state.get('squeezeValue', 0.0))
    #
    #         self.button_pub.publish(msg)
    #
    #     except Exception as e:
    #         self.get_logger().error(f"Error converting buttons: {str(e)}")


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