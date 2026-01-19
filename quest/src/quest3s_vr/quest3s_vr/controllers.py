import numpy as np
import asyncio
from multiprocessing import Array, Manager
import msgpack
from msgpack import ExtType
from pathlib import Path
from scipy.spatial.transform import Rotation

# Vuer imports
from vuer import Vuer, VuerSession
from vuer.schemas import MotionControllers, DefaultScene, Urdf, CoordsMarker

# Pinocchio imports for Forward Kinematics
import pinocchio

from pathlib import Path
from ament_index_python.packages import get_package_share_directory

# URDF_PATH = "/home/midea/licm/quest3s/src/yam_description/yam.urdf"
URDF_PATH = Path(get_package_share_directory('yam_description')) / 'urdf/yam.urdf'
MODEL_PATH = "src/yam_description/urdf/yam.urdf"

class ControllerVuer:
    def __init__(
            self,
            host='0.0.0.0',
            port=8012,
            cert="./cert.pem",
            key="./key.pem",
            queries=dict(grid=False),
            ws_protocol="vuer",
    ):
        self.app = Vuer(
            host=host, port=port, cert=cert, key=key,
            queries=queries, ws_protocol=ws_protocol,
        )

        # Shared memory setup
        self.left_matrix_shared = Array('d', 16, lock=True)
        self.right_matrix_shared = Array('d', 16, lock=True)
        self.head_matrix_shared = Array('d', 16, lock=True)
        self.left_joint_shared = Array('d', 6, lock=True)
        self.right_joint_shared = Array('d', 6, lock=True)
        self.left_joint_shared[:] = [0.0] * 6
        self.right_joint_shared[:] = [0.0] * 6
        manager = Manager()
        self.left_state = manager.dict()
        self.right_state = manager.dict()

        for side in [self.left_state, self.right_state]:
            side.update({
                'trigger': False, 'squeeze': False, 'touchpad': False, 'thumbstick': False,
                'aButton': False, 'bButton': False, 'triggerValue': 0.0, 'squeezeValue': 0.0,
                'touchpadValue': [0.0, 0.0], 'thumbstickValue': [0.0, 0.0],
                'aButtonValue': False, 'bButtonValue': False,
            })

        try:
            urdf_path = Path(URDF_PATH)
            if not urdf_path.exists():
                raise FileNotFoundError(f"URDF file not found at {urdf_path.resolve()}. Please update URDF_PATH.")

            # base_mat_world = pinocchio.rpy.rpyToMatrix(np.array([-np.pi / 2, 0, np.pi / 2]))
            self.base_mat_world = Rotation.from_quat([-0.5, 0.5, 0.5, 0.5]).as_matrix()
            # print(f"base_mat_world: {base_mat_world.as_matrix()}")

            self.base_pose_world_l = pinocchio.SE3(self.base_mat_world, np.array([-0.3, 1, 0]))
            self.base_pose_world_r = pinocchio.SE3(self.base_mat_world, np.array([0.3, 1, 0]))

            self.handle_mat = Rotation.from_quat([-0.5, 0.5, 0.5, -0.5]).as_matrix()

            self.model = pinocchio.buildModelFromUrdf(str(urdf_path))
            self.data = self.model.createData()
            self.eef_frame_id = self.model.getFrameId("link6")

            q_init = np.zeros(6)
            pinocchio.forwardKinematics(self.model, self.data, q_init)
            pinocchio.updateFramePlacement(self.model, self.data, self.eef_frame_id)
            self.eef_matrix_base = self.data.oMf[self.eef_frame_id].copy().rotation

        except Exception as e:
            self.model = None
            self.data = None
            print(f"Error initializing Pinocchio model: {e}")

        self.app.add_handler("CONTROLLER_MOVE", self.handle_controller_move)
        self.app.add_handler("CAMERA_MOVE", self.handle_camera_move)

    async def handle_controller_move(self, event, session):
        # ... (no changes here)
        data = event.value
        try:
            if 'left' in data and not isinstance(data['left'], ExtType):
                with self.left_matrix_shared.get_lock(): self.left_matrix_shared[:] = data['left']
            if 'right' in data and not isinstance(data['right'], ExtType):
                with self.right_matrix_shared.get_lock(): self.right_matrix_shared[:] = data['right']
            if 'leftState' in data: self.left_state.update(data['leftState'])
            if 'rightState' in data: self.right_state.update(data['rightState'])
        except Exception as e:
            print(f"Controller move error: {e}")

    async def handle_camera_move(self, event, session):
        data = event.value
        try:
            if 'camera' in data and 'matrix' in data['camera']:
                matrix = data['camera']['matrix']
                if not isinstance(matrix, ExtType):
                    with self.head_matrix_shared.get_lock(): self.head_matrix_shared[:] = matrix
        except Exception as e:
            print(f"Camera move error: {e}")

    # Properties for shared data access
    @property
    def left_controller_matrix(self):
        with self.left_matrix_shared.get_lock(): return np.array(self.left_matrix_shared[:]).reshape(4, 4, order="F")
    @property
    def right_controller_matrix(self):
        with self.right_matrix_shared.get_lock(): return np.array(self.right_matrix_shared[:]).reshape(4, 4, order="F")
    @property
    def head_matrix(self):
        with self.head_matrix_shared.get_lock(): return np.array(self.head_matrix_shared[:]).reshape(4, 4, order="F")
    @property
    def left_controller_state(self): return dict(self.left_state)
    @property
    def right_controller_state(self): return dict(self.right_state)
    @property
    def left_arm_joints(self):
        with self.left_joint_shared.get_lock(): return list(self.left_joint_shared[:])
    @left_arm_joints.setter
    def left_arm_joints(self, values):
        if len(values) == 6:
            with self.left_joint_shared.get_lock():
                self.left_joint_shared[:] = values
        else:
            print(f"Error: Incorrect number of joint values for left arm. Expected 6, got {len(values)}")

    @property
    def right_arm_joints(self):
        with self.right_joint_shared.get_lock(): return list(self.right_joint_shared[:])
    @right_arm_joints.setter
    def right_arm_joints(self, values):
        if len(values) == 6:
            with self.right_joint_shared.get_lock():
                self.right_joint_shared[:] = values
        else:
            print(f"Error: Incorrect number of joint values for right arm. Expected 6, got {len(values)}")


    def run(self):
        @self.app.spawn(start=True)
        async def sense_loop(session: VuerSession):
            # MODIFICATION 2: Derive Vuer's initial pose from the single source of truth.
            # Convert rotation matrix back to RPY (Euler angles) for Vuer's `rotation` property.
            base_rpy_world_l = Rotation.from_matrix(self.base_pose_world_l.rotation).as_euler("XYZ")
            base_rpy_world_r = Rotation.from_matrix(self.base_pose_world_r.rotation).as_euler("XYZ")

            session.set @ DefaultScene(
                CoordsMarker(key="world", scale=1),
                CoordsMarker(key="handle_l", scale=0.1),
                CoordsMarker(key="handle_r", scale=0.1),

                CoordsMarker(key="base_link_l",                     
                    rotation=base_rpy_world_l,
                    position=self.base_pose_world_l.translation.tolist(),
                    scale=0.1
                ),
                Urdf(
                    src="https://localhost:8012/static/" + MODEL_PATH, key="robot_l",
                    rotation=base_rpy_world_l,
                    position=self.base_pose_world_l.translation.tolist(),
                    scale=1.0,
                ),
                CoordsMarker(key="eef_l", scale=0.1),

                CoordsMarker(key="base_link_r",                     
                    rotation=base_rpy_world_r,
                    position=self.base_pose_world_r.translation.tolist(),
                    scale=0.1
                ),
                Urdf(
                    src="https://localhost:8012/static/" + MODEL_PATH, key="robot_r",
                    rotation=base_rpy_world_r,
                    position=self.base_pose_world_r.translation.tolist(),
                    scale=1,
                ),
                CoordsMarker(key="eef_r", scale=0.1),
                grid=True,
            )
            session.upsert @ MotionControllers(stream=True, key="motion-controller")
            await asyncio.sleep(0.5)

            if not self.model:
                print("Pinocchio model not loaded. FK visualization will be skipped.")
                return

            joint_names = [f"joint{i + 1}" for i in range(6)]

            while True:
                if session.CURRENT_WS_ID not in session.vuer.ws:
                    await asyncio.sleep(0.05)
                    continue

                try:
                    q_l = np.array(self.left_arm_joints)
                    q_r = np.array(self.right_arm_joints)

                    # print(f"left_arm_joints: {self.left_arm_joints}")
                    # print(f"right_arm_joints: {self.right_arm_joints}")

                    handle_pose_l = pinocchio.SE3(self.left_controller_matrix[:3, :3], self.left_controller_matrix[:3, 3])
                    handle_pos_l = handle_pose_l.translation.tolist()
                    handle_mat_l = handle_pose_l.rotation @ self.base_mat_world.T @ self.handle_mat
                    handle_euler_l = Rotation.from_matrix(handle_mat_l).as_euler("XYZ")

                    handle_pose_r = pinocchio.SE3(self.right_controller_matrix[:3, :3], self.right_controller_matrix[:3, 3])
                    handle_pos_r = handle_pose_r.translation.tolist()
                    handle_mat_r = handle_pose_r.rotation @ self.base_mat_world.T @ self.handle_mat
                    handle_euler_r = Rotation.from_matrix(handle_mat_r).as_euler("XYZ")

                    # --- FK for Left Arm ---
                    pinocchio.forwardKinematics(self.model, self.data, q_l)
                    pinocchio.updateFramePlacement(self.model, self.data, self.eef_frame_id)
                    eef_pose_base_l = self.data.oMf[self.eef_frame_id]
                    eef_pose_base_l.rotation = (Rotation.from_matrix(eef_pose_base_l.rotation) * Rotation.from_matrix(self.eef_matrix_base.T)).as_matrix()

                    # Use the class attribute for the transformation
                    eef_pose_world_l = self.base_pose_world_l * eef_pose_base_l
                    pos_l = eef_pose_world_l.translation.tolist()
                    # Convert rotation matrix to XYZ Euler angles (roll-pitch-yaw)
                    euler_xyz_l = Rotation.from_matrix(eef_pose_world_l.rotation).as_euler("XYZ")

                    # print(f"base_pose_world_l: {self.base_pose_world_l}")
                    # print(f"R: {self.base_pose_world_l.rotation * eef_pose_base_l.rotation}")
                    # print(f"t: {self.base_pose_world_l.translation + self.base_pose_world_l.rotation @ eef_pose_base_l.translation}")

                    # print(f"eef_pose_world_l: {eef_pose_world_l}")
                    # print(f"pos_l: {pos_l}")
                    # print(f"euler_xyz_l: {euler_xyz_l}")

                    # --- FK for Right Arm ---
                    pinocchio.forwardKinematics(self.model, self.data, q_r)
                    pinocchio.updateFramePlacement(self.model, self.data, self.eef_frame_id)
                    eef_pose_base_r = self.data.oMf[self.eef_frame_id]
                    eef_pose_base_r.rotation = (Rotation.from_matrix(eef_pose_base_r.rotation) * Rotation.from_matrix(self.eef_matrix_base.T)).as_matrix()

                    eef_pose_world_r = self.base_pose_world_r * eef_pose_base_r
                    pos_r = eef_pose_world_r.translation.tolist()
                    # Convert rotation matrix to XYZ Euler angles (roll-pitch-yaw)
                    euler_xyz_r = Rotation.from_matrix(eef_pose_world_r.rotation).as_euler("XYZ")

                    # MODIFICATION 3: Combine all updates into a single, efficient call.
                    session.update(
                        CoordsMarker(key="handle_l", position=handle_pos_l, rotation=handle_euler_l)
                    )
                    session.update(
                        CoordsMarker(key="handle_r", position=handle_pos_r, rotation=handle_euler_r)
                    )
                    session.update(
                        Urdf(jointValues=dict(zip(joint_names, self.left_arm_joints)), key="robot_l")
                    )
                    session.update(
                        CoordsMarker(key="eef_l", position=pos_l, rotation=euler_xyz_l)
                    )
                    session.update(
                        Urdf(jointValues=dict(zip(joint_names, self.right_arm_joints)), key="robot_r")
                    )
                    session.update(
                        CoordsMarker(key="eef_r", position=pos_r, rotation=euler_xyz_r)
                    )

                except AssertionError:
                    print("Websocket missing while trying to update; skipping this frame.")
                except Exception as e:
                    print(f"An error occurred in the Vuer loop: {e}")

                await asyncio.sleep(0.02)

def main():
    controller = ControllerVuer()
    controller.run()

if __name__ == '__main__':
    main()