import numpy as np
from vuer import Vuer, VuerSession
from vuer.schemas import Hands, DefaultScene
import asyncio
from multiprocessing import Array, Value, Manager
from msgpack import ExtType
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

def load_cert(package_name='quest3s_vr'):
    cert_path = Path(get_package_share_directory(package_name)) / 'config/cert.pem'
    key_path = Path(get_package_share_directory(package_name)) / 'config/key.pem'

    return cert_path, key_path

cert_path, key_path = load_cert()

class HandVuer:
    def __init__(
            self,
            host='0.0.0.0',
            port=8012,
            cert=cert_path,
            key=key_path,
            queries=dict(grid=False),
            ws_protocol="vuer",
    ):
        self.app = Vuer(
            host=host,
            port=port,
            cert=cert,
            key=key,
            queries=queries,
            ws_protocol=ws_protocol,
        )

        # Shared matrices
        self.left_hand_shared = Array('f', 25 * 16, lock=True)
        self.right_hand_shared = Array('f', 25 * 16, lock=True)
        self.head_matrix_shared = Array('d', 16, lock=True)  # Keep head matrix as double

        # Shared hand states
        manager = Manager()
        self.left_hand_state = manager.dict()
        self.right_hand_state = manager.dict()

        # Initialize default hand state values
        for hand_state in [self.left_hand_state, self.right_hand_state]:
            hand_state.update({
                'pinch': False,
                'squeeze': False,
                'tap': False,
                'pinchValue': 0.0,
                'squeezeValue': 0.0,
                'tapValue': 0.0,
            })

        self.app.add_handler("HAND_MOVE", self.handle_hands_move)
        self.app.add_handler("CAMERA_MOVE", self.handle_camera_move)

    async def handle_hands_move(self, event, session):
        data = event.value
        try:
            # Update hand matrices
            if 'left' in data and not isinstance(data['left'], ExtType):
                with self.left_hand_shared.get_lock():
                    self.left_hand_shared[:] = data['left']

            if 'right' in data and not isinstance(data['right'], ExtType):
                with self.right_hand_shared.get_lock():
                    self.right_hand_shared[:] = data['right']

            # Update hand states
            if 'leftState' in data:
                for key in data['leftState']:
                    self.left_hand_state[key] = data['leftState'][key]

            if 'rightState' in data:
                for key in data['rightState']:
                    self.right_hand_state[key] = data['rightState'][key]

        except Exception as e:
            print(f"Hands move error: {e}")

    async def handle_camera_move(self, event, session):
        data = event.value
        try:
            if 'camera' in data and 'matrix' in data['camera']:
                matrix = data['camera']['matrix']
                if not isinstance(matrix, ExtType):
                    with self.head_matrix_shared.get_lock():
                        self.head_matrix_shared[:] = matrix
        except Exception as e:
            print(f"Camera move error: {e}")

    # Hand matrix properties
    @property
    def left_hand_matrices(self):
        """Returns list of 25 4x4 matrices in column-major order"""
        with self.left_hand_shared.get_lock():
            arr = np.frombuffer(self.left_hand_shared.get_obj(), dtype=np.float32)
            return [arr[i * 16:(i + 1) * 16].reshape(4, 4, order='F') for i in range(25)]

    @property
    def right_hand_matrices(self):
        """Returns list of 25 4x4 matrices in column-major order"""
        with self.right_hand_shared.get_lock():
            arr = np.frombuffer(self.right_hand_shared.get_obj(), dtype=np.float32)
            return [arr[i * 16:(i + 1) * 16].reshape(4, 4, order='F') for i in range(25)]

    @property
    def head_matrix(self):
        with self.head_matrix_shared.get_lock():
            return np.array(self.head_matrix_shared[:]).reshape(4, 4, order="F")

    # Hand state properties
    @property
    def left_hand_state_dict(self):
        return dict(self.left_hand_state)

    @property
    def right_hand_state_dict(self):
        return dict(self.right_hand_state)

    def run(self):
        @self.app.spawn(start=True)
        async def sense_loop(session: VuerSession):
            session.set @ DefaultScene(grid=False)

            session.upsert(
                Hands(
                    fps=60,
                    stream=True,
                    key="hands",
                    # hideLeft=False,       # hides the hand, but still streams the data.
                    # hideRight=False,      # hides the hand, but still streams the data.
                    # disableLeft=False,    # disables the left data stream, also hides the hand.
                    # disableRight=False,   # disables the right data stream, also hides the hand.
                ),
                to="bgChildren",
            )

            while True:
                # print(f"left_hand_matrices: {self.left_hand_matrices}")
                # print(f"left_hand_state_dict: {self.left_hand_state_dict}")
                # print(f"head_matrix: {self.head_matrix}")
                await asyncio.sleep(1)


def main():
    hand = HandVuer()
    hand.run()


if __name__ == '__main__':
    main()