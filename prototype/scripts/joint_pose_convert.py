#!/usr/bin/env python3
"""
Convert between joint_position and end_pose for YAM and ARX arms (HDF5 only).

Supports:
- single file -> single file
- directory (.hdf5 batch) -> directory

Pose format: x y z qx qy qz qw
Joint format: 6 values (rad)
"""

import argparse
import os
import sys
import numpy as np

# -----------------------------------------------------------------------------
# ARX kinematics
# -----------------------------------------------------------------------------
_scripts_dir = os.path.dirname(os.path.abspath(__file__))
if _scripts_dir not in sys.path:
    sys.path.insert(0, _scripts_dir)

from arx_kinematics import (
    arx_fk,
    arx_ik,
    T_to_pose_xyzw,
    pose_xyzw_to_T,
)

# -----------------------------------------------------------------------------
# YAM kinematics
# -----------------------------------------------------------------------------
def _yam_urdf_path():
    try:
        from ament_index_python.packages import get_package_share_directory
        d = get_package_share_directory("yam_description")
        return os.path.join(d, "urdf", "yam.urdf")
    except Exception:
        pass

    script_dir = os.path.dirname(os.path.abspath(__file__))
    repo = os.path.normpath(os.path.join(script_dir, ".."))
    urdf = os.path.join(repo, "src", "yam", "yam_description", "urdf", "yam.urdf")
    if os.path.isfile(urdf):
        return urdf

    raise FileNotFoundError("yam.urdf not found")

YAM_EEF_FRAME = "link_6"


def _yam_fk_model(urdf_path):
    import pinocchio as pin
    model = pin.buildModelFromUrdf(urdf_path)
    data = pin.Data(model)
    if not model.existFrame(YAM_EEF_FRAME):
        raise ValueError(f"Frame '{YAM_EEF_FRAME}' not found in URDF")
    eef_id = model.getFrameId(YAM_EEF_FRAME)

    q0 = pin.neutral(model)
    pin.forwardKinematics(model, data, q0)
    pin.updateFramePlacement(model, data, eef_id)
    pose = data.oMf[eef_id]

    return (
        model,
        data,
        eef_id,
        np.array(pose.translation),
        np.array(pose.rotation),
    )


def yam_joint_to_pose(q_row, model, data, eef_id, eef_pos, base2eef):
    import pinocchio as pin
    from scipy.spatial.transform import Rotation

    q = np.asarray(q_row, dtype=np.float64).ravel()[: model.nq]
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacement(model, data, eef_id)

    T = data.oMf[eef_id]
    p = np.array(T.translation) - eef_pos
    R = T.rotation @ base2eef.T
    quat = Rotation.from_matrix(R).as_quat()

    return np.concatenate([p, quat])


def yam_pose_to_joint(pose_row, model, data, eef_id, eef_pos, base2eef, q_init=None):
    import pinocchio as pin
    from scipy.spatial.transform import Rotation

    pose = np.asarray(pose_row, dtype=np.float64).ravel()
    p_norm, quat = pose[:3], pose[3:7]
    R_norm = Rotation.from_quat(quat).as_matrix()

    p = p_norm + eef_pos
    R = R_norm @ base2eef
    target = pin.SE3(R, p)

    q = pin.neutral(model) if q_init is None else q_init.copy()

    for _ in range(80):
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacement(model, data, eef_id)
        err = pin.log(data.oMf[eef_id].inverse() * target).vector
        if np.linalg.norm(err) < 1e-5:
            break
        J = pin.computeFrameJacobian(
            model, data, q, eef_id, pin.LOCAL_WORLD_ALIGNED
        )[:, : model.nq]
        dq = np.linalg.lstsq(J.T, err, rcond=None)[0]
        q += dq

    return q


# -----------------------------------------------------------------------------
# HDF5 helpers
# -----------------------------------------------------------------------------
def read_hdf5(path, dataset):
    import h5py

    with h5py.File(path, "r") as f:
        if dataset not in f:
            raise KeyError(f"{dataset} not in {path}")
        return np.array(f[dataset])


def write_hdf5(path, dataset, data, mode="a"):
    import h5py

    with h5py.File(path, mode) as f:
        if dataset in f:
            del f[dataset]
        f.create_dataset(dataset, data=data)


def copy_hdf5_remaining(src, dst, exclude):
    import h5py

    exclude = {p.lstrip("/") for p in exclude}

    def walk(src_grp, dst_grp, prefix=""):
        for k in src_grp:
            p = f"{prefix}/{k}" if prefix else k
            if p in exclude:
                continue
            item = src_grp[k]
            if isinstance(item, h5py.Dataset):
                src_grp.copy(item, dst_grp, k)
            else:
                g = dst_grp.create_group(k)
                walk(item, g, p)

    with h5py.File(src, "r") as f_in, h5py.File(dst, "w") as f_out:
        walk(f_in, f_out)


# -----------------------------------------------------------------------------
# Core per-file processing
# -----------------------------------------------------------------------------
def process_single_file(args, input_path, output_path):
    copy_remaining = not args.no_copy_remaining

    if copy_remaining and os.path.realpath(input_path) == os.path.realpath(output_path):
        copy_remaining = False

    def ds_pair(side):
        if args.mode == "joint2pose":
            return f"/state/joint_position/{side}", f"/state/end_pose/{side}"
        return f"/state/end_pose/{side}", f"/state/joint_position/{side}"

    if args.sides == "both":
        sides = ["left", "right"]
    else:
        sides = [args.sides]

    if copy_remaining:
        exclude = {ds_pair(s)[1] for s in sides}
        copy_hdf5_remaining(input_path, output_path, exclude)

    if args.arm == "yam":
        urdf = os.environ.get("YAM_URDF") or _yam_urdf_path()
        model, data, eef_id, eef_pos, base2eef = _yam_fk_model(urdf)

    import h5py
    with h5py.File(input_path, "r") as f:
        for side in sides:
            ds_in, ds_out = ds_pair(side)
            if ds_in not in f:
                continue

            data_in = np.array(f[ds_in])
            if data_in.ndim == 1:
                data_in = data_in[None, :]

            out = []
            q_prev = None
            for row in data_in:
                if args.arm == "yam":
                    if args.mode == "joint2pose":
                        v = yam_joint_to_pose(
                            row, model, data, eef_id, eef_pos, base2eef
                        )
                    else:
                        v = yam_pose_to_joint(
                            row,
                            model,
                            data,
                            eef_id,
                            eef_pos,
                            base2eef,
                            q_init=q_prev,
                        )
                else:
                    if args.mode == "joint2pose":
                        v = T_to_pose_xyzw(arx_fk(row[:6]))
                    else:
                        T = pose_xyzw_to_T(row[:7])
                        v = arx_ik(T, q_init=q_prev)
                        if v is None:
                            v = q_prev if q_prev is not None else np.zeros(6)
                q_prev = v
                out.append(v)

            write_hdf5(output_path, ds_out, np.asarray(out))


# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", required=True, choices=["yam", "arx"])
    ap.add_argument("--mode", required=True, choices=["joint2pose", "pose2joint"])
    ap.add_argument("--input", required=True)
    ap.add_argument("--output", required=True)
    ap.add_argument("--sides", default="left", choices=["left", "right", "both"])
    ap.add_argument("--no-copy-remaining", action="store_true")
    args = ap.parse_args()

    # Directory mode
    if os.path.isdir(args.input):
        os.makedirs(args.output, exist_ok=True)
        files = sorted(f for f in os.listdir(args.input) if f.endswith(".hdf5"))
        if not files:
            raise RuntimeError("No .hdf5 files found")

        for f in files:
            print(f"[+] {f}")
            process_single_file(
                args,
                os.path.join(args.input, f),
                os.path.join(args.output, f),
            )
        return

    # Single file
    process_single_file(args, args.input, args.output)


if __name__ == "__main__":
    main()
