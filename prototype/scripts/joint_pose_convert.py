#!/usr/bin/env python3
"""
Convert between joint_position and end_pose for YAM and ARX arms.
Supports HDF5 (e.g. from data_collector) and CSV.

Usage:
  # HDF5: joint -> end_pose
  python joint_pose_convert.py --arm yam --mode joint2pose --input bag.hdf5 --output poses.hdf5 \\
      [--dataset-in /state/joint_position/left] [--dataset-out /state/end_pose/left]

  # HDF5: end_pose -> joint
  python joint_pose_convert.py --arm yam --mode pose2joint --input bag.hdf5 --output joints.hdf5 \\
      [--dataset-in /state/end_pose/left] [--dataset-out /state/joint_position/left]

  # CSV
  python joint_pose_convert.py --arm arx --mode joint2pose --input joints.csv --output poses.csv [--format csv]

Pose format: x y z qx qy qz qw (7 values), matching collect_config end_pose.
Joint format: 6 values (rad) for FK/IK; input may have 7 (first 6 used for YAM).
"""

import argparse
import os
import sys

import numpy as np

# -----------------------------------------------------------------------------
# ARX kinematics (no ROS dependency)
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
# YAM kinematics (Pinocchio + URDF)
# -----------------------------------------------------------------------------
def _yam_urdf_path():
    """Resolve yam.urdf: ROS share dir or repo-relative."""
    try:
        from ament_index_python.packages import get_package_share_directory
        d = get_package_share_directory("yam_description")
        return os.path.join(d, "urdf", "yam.urdf")
    except Exception:
        pass
    # Repo-relative: prototype/scripts -> prototype/src/yam/yam_description/urdf
    script_dir = os.path.dirname(os.path.abspath(__file__))
    repo = os.path.normpath(os.path.join(script_dir, ".."))
    urdf = os.path.join(repo, "src", "yam", "yam_description", "urdf", "yam.urdf")
    if os.path.isfile(urdf):
        return urdf
    raise FileNotFoundError(
        "yam.urdf not found. Source ROS workspace or set YAM_URDF path."
    )


YAM_EEF_FRAME = "link_6"


def _yam_fk_model(urdf_path):
    import pinocchio as pin
    model = pin.buildModelFromUrdf(urdf_path)
    data = pin.Data(model)
    if not model.existFrame(YAM_EEF_FRAME):
        raise ValueError(f"Frame '{YAM_EEF_FRAME}' not found in URDF.")
    eef_id = model.getFrameId(YAM_EEF_FRAME)
    return model, data, eef_id


def yam_joint_to_pose(q_row, model, data, eef_id):
    """Single row: 6 or 7 joint values (rad). Returns 7 pose (x y z qx qy qz qw)."""
    import pinocchio as pin
    from scipy.spatial.transform import Rotation
    nq = model.nq
    q = np.asarray(q_row, dtype=np.float64).ravel()
    if q.size >= nq:
        q = q[:nq]
    else:
        raise ValueError(f"YAM expects at least {nq} joint angles")
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacement(model, data, eef_id)
    T = data.oMf[eef_id]
    R = T.rotation
    p = T.translation
    r = Rotation.from_matrix(R)
    quat = r.as_quat()  # xyzw
    return np.concatenate([p, quat])


def yam_pose_to_joint(pose_row, model, data, eef_id, q_init=None):
    """Single row: 7 pose (x y z qx qy qz qw). Returns nq joint angles (rad)."""
    import pinocchio as pin
    pose = np.asarray(pose_row, dtype=np.float64).ravel()
    if pose.size != 7:
        raise ValueError("Pose must have 7 elements (x y z qx qy qz qw)")
    quat, p = pose[3:7], pose[:3]
    from scipy.spatial.transform import Rotation
    Rmat = Rotation.from_quat(quat).as_matrix()
    oMf_des = pin.SE3(Rmat, p)
    nq = model.nq
    if q_init is None:
        q_init = pin.neutral(model)
    q = np.asarray(q_init, dtype=np.float64).ravel()[:nq].copy()
    for _ in range(80):
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacement(model, data, eef_id)
        oMf = data.oMf[eef_id]
        err = pin.log(oMf.inverse() * oMf_des).vector
        if np.linalg.norm(err) < 1e-5:
            return q
        J = pin.computeFrameJacobian(model, data, q, eef_id, pin.LOCAL_WORLD_ALIGNED)
        J = J[:, :nq]
        dq = np.linalg.lstsq(J.T, err, rcond=None)[0]
        q = q + dq
    return q


# -----------------------------------------------------------------------------
# I/O
# -----------------------------------------------------------------------------
def read_hdf5(path, dataset):
    import h5py
    with h5py.File(path, "r") as f:
        if dataset not in f:
            raise KeyError(f"Dataset '{dataset}' not in {path}")
        return np.array(f[dataset])


def write_hdf5(path, dataset, data, create_mode="a"):
    import h5py
    with h5py.File(path, create_mode) as f:
        if dataset in f:
            del f[dataset]
        f.create_dataset(dataset, data=np.asarray(data), dtype=data.dtype)


def read_csv(path, dtype=np.float64):
    return np.loadtxt(path, delimiter=",", dtype=dtype)


def write_csv(path, data):
    np.savetxt(path, np.asarray(data), delimiter=",")


def infer_format(path):
    p = path.lower()
    if p.endswith(".hdf5") or p.endswith(".h5"):
        return "hdf5"
    if p.endswith(".csv"):
        return "csv"
    return "hdf5"


# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="YAM/ARX joint_position <-> end_pose conversion")
    ap.add_argument("--arm", choices=["yam", "arx"], required=True)
    ap.add_argument("--mode", choices=["joint2pose", "pose2joint"], required=True)
    ap.add_argument("--input", required=True, help="Input file (HDF5 or CSV)")
    ap.add_argument("--output", required=True, help="Output file")
    ap.add_argument("--format", choices=["hdf5", "csv"], default=None, help="Default: infer from extension")
    ap.add_argument("--dataset-in", default=None, help="HDF5 input dataset (e.g. /state/joint_position/left)")
    ap.add_argument("--dataset-out", default=None, help="HDF5 output dataset")
    args = ap.parse_args()

    fmt = args.format or infer_format(args.input)
    if fmt != infer_format(args.output):
        print("Warning: input and output format differ.", file=sys.stderr)

    # Load
    if fmt == "hdf5":
        ds_in = args.dataset_in or "/state/joint_position/left" if args.mode == "joint2pose" else "/state/end_pose/left"
        data_in = read_hdf5(args.input, ds_in)
    else:
        data_in = read_csv(args.input)
    if data_in.ndim == 1:
        data_in = data_in.reshape(1, -1)

    # Convert
    if args.arm == "yam":
        urdf = os.environ.get("YAM_URDF") or _yam_urdf_path()
        model, data, eef_id = _yam_fk_model(urdf)
        if args.mode == "joint2pose":
            out_rows = [yam_joint_to_pose(row, model, data, eef_id) for row in data_in]
        else:
            q_prev = None
            out_rows = []
            for row in data_in:
                q = yam_pose_to_joint(row, model, data, eef_id, q_init=q_prev)
                q_prev = q
                out_rows.append(q)
    else:
        if args.mode == "joint2pose":
            out_rows = []
            for row in data_in:
                q = np.asarray(row).ravel()[:6]
                T = arx_fk(q)
                out_rows.append(T_to_pose_xyzw(T))
        else:
            q_prev = None
            out_rows = []
            for row in data_in:
                T = pose_xyzw_to_T(np.asarray(row).ravel()[:7])
                q = arx_ik(T, q_init=q_prev)
                if q is None:
                    q = q_prev if q_prev is not None else np.zeros(6)
                q_prev = q
                out_rows.append(q)

    out = np.array(out_rows)

    # Save
    if fmt == "hdf5":
        ds_out = args.dataset_out or "/state/end_pose/left" if args.mode == "joint2pose" else "/state/joint_position/left"
        write_hdf5(args.output, ds_out, out, create_mode="a" if os.path.isfile(args.output) else "w")
        print(f"Wrote dataset '{ds_out}' to {args.output} shape {out.shape}")
    else:
        write_csv(args.output, out)
        print(f"Wrote {args.output} shape {out.shape}")


if __name__ == "__main__":
    main()
