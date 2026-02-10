#!/usr/bin/env python3
"""
Convert between joint_position and end_pose for YAM and ARX arms (HDF5 only).

Usage:
  # joint -> end_pose
  python joint_pose_convert.py --arm yam --mode joint2pose --input bag.hdf5 --output poses.hdf5 \\
      [--dataset-in /state/joint_position/left] [--dataset-out /state/end_pose/left]

  # end_pose -> joint
  python joint_pose_convert.py --arm yam --mode pose2joint --input bag.hdf5 --output joints.hdf5 \\
      [--dataset-in /state/end_pose/left] [--dataset-out /state/joint_position/left]

  # both arms in one run
  python joint_pose_convert.py --arm yam --mode joint2pose --input bag.hdf5 --output poses.hdf5 --sides both

  # Copy all other data from input to output (default). Use --no-copy-remaining to write only converted dataset(s).
  python joint_pose_convert.py --arm yam --mode joint2pose --input bag.hdf5 --output out.hdf5 --sides both

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
    """Build YAM model and return (model, data, eef_id, eef_pos, base2eef_mat).
    eef_pos and base2eef_mat are at neutral config, for norm-frame transform (match YamFun)."""
    import pinocchio as pin
    from scipy.spatial.transform import Rotation
    model = pin.buildModelFromUrdf(urdf_path)
    data = pin.Data(model)
    if not model.existFrame(YAM_EEF_FRAME):
        raise ValueError(f"Frame '{YAM_EEF_FRAME}' not found in URDF.")
    eef_id = model.getFrameId(YAM_EEF_FRAME)
    q_neutral = pin.neutral(model)
    pin.forwardKinematics(model, data, q_neutral)
    pin.updateFramePlacement(model, data, eef_id)
    eef_pose = data.oMf[eef_id]
    eef_pos = np.array(eef_pose.translation, copy=True)
    base2eef_mat = np.array(eef_pose.rotation, copy=True)
    return model, data, eef_id, eef_pos, base2eef_mat


def yam_joint_to_pose(q_row, model, data, eef_id, eef_pos, base2eef_mat):
    """Single row: 6 or 7 joint values (rad). Returns 7 pose in NORM frame (x y z qx qy qz qw), matching YamController."""
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
    # Norm frame: position = base_pos - eef_pos, orientation = R_base @ base2eef_mat.T (match computeEEFinNorm)
    p_norm = np.array(T.translation) - eef_pos
    R_norm = T.rotation @ base2eef_mat.T
    r = Rotation.from_matrix(R_norm)
    quat = r.as_quat()  # xyzw
    return np.concatenate([p_norm, quat])


def yam_pose_to_joint(pose_row, model, data, eef_id, eef_pos, base2eef_mat, q_init=None):
    """Single row: 7 pose in NORM frame (x y z qx qy qz qw). Returns nq joint angles (rad)."""
    import pinocchio as pin
    from scipy.spatial.transform import Rotation
    pose = np.asarray(pose_row, dtype=np.float64).ravel()
    if pose.size != 7:
        raise ValueError("Pose must have 7 elements (x y z qx qy qz qw)")
    p_norm, quat = pose[:3], pose[3:7]
    R_norm = Rotation.from_quat(quat).as_matrix()
    # Norm -> base for IK: p_base = p_norm + eef_pos, R_base = R_norm @ base2eef_mat (match computeIK)
    p_base = p_norm + eef_pos
    R_base = R_norm @ base2eef_mat
    oMf_des = pin.SE3(R_base, p_base)
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


def copy_hdf5_remaining(src_path, dst_path, exclude_dataset_paths):
    """Copy all from src HDF5 to dst, skipping datasets whose path is in exclude_dataset_paths (e.g. '/state/end_pose/left')."""
    import h5py
    exclude = {p.lstrip("/") for p in exclude_dataset_paths}  # normalize to no leading slash for comparison

    def path_str(parent_prefix, key):
        return (parent_prefix + "/" + key) if parent_prefix else key

    with h5py.File(src_path, "r") as f_in:
        with h5py.File(dst_path, "w") as f_out:
            def copy_recursive(parent_in, parent_out, prefix):
                for key in parent_in:
                    path = path_str(prefix, key)
                    if path in exclude:
                        continue
                    item = parent_in[key]
                    if isinstance(item, h5py.Dataset):
                        parent_in.copy(item, parent_out, key)
                    else:
                        grp = parent_out.create_group(key)
                        copy_recursive(item, grp, path)
            copy_recursive(f_in, f_out, "")


# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="YAM/ARX joint_position <-> end_pose conversion")
    ap.add_argument("--arm", choices=["yam", "arx"], required=True)
    ap.add_argument("--mode", choices=["joint2pose", "pose2joint"], required=True)
    ap.add_argument("--input", required=True, help="Input HDF5 file")
    ap.add_argument("--output", required=True, help="Output HDF5 file")
    ap.add_argument("--sides", choices=["left", "right", "both"], default="left",
                    help="Arm(s) to convert: left/right/both (conventional dataset names)")
    ap.add_argument("--dataset-in", default=None, help="HDF5 input dataset (e.g. /state/joint_position/left)")
    ap.add_argument("--dataset-out", default=None, help="HDF5 output dataset")
    ap.add_argument("--no-copy-remaining", action="store_true",
                    help="Do not copy other data from input HDF5; write only converted dataset(s)")
    args = ap.parse_args()
    copy_remaining = not args.no_copy_remaining
    if copy_remaining and os.path.realpath(args.input) == os.path.realpath(args.output):
        print("Warning: input and output are the same file; --copy-remaining disabled to avoid overwrite.", file=sys.stderr)
        copy_remaining = False

    def dataset_pair(side, mode):
        if mode == "joint2pose":
            return f"/state/joint_position/{side}", f"/state/end_pose/{side}"
        return f"/state/end_pose/{side}", f"/state/joint_position/{side}"

    if args.sides == "both":
        # Convert both arms in one run; conventional dataset names
        try:
            import h5py
        except ImportError:
            raise RuntimeError("h5py required for --sides both")
        with h5py.File(args.input, "r") as f_in:
            has_any = any(dataset_pair(s, args.mode)[0] in f_in for s in ("left", "right"))
        if not has_any:
            raise KeyError(f"No dataset /state/joint_position/left or /state/joint_position/right in {args.input}")
        exclude_ds = {dataset_pair(s, args.mode)[1] for s in ("left", "right")}
        if copy_remaining:
            copy_hdf5_remaining(args.input, args.output, exclude_ds)
            print(f"Copied remaining data from {args.input} to {args.output} (excluded {exclude_ds})")
        sides_done = 0
        yam_model = yam_data = yam_eef_id = None
        if args.arm == "yam":
            urdf = os.environ.get("YAM_URDF") or _yam_urdf_path()
            yam_model, yam_data, yam_eef_id, yam_eef_pos, yam_base2eef = _yam_fk_model(urdf)
        with h5py.File(args.input, "r") as f_in:
            for side in ("left", "right"):
                ds_in, ds_out = dataset_pair(side, args.mode)
                if ds_in not in f_in:
                    continue
                data_in = np.array(f_in[ds_in])
                if data_in.ndim == 1:
                    data_in = data_in.reshape(1, -1)
                if args.arm == "yam":
                    if args.mode == "joint2pose":
                        out_rows = [yam_joint_to_pose(row, yam_model, yam_data, yam_eef_id, yam_eef_pos, yam_base2eef) for row in data_in]
                    else:
                        q_prev = None
                        out_rows = []
                        for row in data_in:
                            q = yam_pose_to_joint(row, yam_model, yam_data, yam_eef_id, yam_eef_pos, yam_base2eef, q_init=q_prev)
                            q_prev = q
                            out_rows.append(q)
                else:
                    if args.mode == "joint2pose":
                        out_rows = [T_to_pose_xyzw(arx_fk(np.asarray(row).ravel()[:6])) for row in data_in]
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
                write_hdf5(args.output, ds_out, out, create_mode="a")
                print(f"Wrote dataset '{ds_out}' to {args.output} shape {out.shape}")
                sides_done += 1
        return

    # Single-side path
    default_in = f"/state/joint_position/{args.sides}" if args.mode == "joint2pose" else f"/state/end_pose/{args.sides}"
    default_out = f"/state/end_pose/{args.sides}" if args.mode == "joint2pose" else f"/state/joint_position/{args.sides}"
    ds_in = args.dataset_in or default_in
    ds_out = args.dataset_out or default_out
    if copy_remaining:
        copy_hdf5_remaining(args.input, args.output, {ds_out})
        print(f"Copied remaining data from {args.input} to {args.output} (excluded {ds_out})")
    data_in = read_hdf5(args.input, ds_in)
    if data_in.ndim == 1:
        data_in = data_in.reshape(1, -1)

    if args.arm == "yam":
        urdf = os.environ.get("YAM_URDF") or _yam_urdf_path()
        model, data, eef_id, eef_pos, base2eef_mat = _yam_fk_model(urdf)
        if args.mode == "joint2pose":
            out_rows = [yam_joint_to_pose(row, model, data, eef_id, eef_pos, base2eef_mat) for row in data_in]
        else:
            q_prev = None
            out_rows = []
            for row in data_in:
                q = yam_pose_to_joint(row, model, data, eef_id, eef_pos, base2eef_mat, q_init=q_prev)
                q_prev = q
                out_rows.append(q)
    else:
        if args.mode == "joint2pose":
            out_rows = [T_to_pose_xyzw(arx_fk(np.asarray(row).ravel()[:6])) for row in data_in]
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
    write_hdf5(args.output, ds_out, out, create_mode="a" if copy_remaining or os.path.isfile(args.output) else "w")
    print(f"Wrote dataset '{ds_out}' to {args.output} shape {out.shape}")


if __name__ == "__main__":
    main()
