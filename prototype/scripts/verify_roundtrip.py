#!/usr/bin/env python3
"""
Round-trip verification for joint_pose_convert: load joint from HDF5,
compute joint -> pose -> joint in memory, compare with original joints.
Uses current-frame joint as IK seed to test FK/IK inverse (not trajectory).
Usage: python3 verify_roundtrip.py <path_to_collection.hdf5> [--arm yam] [--input-unit rad|deg]
"""

import argparse
import os
import sys
import warnings

import numpy as np

warnings.filterwarnings("ignore", message=".*product.*deprecated.*", category=DeprecationWarning)

# Same conversion logic as joint_pose_convert
_scripts_dir = os.path.dirname(os.path.abspath(__file__))
if _scripts_dir not in sys.path:
    sys.path.insert(0, _scripts_dir)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("hdf5_path", help="Collection HDF5 with /state/joint_position/left and /right")
    ap.add_argument("--arm", choices=["yam", "arx"], default="yam")
    ap.add_argument("--input-unit", choices=["rad", "deg"], default="rad",
                    help="Unit of stored joint_position (controller typically publishes rad)")
    args = ap.parse_args()

    try:
        import h5py
    except ImportError:
        print("h5py required: pip install h5py", file=sys.stderr)
        sys.exit(1)

    with h5py.File(args.hdf5_path, "r") as f:
        if "/state/joint_position/left" not in f and "/state/joint_position/right" not in f:
            print("No /state/joint_position/left or /right in", args.hdf5_path, file=sys.stderr)
            sys.exit(1)
        sides = []
        if "/state/joint_position/left" in f:
            sides.append(("left", np.array(f["/state/joint_position/left"])))
        if "/state/joint_position/right" in f:
            sides.append(("right", np.array(f["/state/joint_position/right"])))

    deg2rad = np.deg2rad(1.0) if args.input_unit == "deg" else 1.0
    nq = 6  # compare first 6 joints
    all_ok = True

    if args.arm == "yam":
        import joint_pose_convert as jpc
        urdf = os.environ.get("YAM_URDF") or jpc._yam_urdf_path()
        model, data, eef_id, eef_pos, base2eef_mat = jpc._yam_fk_model(urdf)
        for side_name, joints in sides:
            if joints.ndim == 1:
                joints = joints.reshape(1, -1)
            q_orig = np.asarray(joints[:, :nq], dtype=np.float64) * deg2rad
            q_rt = np.zeros_like(q_orig)
            for i, row in enumerate(joints):
                row_rad = np.asarray(row, dtype=np.float64) * deg2rad
                pose = jpc.yam_joint_to_pose(row_rad, model, data, eef_id, eef_pos, base2eef_mat)
                q_rt[i] = jpc.yam_pose_to_joint(pose, model, data, eef_id, eef_pos, base2eef_mat, q_init=row_rad[: model.nq])
            err = np.abs(q_orig - q_rt)
            max_err = err.max()
            mean_err = err.mean()
            rmse = np.sqrt((err ** 2).mean())
            ok = max_err < 1e-3
            all_ok = all_ok and ok
            print(f"[{side_name}] samples={len(q_orig)}  max_err={max_err:.6f} rad  mean_err={mean_err:.6f} rad  RMSE={rmse:.6f}  {'PASS' if ok else 'FAIL'}")
    else:
        from arx_kinematics import arx_fk, arx_ik, T_to_pose_xyzw, pose_xyzw_to_T
        for side_name, joints in sides:
            if joints.ndim == 1:
                joints = joints.reshape(1, -1)
            q_orig = np.asarray(joints[:, :nq], dtype=np.float64) * deg2rad
            q_rt = np.zeros_like(q_orig)
            for i, row in enumerate(joints):
                q = np.asarray(row).ravel()[:nq] * deg2rad
                T = arx_fk(q)
                pose = T_to_pose_xyzw(T)
                q_back = arx_ik(pose_xyzw_to_T(pose), q_init=q)
                q_rt[i] = q_back if q_back is not None else np.zeros(nq)
            err = np.abs(q_orig - q_rt)
            max_err = err.max()
            mean_err = err.mean()
            rmse = np.sqrt((err ** 2).mean())
            ok = max_err < 1e-3
            all_ok = all_ok and ok
            print(f"[{side_name}] samples={len(q_orig)}  max_err={max_err:.6f} rad  mean_err={mean_err:.6f} rad  RMSE={rmse:.6f}  {'PASS' if ok else 'FAIL'}")

    print("Round-trip verification:", "PASS" if all_ok else "FAIL (max error >= 1e-3 rad)")
    sys.exit(0 if all_ok else 1)


if __name__ == "__main__":
    main()
