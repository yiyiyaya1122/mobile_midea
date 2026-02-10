#!/usr/bin/env python3
"""
Rebase end_pose to the first frame (HDF5 only).

Supports:
- single file -> single file
- directory (.hdf5 batch) -> directory
- left / right / both arms

Pose format:
  x y z qx qy qz qw
"""

import argparse
import os
import numpy as np
import h5py
from scipy.spatial.transform import Rotation


# -----------------------------------------------------------------------------
# Pose helpers
# -----------------------------------------------------------------------------
def pose_to_T(pose):
    p = pose[:3]
    q = pose[3:7]
    R = Rotation.from_quat(q).as_matrix()
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


def T_to_pose(T):
    p = T[:3, 3]
    q = Rotation.from_matrix(T[:3, :3]).as_quat()
    return np.concatenate([p, q])


def rebase_poses(poses):
    """
    poses: (N, 7)
    return: (N, 7)
    """
    T0 = pose_to_T(poses[0])
    T0_inv = np.linalg.inv(T0)

    out = []
    for pose in poses:
        T = pose_to_T(pose)
        T_rel = T0_inv @ T
        out.append(T_to_pose(T_rel))

    return np.asarray(out)


# -----------------------------------------------------------------------------
# HDF5 helpers (copy remaining, same style as previous script)
# -----------------------------------------------------------------------------
def write_hdf5(path, dataset, data, mode="a"):
    with h5py.File(path, mode) as f:
        if dataset in f:
            del f[dataset]
        f.create_dataset(dataset, data=data)


def copy_hdf5_remaining(src, dst, exclude):
    """
    Copy all datasets/groups except those in exclude
    """
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
# Per-file processing
# -----------------------------------------------------------------------------
def process_single_file(args, input_path, output_path):
    copy_remaining = not args.no_copy_remaining

    if copy_remaining and os.path.realpath(input_path) == os.path.realpath(output_path):
        copy_remaining = False

    # decide datasets
    if args.sides == "both":
        datasets = [
            "/state/end_pose/left",
            "/state/end_pose/right",
        ]
    else:
        datasets = [f"/state/end_pose/{args.sides}"]

    # copy remaining fields first
    if copy_remaining:
        copy_hdf5_remaining(input_path, output_path, datasets)

    # process target datasets
    with h5py.File(input_path, "r") as f:
        for ds in datasets:
            if ds not in f:
                continue

            poses = np.array(f[ds])
            if poses.ndim == 1:
                poses = poses[None, :]

            poses_rel = rebase_poses(poses)
            write_hdf5(output_path, ds, poses_rel)

    print(f"  ✔ {os.path.basename(input_path)}")


# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(
        description="Rebase end_pose to first frame (copy remaining fields)"
    )
    ap.add_argument("--input", required=True)
    ap.add_argument("--output", required=True)
    ap.add_argument(
        "--sides",
        choices=["left", "right", "both"],
        default="left",
    )
    ap.add_argument("--no-copy-remaining", action="store_true")
    args = ap.parse_args()

    # directory mode
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

    # single file
    process_single_file(args, args.input, args.output)


if __name__ == "__main__":
    main()
