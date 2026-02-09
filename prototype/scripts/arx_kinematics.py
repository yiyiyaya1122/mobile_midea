"""
ARX R5 forward/inverse kinematics (space-form screws).
Matches C++ ARX_R5_fun: M_, Slist_, FKinSpace, iterative IK with JacobianSpace.
"""

import numpy as np

# Home configuration (4x4)
ARX_M = np.array([
    [1, 0, 0, 0.187],
    [0, 1, 0, 0],
    [0, 0, 1, 0.163],
    [0, 0, 0, 1],
], dtype=np.float64)

# Space screw axes (6x6: each column is one joint axis [omega; v])
ARX_SLIST = np.array([
    [0, 0, 0, 0, 0, 1],
    [0, 1, -1, -1, 0, 0],
    [1, 0, 0, 0, -1, 0],
    [0, -0.104, 0.104, 0.163, 0, 0],
    [0, 0, 0, 0, 0.075, 0.163],
    [0, 0.02, 0.244, -0.001, 0, 0],
], dtype=np.float64)

# Joint limits (rad) for IK
ARX_Q_MIN = np.array([-2.618, 0, 0, -1.745, -1.22, -2.0944])
ARX_Q_MAX = np.array([2.618, np.pi, np.pi, 1.745, 1.22, 2.0944])

_EPS = 1e-9


def _vec_to_so3(omg):
    """VecToso3: 3-vector -> 3x3 skew-symmetric."""
    return np.array([
        [0, -omg[2], omg[1]],
        [omg[2], 0, -omg[0]],
        [-omg[1], omg[0], 0],
    ], dtype=np.float64)


def _so3_to_vec(so3mat):
    """so3ToVec: 3x3 skew -> 3-vector."""
    return np.array([so3mat[2, 1], so3mat[0, 2], so3mat[1, 0]], dtype=np.float64)


def _axis_ang3(expc3):
    """AxisAng3: (axis, theta) from exponential coordinates."""
    theta = np.linalg.norm(expc3)
    if theta < _EPS:
        return np.array([0, 0, 0, 0], dtype=np.float64)
    return np.concatenate([expc3 / theta, [theta]])


def _matrix_exp3(so3mat):
    """MatrixExp3: so3 -> SO3 (Rodrigues)."""
    omgtheta = _so3_to_vec(so3mat)
    if np.linalg.norm(omgtheta) < _EPS:
        return np.eye(3)
    theta = _axis_ang3(omgtheta)[3]
    omgmat = so3mat / theta
    return np.eye(3) + np.sin(theta) * omgmat + (1 - np.cos(theta)) * (omgmat @ omgmat)


def _matrix_log3(R):
    """MatrixLog3: SO3 -> so3."""
    if np.linalg.norm(R - np.eye(3)) < _EPS:
        return np.zeros((3, 3))
    ac = np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))
    if np.abs(ac) < _EPS:
        return np.zeros((3, 3))
    return ac / (2 * np.sin(ac)) * (R - R.T)


def _vec_to_se3(V):
    """VecTose3: 6-vector [omega; v] -> 4x4 se3 matrix."""
    exp = V[:3]
    linear = V[3:6]
    m = np.eye(4)
    m[:3, :3] = _vec_to_so3(exp)
    m[:3, 3] = linear
    return m


def _matrix_exp6(se3mat):
    """MatrixExp6: se3 -> SE3."""
    omgtheta = _so3_to_vec(se3mat[:3, :3])
    if np.linalg.norm(omgtheta) < _EPS:
        m = np.eye(4)
        m[:3, 3] = se3mat[:3, 3]
        return m
    theta = _axis_ang3(omgtheta)[3]
    omgmat = se3mat[:3, :3] / theta
    linear = se3mat[:3, 3]
    exp_expand = np.eye(3) * theta + (1 - np.cos(theta)) * omgmat + (theta - np.sin(theta)) * (omgmat @ omgmat)
    G_theta_v = exp_expand @ linear / theta
    m = np.eye(4)
    m[:3, :3] = _matrix_exp3(se3mat[:3, :3])
    m[:3, 3] = G_theta_v
    return m


def _trans_to_rp(T):
    """TransToRp: 4x4 T -> (R, p)."""
    return T[:3, :3].copy(), T[:3, 3].copy()


def _rp_to_trans(R, p):
    """RpToTrans: (R, p) -> 4x4 T."""
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


def _matrix_log6(T):
    """MatrixLog6: SE3 -> se3 matrix (matches C++ modern_robotics)."""
    R, p = _trans_to_rp(T)
    omgmat = _matrix_log3(R)
    m = np.zeros((4, 4))
    if np.linalg.norm(omgmat) < _EPS:
        m[:3, 3] = p
        return m
    theta = np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))
    log_expand1 = np.eye(3) - omgmat / 2
    log_expand2 = (1 / theta - 1 / (2 * np.tan(theta / 2))) * (omgmat @ omgmat) / theta
    log_expand = log_expand1 + log_expand2
    m[:3, :3] = omgmat
    m[:3, 3] = log_expand @ p
    return m


def _se3_to_vec(se3mat):
    """se3ToVec: 4x4 se3 -> 6-vector [omega; v]."""
    return np.array([
        se3mat[2, 1], se3mat[0, 2], se3mat[1, 0],
        se3mat[0, 3], se3mat[1, 3], se3mat[2, 3],
    ], dtype=np.float64)


def _adjoint(T):
    """Adjoint: 4x4 T -> 6x6 Ad(T)."""
    R, p = _trans_to_rp(T)
    ad = np.zeros((6, 6))
    ad[:3, :3] = R
    ad[3:, 3:] = R
    ad[3:, :3] = _vec_to_so3(p) @ R
    return ad


def _trans_inv(T):
    """TransInv: inverse of 4x4 T."""
    R, p = _trans_to_rp(T)
    return _rp_to_trans(R.T, -R.T @ p)


def fkin_space(M, Slist, theta_list):
    """FKinSpace: T = exp(S1*t1)*...*exp(S6*t6)*M."""
    T = M.copy()
    for i in range(len(theta_list) - 1, -1, -1):
        T = _matrix_exp6(_vec_to_se3(Slist[:, i] * theta_list[i])) @ T
    return T


def jacobian_space(Slist, theta_list):
    """JacobianSpace: 6xn spatial Jacobian."""
    n = theta_list.size
    Js = np.zeros((6, n))
    Js[:, 0] = Slist[:, 0]
    T = np.eye(4)
    for i in range(1, n):
        T = T @ _matrix_exp6(_vec_to_se3(Slist[:, i - 1] * theta_list[i - 1]))
        Js[:, i] = _adjoint(T) @ Slist[:, i]
    return Js


def arx_fk(theta_list):
    """ARX R5 forward kinematics. theta_list: (6,) joint angles rad. Returns 4x4 T."""
    theta_list = np.asarray(theta_list, dtype=np.float64).ravel()
    if theta_list.size != 6:
        raise ValueError("ARX expects 6 joint angles")
    return fkin_space(ARX_M, ARX_SLIST, theta_list)


def _clamp_joints(q):
    """Clamp and wrap to [q_min, q_max] (simplified wrap)."""
    q = np.asarray(q)
    for i in range(6):
        if q[i] < ARX_Q_MIN[i]:
            q[i] = np.clip(q[i], ARX_Q_MIN[i], ARX_Q_MAX[i])
        if q[i] > ARX_Q_MAX[i]:
            q[i] = np.clip(q[i], ARX_Q_MIN[i], ARX_Q_MAX[i])
    return q


def arx_ik(T_desired, q_init=None, eomg=5e-4, ev=5e-5, max_iter=40):
    """
    ARX R5 iterative IK (space form). T_desired: 4x4 SE3.
    Returns joint angles (6,) or None on failure.
    """
    if q_init is None:
        q_init = np.zeros(6)
    thetalist = np.asarray(q_init, dtype=np.float64).ravel()
    if thetalist.size != 6:
        thetalist = np.zeros(6)

    for _ in range(max_iter):
        Tfk = arx_fk(thetalist)
        Tdiff = _trans_inv(Tfk) @ T_desired
        Vs = _adjoint(Tfk) @ _se3_to_vec(_matrix_log6(Tdiff))
        angular = Vs[:3]
        linear = Vs[3:6]
        if np.linalg.norm(angular) <= eomg and np.linalg.norm(linear) <= ev:
            return _clamp_joints(thetalist)
        Js = jacobian_space(ARX_SLIST, thetalist)
        try:
            dq = np.linalg.lstsq(Js.T, Vs, rcond=None)[0]
        except np.linalg.LinAlgError:
            return None
        thetalist = thetalist + dq
        thetalist = _clamp_joints(thetalist)

    return _clamp_joints(thetalist)


def T_to_pose_xyzw(T):
    """4x4 SE3 -> pose as (x, y, z, qx, qy, qz, qw)."""
    from scipy.spatial.transform import Rotation
    R, p = _trans_to_rp(T)
    r = Rotation.from_matrix(R)
    q = r.as_quat()  # xyzw
    return np.concatenate([p, q])


def pose_xyzw_to_T(pose):
    """pose (x,y,z, qx,qy,qz,qw) -> 4x4 SE3."""
    from scipy.spatial.transform import Rotation
    p = np.asarray(pose[:3], dtype=np.float64)
    q = np.asarray(pose[3:7], dtype=np.float64)
    R = Rotation.from_quat(q).as_matrix()
    return _rp_to_trans(R, p)
