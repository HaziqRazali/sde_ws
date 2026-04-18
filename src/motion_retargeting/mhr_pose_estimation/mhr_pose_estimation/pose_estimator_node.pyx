#!/usr/bin/env python3
"""
Pose Estimator Node (Python 3.x)

Pipeline:
  RTMPose (PoseTracker) -> 2D keypoints [21, 2]
    -> kpts2smpl MLP       -> SMPL-X rot6d [22, 6]
    -> optional smoother   -> smoothed rot6d [22, 6]
    -> SMPL-X forward pass -> mesh vertices -> PointCloud2
    -> retargeting         -> Booster T1 JointState /ik/joint_states

Retarget modes (--retarget):
  naive   - direct XYZ Euler decomposition (default, fast, zero crosstalk)
  gmr     - SMPL-X FK + GMR mink IK (requires /root/GMR mount, slower but
            uses full kinematic chain; best for whole-body motions)

Smoother options (--smoother):
  none     - pass-through
  ema      - exponential moving average  (--ema_alpha,         default 0.3)
  oneeuro  - 1-Euro filter               (--oneeuro_min_cutoff default 1.0,
                                          --oneeuro_beta        default 0.007,
                                          --oneeuro_dcutoff     default 1.0)

Keypoint config : upper_body_with_hips (21 keypoints from COCO Wholebody 133)
Body model      : SMPL-X neutral, no PCA hands, flat_hand_mean=True

All options are plain CLI flags — no --ros-args needed:
  ros2 run mhr_pose_estimation pose_estimator \\
    -w models/all_epoch_0031_best_0031_state_dict.pt \\
    -c models/all_epoch_0031_best_0031_config.yaml \\
    --smoother oneeuro \\
    --retarget gmr
"""

import os
import re
import sys
import yaml

import rclpy
from rclpy.node import Node
import struct
from sensor_msgs.msg import JointState, PointCloud2, PointField

import torch
import torch.nn as nn
import numpy as np
from scipy.spatial.transform import Rotation
import cv2
import smplx as smplx_lib
from mmdeploy_runtime import PoseTracker

# ─── Filtered keypoint config: "upper_body_with_hips" ─────────────────────────
# 21 keypoints selected from COCO Wholebody 133-kpt set.
# Positions in this filtered array are used throughout the node.
#
# Pos | COCO ID | Name
#   0 |    0    | nose
#   1 |    1    | left_eye
#   2 |    2    | right_eye
#   3 |    3    | left_ear
#   4 |    4    | right_ear
#   5 |    5    | left_shoulder
#   6 |    6    | right_shoulder
#   7 |    7    | left_elbow
#   8 |    8    | right_elbow
#   9 |    9    | left_wrist
#  10 |   10    | right_wrist
#  11 |   11    | left_hip
#  12 |   12    | right_hip
#  13 |   91    | left_hand_root
#  14 |   95    | left_thumb4
#  15 |  103    | left_middle_finger4
#  16 |  111    | left_pinky_finger4
#  17 |  112    | right_hand_root
#  18 |  116    | right_thumb4
#  19 |  124    | right_middle_finger4
#  20 |  132    | right_pinky_finger4
UPPER_BODY_ORIG_IDX = [
    0, 1, 2, 3, 4,
    5, 6, 7, 8, 9, 10,
    11, 12,
    91, 95, 103, 111,
    112, 116, 124, 132,
]  # len = 21

# Skeleton links for visualization (indices into the 21-kpt filtered array)
UPPER_BODY_LINKS = [
    [5, 7], [7, 9], [6, 8], [8, 10],        # arms
    [5, 6], [5, 11], [6, 12], [11, 12],     # torso
    [0, 1], [0, 2], [1, 3], [2, 4],         # face
    [3, 5], [4, 6],                          # ear-shoulder
    [9,  13], [9,  14], [9,  15], [9,  16], # left wrist -> hand tips
    [10, 17], [10, 18], [10, 19], [10, 20], # right wrist -> hand tips
]

# ─── Model input index sets (positions in the 21-kpt filtered array) ──────────
# left arm:  l_shoulder(5), l_elbow(7), l_wrist(9),
#            l_hand_root(13), l_thumb4(14), l_mid4(15), l_pinky4(16)
LEFT_ARM_IDX  = [5, 7, 9, 13, 14, 15, 16]
# right arm: r_shoulder(6), r_elbow(8), r_wrist(10),
#            r_hand_root(17), r_thumb4(18), r_mid4(19), r_pinky4(20)
RIGHT_ARM_IDX = [6, 8, 10, 17, 18, 19, 20]
# torso context: l_hip(11), r_hip(12), l_shoulder(5), r_shoulder(6)
TORSO_IDX     = [11, 12, 5, 6]
# spine: l_hip(11), r_hip(12), l_shoulder(5), r_shoulder(6),
#        nose(0), l_eye(1), r_eye(2), l_ear(3), r_ear(4)
SPINE_IDX     = [11, 12, 5, 6, 0, 1, 2, 3, 4]

# SMPL-X joint indices filled by spine_mlp output (9 joints x rot6d)
# L_Hip(1), R_Hip(2), Spine1(3), Spine2(6), Spine3(9),
# Neck(12), L_Collar(13), R_Collar(14), Head(15)
SPINE_SMPL_JOINTS = [1, 2, 3, 6, 9, 12, 13, 14, 15]


# ─────────────────────────────────────────────────────────────────────────────
#  General helpers
# ─────────────────────────────────────────────────────────────────────────────

def make_mlp(dim_list, activations, dropout=0):
    """Build a Sequential MLP from a list of layer dims and activation names."""
    if len(dim_list) == 0 and len(activations) == 0:
        return nn.Identity()
    assert len(dim_list) == len(activations) + 1
    layers = []
    for dim_in, dim_out, activation in zip(dim_list[:-1], dim_list[1:], activations):
        layers.append(nn.Linear(dim_in, dim_out))
        for act in re.split('-', activation):
            if 'leakyrelu' in act:
                layers.append(nn.LeakyReLU(
                    negative_slope=float(re.split('=', act)[1]), inplace=True))
            elif act == 'relu':
                layers.append(nn.ReLU())
            elif act == 'sigmoid':
                layers.append(nn.Sigmoid())
            elif act == 'batchnorm':
                layers.append(nn.BatchNorm1d(dim_out))
            elif act == 'none':
                pass
        if dropout > 0:
            layers.append(nn.Dropout(p=dropout))
    return nn.Sequential(*layers)


def rot6d_to_rotmat(r6d: np.ndarray) -> np.ndarray:
    """Convert 6D rotation representation (numpy shape (6,)) to 3x3 rotation matrix.
    Gram-Schmidt orthonormalisation; columns of R = (b1, b2, b3).
    Reference: Zhou et al., CVPR 2019.
    Returns identity matrix for degenerate (near-zero) inputs.
    """
    v1, v2 = r6d[:3], r6d[3:]
    n1 = np.linalg.norm(v1)
    if n1 < 1e-6:
        return np.eye(3, dtype=np.float64)
    b1 = v1 / n1
    b2 = v2 - np.dot(b1, v2) * b1
    n2 = np.linalg.norm(b2)
    if n2 < 1e-6:
        return np.eye(3, dtype=np.float64)
    b2 = b2 / n2
    b3 = np.cross(b1, b2)
    return np.stack([b1, b2, b3], axis=-1)   # (3, 3)


def smpl_rot6d_to_booster_joints(smpl_r6d: np.ndarray) -> dict:
    """Retarget SMPL-X rot6d pose [22, 6] to Booster T1 joint angles (radians).

    Naive XYZ Euler decomposition — identical to smpl_pose_editor.py --method naive.
    Verified: varying SMPL Y (lost DoF) causes zero drift on Pitch and Roll.
      Left  shoulder : R(13) @ R(16) -> as_euler("xyz") -> [0]=pitch, [2]=roll
      Left  elbow    : R(18)          -> as_euler("xyz") -> [0]=pitch, [1]=yaw
      Left  wrist    : R(20)          -> as_euler("xyz") -> [0]=pitch, [1]=yaw, [2]=roll
      Right side     : symmetric  (collar=14, shoulder=17, elbow=19, wrist=21)
    """
    def R(j):
        return rot6d_to_rotmat(smpl_r6d[j])

    def xyz(mat):
        return Rotation.from_matrix(mat).as_euler("xyz", degrees=True)

    L_sh = xyz(R(13) @ R(16))
    R_sh = xyz(R(14) @ R(17))
    L_el = xyz(R(18))
    R_el = xyz(R(19))
    L_wr = xyz(R(20))
    R_wr = xyz(R(21))

    def _r(deg): return float(np.radians(deg))

    return {
        'l_sh_pitch':  _r(L_sh[0]), 'l_sh_roll':   _r(L_sh[2]),
        'l_el_pitch':  _r(L_el[0]), 'l_el_yaw':    _r(L_el[1]),
        'l_wr_pitch':  _r(L_wr[0]), 'l_wr_yaw':    _r(L_wr[1]), 'l_hand_roll': _r(L_wr[2]),
        'r_sh_pitch':  _r(R_sh[0]), 'r_sh_roll':   _r(R_sh[2]),
        'r_el_pitch':  _r(R_el[0]), 'r_el_yaw':    _r(R_el[1]),
        'r_wr_pitch':  _r(R_wr[0]), 'r_wr_yaw':    _r(R_wr[1]), 'r_hand_roll': _r(R_wr[2]),
    }


# ─────────────────────────────────────────────────────────────────────────────
#  GMR-based retargeter  (lightweight FK  +  mink IK, no smplx at runtime)
# ─────────────────────────────────────────────────────────────────────────────

# Precomputed T-pose data (neutral SMPL-X, zero betas, zero pose).
# Generated once by gen_smplx_tpose.py — no smplx import needed at runtime.
_TPOSE_NPZ = os.path.join(os.path.dirname(__file__), 'tpose_joint_offsets.npz')

class GMRRetargeter:
    """Full IK retargeting: rot6d → lightweight FK → GMR mink IK → Booster T1 joints.

    FK uses precomputed T-pose joint offsets (smplx_tpose.npz) — no smplx model
    is loaded or called at runtime.  Requires only: /root/GMR, mink, mujoco.

    Returned dict has the same keys as smpl_rot6d_to_booster_joints().
    """
    _GMR_PATH = '/root/GMR'

    # Valid robot choices for --gmr_robot
    ROBOT_CHOICES = ['booster_t1_29dof', 'booster_t1_29dof_posonly']

    # Robot DOF name → pyx output key
    _DOF_TO_PYX = {
        'Left_Shoulder_Pitch':  'l_sh_pitch',
        'Left_Shoulder_Roll':   'l_sh_roll',
        'Left_Elbow_Pitch':     'l_el_pitch',
        'Left_Elbow_Yaw':       'l_el_yaw',
        'Left_Wrist_Pitch':     'l_wr_pitch',
        'Left_Wrist_Yaw':       'l_wr_yaw',
        'Left_Hand_Roll':       'l_hand_roll',
        'Right_Shoulder_Pitch': 'r_sh_pitch',
        'Right_Shoulder_Roll':  'r_sh_roll',
        'Right_Elbow_Pitch':    'r_el_pitch',
        'Right_Elbow_Yaw':      'r_el_yaw',
        'Right_Wrist_Pitch':    'r_wr_pitch',
        'Right_Wrist_Yaw':      'r_wr_yaw',
        'Right_Hand_Roll':      'r_hand_roll',
    }

    def __init__(self, robot: str = 'booster_t1_29dof', logger=None):
        import sys as _sys
        if self._GMR_PATH not in _sys.path:
            _sys.path.insert(0, self._GMR_PATH)

        from general_motion_retargeting import GeneralMotionRetargeting

        if logger:
            logger.info(f'GMRRetargeter: initialising  robot={robot}...')

        self.gmr = GeneralMotionRetargeting(
            src_human='smplx', tgt_robot=robot, verbose=False)

        # Build qpos-index lookup: DOF name → qpos array index
        _m = self.gmr.model
        self._dof_qpos_idx: dict = {}
        for dof_name, dof_id in self.gmr.robot_dof_names.items():
            if dof_name is None:
                continue
            jntid = _m.dof_jntid[dof_id]
            self._dof_qpos_idx[dof_name] = int(_m.jnt_qposadr[jntid])

        # ── Load precomputed T-pose data (no smplx needed) ────────────────────
        tpose = np.load(_TPOSE_NPZ, allow_pickle=True)
        self._offsets     = tpose['offsets'].astype(np.float64)   # (N, 3) parent-relative
        self._parents     = tpose['parents'].astype(np.int32)     # (N,)
        self._joint_names = list(tpose['joint_names'])            # (N,)
        self._N           = len(self._parents)

        if logger:
            logger.info(
                f'GMRRetargeter ready  robot={robot}  '
                f'joints={self._N}  DOFs={len(self._dof_qpos_idx)}  '
                f'(offsets from {os.path.basename(_TPOSE_NPZ)}, no smplx)')

    def retarget(self, smpl_r6d: np.ndarray) -> dict:
        """Retarget SMPL-X rot6d [22, 6] via lightweight FK + GMR IK.

        Returns the same dict format as smpl_rot6d_to_booster_joints().
        """
        # ── 1. rot6d [22, 6] → rotation matrices → local rotvecs ─────────────
        rotmats = np.stack([rot6d_to_rotmat(smpl_r6d[j]) for j in range(22)])   # (22,3,3)
        rotvecs = Rotation.from_matrix(rotmats).as_rotvec()                      # (22,3)

        # ── 2. Lightweight FK: accumulate global rotations + positions ─────────
        # Uses precomputed T-pose offsets — no smplx call.
        # Joints 0..21 come from MLP rot6d; joints 22+ (hands/face) use identity.
        joint_rot = [None] * self._N
        joint_pos = np.zeros((self._N, 3), dtype=np.float64)

        for i in range(self._N):
            p = int(self._parents[i])
            if i == 0:
                joint_rot[i] = Rotation.from_rotvec(rotvecs[0])
                joint_pos[i] = self._offsets[0]                  # root world pos
            else:
                local_rv = rotvecs[i] if i <= 21 else np.zeros(3)
                joint_rot[i] = joint_rot[p] * Rotation.from_rotvec(local_rv)
                joint_pos[i] = joint_pos[p] + joint_rot[p].apply(self._offsets[i])

        # ── 3. Build human_data dict for GMR ──────────────────────────────────
        human_data = {
            jname: (joint_pos[i], joint_rot[i].as_quat(scalar_first=True))
            for i, jname in enumerate(self._joint_names)
        }

        # ── 4. GMR IK ─────────────────────────────────────────────────────────
        qpos = self.gmr.retarget(human_data)   # (34,)

        # ── 5. Extract named DOF angles from qpos ─────────────────────────────
        result: dict = {}
        for dof_name, pyx_key in self._DOF_TO_PYX.items():
            idx = self._dof_qpos_idx.get(dof_name)
            result[pyx_key] = float(qpos[idx]) if idx is not None else 0.0
        for k in ('l_sh_pitch', 'l_sh_roll', 'l_el_pitch', 'l_el_yaw',
                  'l_wr_pitch', 'l_wr_yaw', 'l_hand_roll',
                  'r_sh_pitch', 'r_sh_roll', 'r_el_pitch', 'r_el_yaw',
                  'r_wr_pitch', 'r_wr_yaw', 'r_hand_roll'):
            result.setdefault(k, 0.0)
        return result


# ── COCO wholebody indices used as torso anchors (same as training dataloader)
_TORSO_L_SHOULDER = 5
_TORSO_R_SHOULDER = 6
_TORSO_L_HIP      = 11
_TORSO_R_HIP      = 12


def normalize_2d_points_torso(points2d: np.ndarray,
                              kpts_all_pixel: np.ndarray,
                              scores_all: np.ndarray,
                              score_thr: float,
                              fallback_bbox=None,
                              min_torso_span_px: float = 10.0) -> np.ndarray:
    """Torso-span normalisation — matches dataloaders/synthium/mocap.py exactly.

    Origin : mid_hip  = 0.5 * (L_hip_px + R_hip_px)
    Scale  : torso_span = ||mid_shoulder_px - mid_hip_px||

    Output is torso-relative (mid_hip -> 0, mid_shoulder ~1 unit away).
    Values are NOT bounded to [0, 1].
    Falls back to bbox normalisation when hip/shoulder confidence is low.

    Parameters
    ----------
    points2d       : [N, 2]   pixel coords to normalise (e.g. 21-kpt subset)
    kpts_all_pixel : [133, 2] full COCO wholebody pixel coords (for anchor)
    scores_all     : [133]    per-joint confidence scores
    score_thr      : float    minimum score to trust a torso anchor
    fallback_bbox  : (x1,y1,x2,y2) or None
    min_torso_span_px : float safety clamp
    """
    pts  = np.asarray(points2d,      dtype=np.float64)
    kpts = np.asarray(kpts_all_pixel, dtype=np.float64)
    sc   = np.asarray(scores_all,    dtype=np.float64)

    hip_ok = (sc[_TORSO_L_HIP]      > score_thr) and (sc[_TORSO_R_HIP]      > score_thr)
    sho_ok = (sc[_TORSO_L_SHOULDER] > score_thr) and (sc[_TORSO_R_SHOULDER] > score_thr)

    if hip_ok and sho_ok:
        mid_hip   = 0.5 * (kpts[_TORSO_L_HIP]      + kpts[_TORSO_R_HIP])
        mid_sho   = 0.5 * (kpts[_TORSO_L_SHOULDER] + kpts[_TORSO_R_SHOULDER])
        torso_span = max(float(np.linalg.norm(mid_sho - mid_hip)), min_torso_span_px)
        pts_norm  = (pts - mid_hip) / torso_span
    elif fallback_bbox is not None:
        x1, y1, x2, y2 = fallback_bbox
        w = max(float(x2 - x1), 1.0)
        h = max(float(y2 - y1), 1.0)
        pts_norm = (pts - np.array([x1, y1])) / np.array([w, h])
    else:
        mins  = pts.min(axis=0)
        maxs  = pts.max(axis=0)
        scale = np.maximum(maxs - mins, 1.0)
        pts_norm = (pts - mins) / scale

    return pts_norm.astype(np.float32)


def _load_training_cfg(config_path: str | None, weights_path: str | None,
                       logger=None) -> dict:
    """Load training config yaml and return a flat dict of preprocessing params.

    We only extract the subset of keys that govern *inference-time preprocessing*:
      kpts_score_threshold   (dataset_settings)
      kpts_root_relative     (dataset_settings)

    If config_path is None, we try to auto-discover it by replacing the
    weights .pt suffix with .yaml (so weights and config can live side-by-side).
    If neither is found, safe hardcoded defaults are used and a warning is logged.

    Returns
    -------
    dict with keys:
        'kpts_score_threshold' : float
        'kpts_root_relative'   : bool
    """
    DEFAULTS = {
        'kpts_score_threshold': 0.5,
        'kpts_root_relative':   True,
    }

    def _warn(msg):
        print(f'[WARN] {msg}')

    def _info(msg):
        print(f'[INFO] {msg}')

    # --- resolve config path ---
    if config_path is None and weights_path is not None:
        # e.g. /path/to/model_state_dict.pt  ->  /path/to/model_state_dict.yaml
        base, _ = os.path.splitext(weights_path)
        candidate = base + '.yaml'
        if os.path.isfile(candidate):
            config_path = candidate
            _info(f'Auto-discovered training config: {config_path}')
        else:
            _warn(
                f'No config yaml found next to weights ({candidate}). '
                f'Using defaults: {DEFAULTS}'
            )
            return dict(DEFAULTS)

    if config_path is None:
        _warn(f'No --config provided and no weights to auto-discover from. '
              f'Using defaults: {DEFAULTS}')
        return dict(DEFAULTS)

    if not os.path.isfile(config_path):
        _warn(f'Config file not found: {config_path}. Using defaults: {DEFAULTS}')
        return dict(DEFAULTS)

    with open(config_path, 'r') as f:
        raw = yaml.safe_load(f)

    ds = raw.get('dataset_settings', {})
    out = {
        'kpts_score_threshold': float(ds.get(
            'kpts_score_threshold', DEFAULTS['kpts_score_threshold'])),
        'kpts_root_relative':   bool(ds.get(
            'kpts_root_relative',   DEFAULTS['kpts_root_relative'])),
    }
    _info(
        f'Training config loaded from {config_path}:\n'
        f'  kpts_score_threshold = {out["kpts_score_threshold"]}\n'
        f'  kpts_root_relative   = {out["kpts_root_relative"]}'
    )
    return out


def vertices_to_pointcloud2(vertices: np.ndarray,
                             frame_id: str = 'world',
                             stamp=None) -> PointCloud2:
    """Pack SMPL-X vertices [V, 3] into a PointCloud2 message.

    Uses raw np.tobytes() — no per-point Python allocation.
    RViz2: Add -> By topic -> smplx_cloud -> PointCloud2. Fixed Frame = 'world'.
    """
    verts = np.asarray(vertices, dtype=np.float32)   # [V, 3]
    n = verts.shape[0]
    pc2 = PointCloud2()
    pc2.header.frame_id = frame_id
    if stamp is not None:
        pc2.header.stamp = stamp
    pc2.height     = 1
    pc2.width      = n
    pc2.fields     = [
        PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
    ]
    pc2.is_bigendian = False
    pc2.point_step   = 12
    pc2.row_step     = 12 * n
    pc2.is_dense     = True
    pc2.data         = verts.tobytes()
    return pc2


def apply_rpy_rotation(vertices_np: np.ndarray,
                       rpy_deg=(0.0, 0.0, 0.0),
                       translation=(0.0, 0.0, 0.0)) -> np.ndarray:
    if vertices_np is None or len(vertices_np) == 0:
        return vertices_np
    if all(v == 0.0 for v in (*rpy_deg, *translation)):
        return vertices_np.copy()
    R_mat = Rotation.from_euler('xyz', rpy_deg, degrees=True).as_matrix()
    return (R_mat @ vertices_np.T).T + np.array(translation, dtype=np.float32)


# ─────────────────────────────────────────────────────────────────────────────
#  Smoothers  (applied element-wise to a flat numpy array, e.g. rot6d [132])
# ─────────────────────────────────────────────────────────────────────────────

class EMAFilter:
    """Exponential moving average:  x_hat = alpha*x + (1-alpha)*x_prev

    Default alpha=0.3 gives good smoothing at 30 Hz without excessive lag.
    Tune: higher alpha -> more responsive, lower alpha -> smoother.
    """
    def __init__(self, alpha: float = 0.3):
        self.alpha  = alpha
        self._state = None

    def __call__(self, x: np.ndarray) -> np.ndarray:
        if self._state is None:
            self._state = x.copy()
            return x.copy()
        self._state = self.alpha * x + (1.0 - self.alpha) * self._state
        return self._state.copy()

    def reset(self):
        self._state = None


class OneEuroFilter:
    """1-Euro filter applied element-wise to a numpy array.

    Reference: Casiez et al., CHI 2012.

    Recommended defaults for 30 Hz SMPL-X rot6d:
        min_cutoff = 1.0   (lower  -> smoother but more lag on slow motion)
        beta       = 0.007 (higher -> less lag on fast motion)
        dcutoff    = 1.0   (cutoff for the derivative low-pass filter)

    To dial in:
      - If output jitters at rest  -> decrease min_cutoff
      - If output lags during fast motion -> increase beta
    """
    def __init__(self, min_cutoff: float = 1.0, beta: float = 0.007,
                 dcutoff: float = 1.0, freq: float = 30.0):
        self.min_cutoff = min_cutoff
        self.beta       = beta
        self.dcutoff    = dcutoff
        self.freq       = freq
        self._x_prev    = None
        self._dx_prev   = None

    def _alpha(self, cutoff: np.ndarray) -> np.ndarray:
        """Compute per-element smoothing factor from cutoff frequency."""
        te  = 1.0 / self.freq
        tau = 1.0 / (2.0 * np.pi * cutoff)
        return 1.0 / (1.0 + tau / te)

    def __call__(self, x: np.ndarray) -> np.ndarray:
        if self._x_prev is None:
            self._x_prev  = x.copy()
            self._dx_prev = np.zeros_like(x)
            return x.copy()
        # Derivative estimate (low-pass filtered)
        dx   = (x - self._x_prev) * self.freq
        a_d  = self._alpha(np.full_like(x, self.dcutoff))
        edx  = a_d * dx + (1.0 - a_d) * self._dx_prev
        # Adaptive per-element cutoff
        a_x  = self._alpha(self.min_cutoff + self.beta * np.abs(edx))
        x_hat = a_x * x + (1.0 - a_x) * self._x_prev
        self._x_prev  = x_hat.copy()
        self._dx_prev = edx.copy()
        return x_hat

    def reset(self):
        self._x_prev  = None
        self._dx_prev = None


# ─────────────────────────────────────────────────────────────────────────────
#  kpts2smpl model  (body_part_mlp, smplx)
# ─────────────────────────────────────────────────────────────────────────────

class KptsSMPLXModel(nn.Module):
    """Body-part MLP: 21 keypoints [1, 21, 2] -> SMPL-X rot6d [22, 6].

    Architecture (must match training config body_part_mlp + smplx):
      left/right arm MLP : [22 -> 128 -> 128 -> 18]  relu, relu, none
      spine MLP          : [18 -> 256 -> 256 -> 54]  relu, relu, none

    Input sizes:
      arm   = 7 arm kpts x 2 + 4 torso context kpts x 2 = 22
      spine = 9 spine kpts x 2 = 18

    Output assignment:
      spine_mlp  (54) -> joints 1,2,3,6,9,12,13,14,15  (9 x rot6d)
      left_mlp   (18) -> joints 16, 18, 20              (L_Shoulder, L_Elbow, L_Wrist)
      right_mlp  (18) -> joints 17, 19, 21              (R_Shoulder, R_Elbow, R_Wrist)
      joint 0 (global_orient) = identity (zero rotation, not predicted)
    """
    def __init__(self):
        super().__init__()
        self.left_arm_mlp  = make_mlp([22, 128, 128, 18], ["relu", "relu", "none"])
        self.right_arm_mlp = make_mlp([22, 128, 128, 18], ["relu", "relu", "none"])
        self.spine_mlp     = make_mlp([18, 256, 256, 54], ["relu", "relu", "none"])

    def forward(self, kpts: torch.Tensor) -> torch.Tensor:
        """kpts: [1, 21, 2]  (torso-span-normalised, optionally root-relative)  ->  [22, 6] rot6d"""
        torso     = kpts[:, TORSO_IDX, :].reshape(1, -1)           # [1,  8]
        left_arm  = torch.cat(
            [kpts[:, LEFT_ARM_IDX, :].reshape(1, -1), torso], dim=-1)   # [1, 22]
        right_arm = torch.cat(
            [kpts[:, RIGHT_ARM_IDX, :].reshape(1, -1), torso], dim=-1)  # [1, 22]
        spine     = kpts[:, SPINE_IDX, :].reshape(1, -1)           # [1, 18]

        left_out  = self.left_arm_mlp(left_arm)    # [1, 18]
        right_out = self.right_arm_mlp(right_arm)  # [1, 18]
        spine_out = self.spine_mlp(spine)           # [1, 54]

        pred = torch.zeros(1, 22, 6)

        # Spine joints
        spine_r = spine_out.reshape(1, 9, 6)
        for pos, ji in enumerate(SPINE_SMPL_JOINTS):
            pred[:, ji, :] = spine_r[:, pos, :]

        # Left arm:  L_Shoulder(16), L_Elbow(18), L_Wrist(20)
        left_r = left_out.reshape(1, 3, 6)
        pred[:, 16, :] = left_r[:, 0, :]
        pred[:, 18, :] = left_r[:, 1, :]
        pred[:, 20, :] = left_r[:, 2, :]

        # Right arm: R_Shoulder(17), R_Elbow(19), R_Wrist(21)
        right_r = right_out.reshape(1, 3, 6)
        pred[:, 17, :] = right_r[:, 0, :]
        pred[:, 19, :] = right_r[:, 1, :]
        pred[:, 21, :] = right_r[:, 2, :]

        return pred[0]  # [22, 6]


# ─────────────────────────────────────────────────────────────────────────────
#  ROS2 Node
# ─────────────────────────────────────────────────────────────────────────────

class PoseEstimatorNode(Node):
    def __init__(self,
                 weights_path: str | None = None,
                 config_path:  str | None = None,
                 smoother_type:        str   = 'oneeuro',
                 ema_alpha:            float = 0.3,
                 oneeuro_min_cutoff:   float = 1.0,
                 oneeuro_beta:         float = 0.007,
                 oneeuro_dcutoff:      float = 1.0,
                 publish_mesh:         bool  = False,
                 retarget_mode:        str   = 'naive',
                 gmr_robot:            str   = 'booster_t1_29dof'):
        super().__init__('pose_estimator')

        self._smoother = self._init_smoother(
            smoother_type, ema_alpha,
            oneeuro_min_cutoff, oneeuro_beta, oneeuro_dcutoff)

        # ── Load training config (drives preprocessing to match training exactly) ──
        train_cfg = _load_training_cfg(
            config_path, weights_path, logger=self.get_logger())
        self._score_thr:          float = train_cfg['kpts_score_threshold']
        self._kpts_root_relative: bool  = train_cfg['kpts_root_relative']

        # ── PoseTracker (RTMDet + RTMPose) ─────────────────────────────────
        det_model  = 'src/motion_retargeting/mhr_pose_estimation/models/rtmpose-ort/rtmdet-nano'
        pose_model = 'src/motion_retargeting/mhr_pose_estimation/models/rtmpose-ort/rtmw-dw-l-m'
        self.get_logger().info('Initializing PoseTracker...')
        self.tracker = PoseTracker(
            det_model=det_model, pose_model=pose_model,
            device_name='cpu', device_id=0)
        self.tracker_state = self.tracker.create_state(
            det_interval=20, det_min_bbox_size=100)
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('Failed to open webcam!')
            raise RuntimeError('Webcam not available')
        w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.get_logger().info(f'Webcam opened: {w}x{h}')

        # ── kpts2smpl network ───────────────────────────────────────────────
        self.net = KptsSMPLXModel()
        self.net.eval()
        ckpt_path = weights_path
        self.get_logger().info(f'Loading kpts2smpl weights from {ckpt_path}...')
        ckpt = torch.load(ckpt_path, map_location='cpu', weights_only=False)
        # Support both a raw state_dict and a full trainer checkpoint
        if isinstance(ckpt, dict) and 'model_state' in ckpt:
            state = ckpt['model_state']
        elif isinstance(ckpt, dict) and 'state_dict' in ckpt:
            state = ckpt['state_dict']
        else:
            state = ckpt
        missing, unexpected = self.net.load_state_dict(state, strict=True)
        self.get_logger().info(
            f'kpts2smpl loaded — missing={len(missing)}, unexpected={len(unexpected)}')

        # ── SMPL-X body model (optional, only for mesh visualization) ─────
        self._publish_mesh = publish_mesh
        if self._publish_mesh:
            SMPLX_MODEL_PATH = '/data/mocap/data/models_smplx_v1_1/models'
            self.get_logger().info('Loading SMPL-X body model...')
            self.smplx_body = smplx_lib.create(
                SMPLX_MODEL_PATH, model_type='smplx', gender='neutral',
                use_pca=False, flat_hand_mean=True)
            self.smplx_body.eval()
            self.get_logger().info('SMPL-X body model loaded (10475 vertices).')
        else:
            self.smplx_body = None
            self.get_logger().info('Point cloud visualization disabled (--mesh to enable).')

        # ── GMR retargeter (optional, only when --retarget gmr) ─────────────
        self._retarget_mode = retarget_mode
        if retarget_mode == 'gmr':
            self.get_logger().info(f'Retarget mode: GMR (SMPL-X FK + mink IK)  robot={gmr_robot}')
            self._gmr_retargeter = GMRRetargeter(robot=gmr_robot, logger=self.get_logger())
        else:
            self._gmr_retargeter = None
            self.get_logger().info('Retarget mode: naive (XYZ Euler decomposition)')

        # ── Publishers ──────────────────────────────────────────────────────
        self.joint_pub = self.create_publisher(JointState,   '/ik/joint_states', 10)
        self.pc_pub    = self.create_publisher(PointCloud2, 'smplx_cloud',       1) if self._publish_mesh else None

        # Rotation/translation applied to SMPL-X vertices before publishing.
        # Default: +90 deg X, +90 deg Z → model appears upright in RViz.
        self.pc_rotation_rpy_deg = (90.0, 0.0, 90.0)
        self.pc_translation_m    = (0.0, 0.0, 0.0)

        # ── Timer 30 Hz ─────────────────────────────────────────────────────
        self.timer = self.create_timer(0.033, self.process_frame)

        # ── Recording state (toggle with R key, saves raw video) ──────────
        self._recording    = False
        self._video_writer = None
        self._record_dir   = os.path.dirname(os.path.abspath(__file__))
        self._record_fps   = 30.0

    # ── Smoother factory ───────────────────────────────────────────────────
    def _init_smoother(self, smoother_type: str,
                       ema_alpha: float = 0.3,
                       oneeuro_min_cutoff: float = 1.0,
                       oneeuro_beta: float = 0.007,
                       oneeuro_dcutoff: float = 1.0):
        if smoother_type == 'ema':
            self.get_logger().info(f'Smoother: EMA  alpha={ema_alpha}')
            return EMAFilter(alpha=ema_alpha)
        elif smoother_type == 'oneeuro':
            self.get_logger().info(
                f'Smoother: 1-Euro  min_cutoff={oneeuro_min_cutoff}  '
                f'beta={oneeuro_beta}  dcutoff={oneeuro_dcutoff}')
            return OneEuroFilter(
                min_cutoff=oneeuro_min_cutoff, beta=oneeuro_beta,
                dcutoff=oneeuro_dcutoff, freq=30.0)
        else:
            self.get_logger().info('Smoother: none')
            return None

    # ── Recording toggle ──────────────────────────────────────────────────
    def _toggle_recording(self, frame_shape):
        """Start or stop raw video recording.  Press R to toggle.

        Saves a timestamped .mp4 (no skeleton drawings) to the same directory
        as this script.  Works with any keyboard input including wireless
        USB/BT HID devices (numpad, presenter remote) mapped to the 'r' key.
        """
        import datetime
        if self._recording:
            if self._video_writer is not None:
                self._video_writer.release()
                self._video_writer = None
            self._recording = False
            self.get_logger().info('Recording stopped and saved.')
        else:
            os.makedirs(self._record_dir, exist_ok=True)
            ts     = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
            fname  = os.path.join(self._record_dir, f'raw_{ts}.mp4')
            h, w   = frame_shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self._video_writer = cv2.VideoWriter(
                fname, fourcc, self._record_fps, (w, h))
            if not self._video_writer.isOpened():
                self.get_logger().error(f'Failed to open VideoWriter: {fname}')
                self._video_writer = None
                return
            self._recording = True
            self.get_logger().info(f'Recording started -> {fname}')

    # ── Skeleton visualization ─────────────────────────────────────────────
    def draw_skeleton(self, frame, kpts_xy, skeleton_links,
                      scores=None, conf_thr=0.5):
        for a, b in skeleton_links:
            if scores is not None and (scores[a] < conf_thr or scores[b] < conf_thr):
                continue
            pt1 = tuple(kpts_xy[a].astype(int))
            pt2 = tuple(kpts_xy[b].astype(int))
            if not np.all(pt1 == 0) and not np.all(pt2 == 0):
                cv2.line(frame, pt1, pt2, (0, 255, 0), 2, cv2.LINE_AA)
        for i, pt in enumerate(kpts_xy):
            if scores is not None and scores[i] < conf_thr:
                continue
            if not np.all(pt == 0):
                cv2.circle(frame, tuple(pt.astype(int)), 3, (0, 0, 255), -1, cv2.LINE_AA)
        return frame

    # ── Main camera loop ───────────────────────────────────────────────────
    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to read frame')
            return

        # ── 2D keypoint detection ─────────────────────────────────────────
        keypoints, bboxes, target_ids = self.tracker(
            self.tracker_state, frame, detect=-1)
        if not len(keypoints):
            return

        kpts_raw      = np.array(keypoints[0])  # [133, 3]  x, y, score
        bbox          = bboxes[0]
        kpts_xy_pixel = kpts_raw[:, :2]
        scores        = kpts_raw[:, 2]

        # ── Draw filtered skeleton in pixel space ─────────────────────────
        kpts_px_filt  = kpts_xy_pixel[UPPER_BODY_ORIG_IDX].copy()   # [21, 2]
        scores_filt   = scores[UPPER_BODY_ORIG_IDX]                  # [21]
        kpts_px_filt[scores_filt < 0.5] = 0.0
        vis = self.draw_skeleton(
            frame.copy(), kpts_px_filt, UPPER_BODY_LINKS, scores=scores_filt)

        # ── Torso visibility gate ──────────────────────────────────────────
        # Retargeting requires both shoulders AND both hips to be detected.
        # Positions in the 21-kpt filtered array:
        #   5=L_shoulder  6=R_shoulder  11=L_hip  12=R_hip
        _thr = self._score_thr
        torso_visible = (
            scores_filt[_TORSO_L_SHOULDER] > _thr and
            scores_filt[_TORSO_R_SHOULDER] > _thr and
            scores_filt[_TORSO_L_HIP]      > _thr and
            scores_filt[_TORSO_R_HIP]      > _thr
        )

        h, w = vis.shape[:2]
        if torso_visible:
            label      = 'Retargeting active'
            text_color = (0, 220, 0)       # green
            bg_color   = (0, 60, 0)
        else:
            label      = 'Step back - full torso not visible'
            text_color = (0, 200, 255)     # amber
            bg_color   = (0, 50, 80)

        font       = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        thickness  = 1
        (tw, th), baseline = cv2.getTextSize(label, font, font_scale, thickness)
        margin = 8
        x0, y0 = margin, h - margin - th - baseline
        cv2.rectangle(vis, (x0 - 4, y0 - th - 4),
                      (x0 + tw + 4, y0 + baseline + 4), bg_color, -1)
        cv2.putText(vis, label, (x0, y0), font, font_scale,
                    text_color, thickness, cv2.LINE_AA)

        # ── REC indicator (display only, not written to video) ────────────
        if self._recording:
            cv2.circle(vis, (w - 20, 20), 8, (0, 0, 255), -1)
            cv2.putText(vis, 'REC', (w - 52, 26),
                        font, 0.55, (0, 0, 255), 2, cv2.LINE_AA)

        cv2.imshow('Filtered Skeleton', vis)

        # ── Key handling: R = toggle raw recording ────────────────────────
        key = cv2.waitKey(1) & 0xFF
        if key in (ord('r'), ord('R')):
            self._toggle_recording(frame.shape)

        # ── Write raw (no drawings) frame when recording ──────────────────
        if self._recording and self._video_writer is not None:
            self._video_writer.write(frame)

        if not torso_visible:
            return   # skip inference and publishing until full torso is seen

        # ── Preprocess keypoints for model (matches training pipeline exactly) ──
        # 1) Select 21-kpt subset
        kpts_sub   = kpts_xy_pixel[UPPER_BODY_ORIG_IDX].copy()   # [21, 2]
        scores_sub = scores[UPPER_BODY_ORIG_IDX]                  # [21]

        # 2) Torso-span normalisation — origin = mid_hip, scale = ||mid_sho - mid_hip||
        #    Mirrors dataloaders/synthium/mocap.py::normalize_2d_points_torso.
        #    Uses full 133-kpt array for anchor computation; falls back to bbox.
        kpts_xy = normalize_2d_points_torso(
            kpts_sub, kpts_xy_pixel, scores,
            score_thr=self._score_thr, fallback_bbox=bbox,
        )  # [21, 2]  torso-relative, NOT bounded to [0, 1]

        # 3) Root-relative: subtract mid-hip in normalised space
        #    Driven by kpts_root_relative from training config.
        #    left_hip = filtered idx 11, right_hip = filtered idx 12
        if self._kpts_root_relative:
            if scores_sub[11] > self._score_thr and scores_sub[12] > self._score_thr:
                hip_center = 0.5 * (kpts_xy[11] + kpts_xy[12])
            else:
                vis = scores_sub > self._score_thr
                hip_center = (kpts_xy[vis].mean(axis=0) if vis.sum() >= 1
                              else kpts_xy.mean(axis=0))
            kpts_xy = kpts_xy - hip_center                        # [21, 2]

        # 4) Zero low-confidence keypoints (inclusive threshold, matches training)
        kpts_xy[scores_sub <= self._score_thr] = 0.0

        # ── kpts2smpl inference ────────────────────────────────────────────
        kpts_t = torch.from_numpy(kpts_xy.astype(np.float32)).unsqueeze(0)  # [1, 21, 2]
        with torch.inference_mode():
            pred_smpl = self.net(kpts_t)   # [22, 6]  rot6d

        pred_np = pred_smpl.numpy()        # [22, 6]

        # ── Smoothing on flattened rot6d [132] ────────────────────────────
        if self._smoother is not None:
            pred_np = self._smoother(pred_np.reshape(-1)).reshape(22, 6)

        stamp = self.get_clock().now().to_msg()

        # ── SMPL-X forward pass -> PointCloud2 (only when --mesh enabled) ──
        if self._publish_mesh:
            body_pose_aa = np.zeros((1, 63), dtype=np.float32)
            for j in range(1, 22):
                aa = Rotation.from_matrix(rot6d_to_rotmat(pred_np[j])).as_rotvec()
                body_pose_aa[0, (j - 1) * 3 : j * 3] = aa
            with torch.no_grad():
                smplx_out = self.smplx_body(
                    global_orient=torch.zeros(1, 3),
                    body_pose=torch.tensor(body_pose_aa),
                )
            verts_np = smplx_out.vertices[0].detach().numpy()        # [10475, 3]
            verts_np = apply_rpy_rotation(
                verts_np,
                rpy_deg=self.pc_rotation_rpy_deg,
                translation=self.pc_translation_m,
            )
            self.pc_pub.publish(
                vertices_to_pointcloud2(verts_np, frame_id='world', stamp=stamp))

        # ── Retarget SMPL-X pose to Booster T1 joint angles ───────────────
        if self._retarget_mode == 'gmr' and self._gmr_retargeter is not None:
            j = self._gmr_retargeter.retarget(pred_np)
        else:
            j = smpl_rot6d_to_booster_joints(pred_np)

        msg = JointState()
        msg.header.stamp = stamp
        msg.name = [
            'AAHead_yaw',                    'Head_pitch',
            'Left_Shoulder_Pitch',           'Left_Shoulder_Roll',
            'Left_Elbow_Pitch',              'Left_Elbow_Yaw',
            'Left_Wrist_Pitch',              'Left_Wrist_Yaw',     'Left_Hand_Roll',
            'T1_left_base_link_left_Link1',  'T1_left_Link1_left_Link11',
            'T1_left_base_link_left_Link2',  'T1_left_Link2_left_Link22',
            'Right_Shoulder_Pitch',          'Right_Shoulder_Roll',
            'Right_Elbow_Pitch',             'Right_Elbow_Yaw',
            'Right_Wrist_Pitch',             'Right_Wrist_Yaw',    'Right_Hand_Roll',
            'T1_right_base_link_right_Link1', 'T1_right_Link1_right_Link11',
            'T1_right_base_link_right_Link2', 'T1_right_Link2_right_Link22',
            'Waist_joint',
            'Left_Hip_Pitch',   'Left_Hip_Roll',   'Left_Hip_Yaw',
            'Left_Knee_Pitch',  'Left_Ankle_Pitch', 'Left_Ankle_Roll',
            'Right_Hip_Pitch',  'Right_Hip_Roll',  'Right_Hip_Yaw',
            'Right_Knee_Pitch', 'Right_Ankle_Pitch', 'Right_Ankle_Roll',
        ]
        msg.position = [
            0.0, 0.0,                                                  # head
            j['l_sh_pitch'],  j['l_sh_roll'],                         # L shoulder
            j['l_el_pitch'],  j['l_el_yaw'],                          # L elbow
            j['l_wr_pitch'],  j['l_wr_yaw'], j['l_hand_roll'],        # L wrist/hand
            0.0, 0.0, 0.0, 0.0,                                       # T1_left fingers
            j['r_sh_pitch'],  j['r_sh_roll'],                         # R shoulder
            j['r_el_pitch'],  j['r_el_yaw'],                          # R elbow
            j['r_wr_pitch'],  j['r_wr_yaw'], j['r_hand_roll'],        # R wrist/hand
            0.0, 0.0, 0.0, 0.0,                                       # T1_right fingers
            0.0,                                                       # Waist
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,                           # L leg
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,                           # R leg
        ]
        self.joint_pub.publish(msg)

    def destroy_node(self):
        if self._video_writer is not None:
            self._video_writer.release()
            self._video_writer = None
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    import argparse

    # ── Parse our own args before handing control to ROS2 ─────────────────
    # We consume --weights / -w and leave everything else (including
    # --ros-args ...) for rclpy.init().
    parser = argparse.ArgumentParser(
        description='Pose Estimator Node',
        add_help=False,   # don't conflict with ros2 --help
    )
    parser.add_argument(
        '--weights', '-w',
        default=None,
        metavar='PATH',
        help=(
            'Path to kpts2smpl checkpoint (.pt). '
            'Default: models/all_epoch_0031_best_0031_state_dict.pt'
        ),
    )
    parser.add_argument(
        '--config', '-c',
        default=None,
        metavar='PATH',
        help=(
            'Path to training config yaml. '
            'If omitted, auto-discovered as <weights_stem>.yaml next to the weights file.'
        ),
    )
    parser.add_argument(
        '--smoother',
        default='oneeuro',
        choices=['none', 'ema', 'oneeuro'],
        help='Smoother type (default: oneeuro)',
    )
    parser.add_argument(
        '--ema_alpha', type=float, default=0.3,
        help='EMA smoothing factor (default: 0.3)',
    )
    parser.add_argument(
        '--oneeuro_min_cutoff', type=float, default=1.0,
        help='1-Euro min cutoff frequency (default: 1.0)',
    )
    parser.add_argument(
        '--oneeuro_beta', type=float, default=0.007,
        help='1-Euro beta (speed coefficient) (default: 0.007)',
    )
    parser.add_argument(
        '--oneeuro_dcutoff', type=float, default=1.0,
        help='1-Euro derivative cutoff frequency (default: 1.0)',
    )
    parser.add_argument(
        '--mesh', action='store_true', default=False,
        help='Enable SMPL-X point cloud visualization on smplx_cloud topic (default: disabled)',
    )
    parser.add_argument(
        '--retarget',
        default='naive',
        choices=['naive', 'gmr'],
        help=(
            'Retargeting mode (default: naive). '
            'naive: direct XYZ Euler decomposition. '
            'gmr: SMPL-X FK + GMR mink IK (requires /root/GMR mount).'
        ),
    )
    parser.add_argument(
        '--gmr_robot',
        default='booster_t1_29dof',
        choices=['booster_t1_29dof', 'booster_t1_29dof_posonly'],
        help=(
            'GMR robot config (only used when --retarget gmr, default: booster_t1_29dof). '
            'booster_t1_29dof:        position + orientation weights (full IK). '
            'booster_t1_29dof_posonly: position weights only (orientation weights = 0).'
        ),
    )
    our_ns, ros_args = parser.parse_known_args(sys.argv[1:])

    # Pass the remaining (ROS2) args to rclpy
    rclpy.init(args=ros_args)
    node = PoseEstimatorNode(
        weights_path=our_ns.weights,
        config_path=our_ns.config,
        smoother_type=our_ns.smoother,
        ema_alpha=our_ns.ema_alpha,
        oneeuro_min_cutoff=our_ns.oneeuro_min_cutoff,
        oneeuro_beta=our_ns.oneeuro_beta,
        oneeuro_dcutoff=our_ns.oneeuro_dcutoff,
        publish_mesh=our_ns.mesh,
        retarget_mode=our_ns.retarget,
        gmr_robot=our_ns.gmr_robot,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
