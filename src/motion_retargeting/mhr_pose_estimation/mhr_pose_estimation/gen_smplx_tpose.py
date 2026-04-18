#!/usr/bin/env python3
"""Run once to precompute SMPL-X neutral T-pose joint offsets.

Output: tpose_joint_offsets.npz  (saved next to this script)
  offsets    (N, 3) float32 — each joint's position relative to its parent in T-pose
  parents    (N,)   int32   — parent index for each joint (-1 for root)
  joint_names (N,)  str     — SMPL-X joint names

No betas / no shape: neutral-gender zero-shape model.
The file is loaded at runtime by GMRRetargeter for lightweight FK — no smplx import needed.
"""
import pathlib
import numpy as np
import torch
import smplx
from smplx.joint_names import JOINT_NAMES

SMPLX_MODEL_DIR = '/root/GMR/assets/body_models'
OUT_PATH = pathlib.Path(__file__).parent / 'tpose_joint_offsets.npz'

model = smplx.create(SMPLX_MODEL_DIR, model_type='smplx',
                     gender='neutral', use_pca=False, flat_hand_mean=True)
model.eval()

N = len(model.parents)
names = list(JOINT_NAMES[:N])
parents = model.parents.numpy().astype(np.int32)   # (N,)

with torch.no_grad():
    out = model(
        global_orient=torch.zeros(1, 3),
        body_pose=torch.zeros(1, 63),
        left_hand_pose=torch.zeros(1, 45),
        right_hand_pose=torch.zeros(1, 45),
        jaw_pose=torch.zeros(1, 3),
        leye_pose=torch.zeros(1, 3),
        reye_pose=torch.zeros(1, 3),
    )

pos = out.joints[0].detach().numpy()   # (N+, 3) world positions in T-pose

# Parent-relative offsets (parent rotation = identity in T-pose, so no rotation needed)
offsets = np.zeros((N, 3), dtype=np.float32)
for i in range(N):
    if i == 0:
        offsets[i] = pos[i].astype(np.float32)
    else:
        offsets[i] = (pos[i] - pos[int(parents[i])]).astype(np.float32)

np.savez(str(OUT_PATH), offsets=offsets, parents=parents,
         joint_names=np.array(names))
print(f'Saved {N} joints → {OUT_PATH}')
print('Sample offsets (joints 0-4):')
for i in range(5):
    print(f'  [{i}] {names[i]:20s}  {offsets[i]}')
