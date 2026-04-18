"""
Manual Validation Script for LeRobot Dataset (Headless / Save-to-Disk)
Usage: python validate_dataset.py --frame 100
"""

import argparse
import yaml
import json
import pandas as pd
import cv2
import numpy as np
import os
from pathlib import Path

# --- FIX: Force Headless Backend to prevent Segfaults ---
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ==========================================
# CONFIGURATION: CHANGE TARGET FILE HERE
# ==========================================
CHUNK_NAME = "chunk-000"
EPISODE_NAME = "file-000"  # Do not add extension here
# ==========================================


def load_config(config_path="config.yaml"):
    if not os.path.exists(config_path):
        print(f"Error: Config file not found at {config_path}")
        exit(1)
    with open(config_path, "r") as f:
        return yaml.safe_load(f)


def validate(frame_idx=100):
    # 1. Locate Dataset
    config = load_config()
    output_root = Path(config["settings"]["output_root"])
    dataset_name = config["settings"]["dataset_name"]
    dataset_path = output_root / dataset_name

    print(f"--- Validating Local Dataset: {dataset_path} ---")

    if not dataset_path.exists():
        print(f"CRITICAL: Dataset folder not found: {dataset_path}")
        return

    # 2. Load Data (Parquet)
    # Structure: dataset/data/chunk-000/file-000.parquet
    parquet_path = dataset_path / "data" / CHUNK_NAME / f"{EPISODE_NAME}.parquet"

    if not parquet_path.exists():
        print(f"Parquet not found at specific path: {parquet_path}")
        # Fallback search
        pqs = list((dataset_path / "data" / CHUNK_NAME).glob("*.parquet"))
        if pqs:
            parquet_path = pqs[0]
            print(f"Fallback: Using first found parquet: {parquet_path.name}")
        else:
            print("No parquet files found in chunk directory.")
            return

    print(f"[Data] Loading parquet: {parquet_path.name}")
    df = pd.read_parquet(parquet_path)

    total_frames = len(df)
    print(f"[Data] Total frames in episode: {total_frames}")

    if frame_idx >= total_frames:
        print(
            f"Requested frame {frame_idx} exceeds length. Clamping to {total_frames - 1}"
        )
        frame_idx = total_frames - 1

    # 3. Setup Plot
    fig, axs = plt.subplots(2, 2, figsize=(16, 10))
    axs = axs.flatten()
    plt.suptitle(f"Frame {frame_idx} | {CHUNK_NAME}/{EPISODE_NAME}")

    # --- Color Image ---
    color_key = "observation.images.color"
    color_dir = dataset_path / "videos" / color_key / CHUNK_NAME
    color_vids = list(color_dir.glob("*.mp4"))

    if color_vids:
        cap = cv2.VideoCapture(str(color_vids[0])) # Grab the first valid mp4
        cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx)
        ret, frame = cap.read()
        cap.release()
        if ret:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            axs[0].imshow(frame)
            axs[0].set_title(f"Color Camera\n{frame.shape}")
        else:
            axs[0].text(0.5, 0.5, "Read Error", ha="center")
    else:
        print(f"Missing Color Video in: {color_dir}")
        axs[0].text(0.5, 0.5, "Missing File", ha="center")
    axs[0].axis("off")

    # --- Depth Image (Decoded) ---
    depth_key = "observation.images.depth"
    depth_dir = dataset_path / "videos" / depth_key / CHUNK_NAME
    depth_vids = list(depth_dir.glob("*.mp4"))

    if depth_vids:
        cap = cv2.VideoCapture(str(depth_vids[0])) # Grab the first valid mp4
        cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx)
        ret, frame = cap.read()
        cap.release()

        if ret:
            # Decode: High Byte (Red) << 8 | Low Byte (Green)
            # Assuming encoded as RGB where R=High, G=Low, B=0
            high = frame[:, :, 2].astype(np.uint16)  # Opencv reads BGR, so 2 is Red
            low = frame[:, :, 1].astype(np.uint16)  # 1 is Green
            depth_recon = (high << 8) | low

            valid_depth = depth_recon[depth_recon > 0]
            if len(valid_depth) > 0:
                print(
                    f"[Depth Stats] Min: {valid_depth.min()}mm, Max: {valid_depth.max()}mm"
                )
                vmin = np.percentile(valid_depth, 1)
                vmax = np.percentile(valid_depth, 98)
            else:
                vmin, vmax = 0, 10000

            im = axs[1].imshow(depth_recon, cmap="gray", vmin=vmin, vmax=vmax)
            plt.colorbar(im, ax=axs[1], fraction=0.046, pad=0.04)
            axs[1].set_title(f"Depth (Decoded)\nRange: {vmin:.0f}-{vmax:.0f}mm")
        else:
            axs[1].text(0.5, 0.5, "Frame Read Error", ha="center")
    else:
        print(f"Missing Depth Video in: {depth_dir}")
        axs[1].text(0.5, 0.5, "No Depth Video", ha="center")
    axs[1].axis("off")

    # --- State Vector ---
    state_key = "observation.state"
    if state_key in df.columns:
        row = df.iloc[frame_idx]
        state_vec = np.array(row[state_key])
        axs[2].bar(range(len(state_vec)), state_vec, color="teal")
        axs[2].set_title(f"State Vector\nDim: {len(state_vec)}")
        axs[2].set_ylim(state_vec.min() - 0.1, state_vec.max() + 0.1)
    else:
        axs[2].text(0.5, 0.5, "No State Data", ha="center")

    # --- Action Vector ---
    action_key = "action"
    if action_key in df.columns:
        row = df.iloc[frame_idx]
        action_vec = np.array(row[action_key])
        axs[3].bar(range(len(action_vec)), action_vec, color="orange")
        axs[3].set_title(f"Action Vector (from IK)\nDim: {len(action_vec)}")
        axs[3].set_ylim(action_vec.min() - 0.1, action_vec.max() + 0.1)
    else:
        axs[3].text(0.5, 0.5, "No Action Data Found", ha="center")
        axs[3].axis("off")

    plt.tight_layout()

    # --- SAVE TO DISK INSTEAD OF SHOWING ---
    output_filename = f"validation_frame_{frame_idx}.png"
    plt.savefig(output_filename)
    print(f"\n[Success] Validation image saved to: {output_filename}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--frame", type=int, default=50)
    args = parser.parse_args()
    validate(args.frame)
