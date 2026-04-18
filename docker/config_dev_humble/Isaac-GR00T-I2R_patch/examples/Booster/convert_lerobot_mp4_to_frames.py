# DO NOT USE THIS SCRIPT, IT WILL CREATE FILE W V LARGE SIZE AS RAW TENSOR DATA

import cv2
import torch
import torchvision
from pathlib import Path
from tqdm import tqdm

# For method 1
import os
os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"
os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "video_codec;av1" # Force codec hint

DATASET_ROOT = Path("Jan29")

video_dirs = list(DATASET_ROOT.glob("**/*.mp4"))

for video_path in tqdm(video_dirs):
    ### Method 1
    cap = cv2.VideoCapture(str(video_path))
    frames = []

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frames.append(frame)

    cap.release()
    video_frames = torch.tensor(frames)  # T,H,W,C

    ### Method 2 instead of above
    # video_frames, audio, metadata = torchvision.io.read_video(str(video_path), pts_unit="sec")

    save_path = video_path.with_suffix(".pt")
    torch.save(video_frames, save_path)
    print(f"video {video_path} processed")