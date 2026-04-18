# Booster ROS 2 → LeRobot Converter

Converts **Booster Robotics ROS 2 bag files** into **LeRobot-compatible datasets**, with a companion tool to filter and merge those datasets by task label for training.

---

## Project Structure

```text
.
├── bags/                        # [INPUT]  ROS 2 bag folders (git-ignored)
├── lerobot_dataset_storage/     # [OUTPUT] Per-bag LeRobot datasets (git-ignored)
├── merged_dataset/              # [OUTPUT] Merged training dataset (git-ignored)
│
├── rosbag_to_lerobot.py         # Convert ROS bags → LeRobot datasets
├── config.yaml                  # Conversion config
│
├── merge_lerobot_datasets.py    # Filter & merge datasets by task label
├── merge_config.yaml            # Merge config
│
├── validate_dataset.py          # Inspect a converted dataset
├── inspect_parquet.py           # Inspect a parquet file directly
└── requirements.txt
```

---

## 1 · Convert ROS Bags

### Setup

```bash
conda create -n lerobot_converter python=3.10 -y
conda activate lerobot_converter
```

OpenCV must be built from source for full FFmpeg/depth support:

```bash
pip uninstall -y opencv-python opencv-python-headless
sudo apt-get install -y libavcodec-dev libavformat-dev libswscale-dev libavutil-dev
export CMAKE_ARGS="-D WITH_FFMPEG=ON -D CMAKE_BUILD_TYPE=RELEASE"
pip install --no-binary opencv-python-headless --force-reinstall opencv-python-headless
pip install -r requirements.txt
```

### Configure

Edit `config.yaml`:

```yaml
settings:
  bag_dir: "./bags"
  output_root: "./lerobot_dataset_storage"
  robot_type: "booster_t1"
  target_freq: 50
  use_videos: true
  split_episodes_by_bag: true   # true = one LeRobot dataset per bag
```

**`split_episodes_by_bag: true`** — each bag becomes its own standalone dataset folder under `lerobot_dataset_storage/`. This is the intended mode; use the merge tool (below) to combine them for training.

### Run

```bash
python rosbag_to_lerobot.py
```

Each output folder contains:

```text
lerobot_dataset_storage/<bag_name>/
  data/chunk-000/file-000.parquet
  videos/observation.images.color/chunk-000/file-000.mp4
  videos/observation.images.depth/chunk-000/file-000.mp4
  meta/episodes.jsonl
  meta/tasks.jsonl
  meta/info.json
  meta/stats.json
  meta/modality.json
```

### Validate

```bash
python validate_dataset.py --frame 100
# If you hit a GLib symbol error:
export LD_PRELOAD=$CONDA_PREFIX/lib/libglib-2.0.so.0
```

---

## 2 · Merge Datasets by Task

After converting multiple bags, use this tool to filter by task label and produce a single merged dataset ready for training.

### Configure

Edit `merge_config.yaml`:

```yaml
storage_dir: "./lerobot_dataset_storage"
output_dir:  "./merged_dataset"

task_filter:
  labels:
    - "Wave"
    - "Salute"
    # - "Shake hands"
    # - "Place hands on head"
```

- Only datasets whose **primary task** (first entry in `episodes.jsonl → tasks[]`) matches a label are included.
- Episodes are ordered by label position, then by folder name: Wave 1, Wave 2 … Salute 1, Salute 2 …
- Secondary per-frame tasks (e.g. `"Do nothing"`, `"Idle"`) are remapped to a global task index and kept — they do not affect filtering.
- The output directory is **wiped on every run** to avoid stale artifacts.

### Run

```bash
python merge_lerobot_datasets.py
# or specify a config:
python merge_lerobot_datasets.py --config merge_config.yaml
```

Output layout (flat, no chunk subdirs):

```text
merged_dataset/
  data/
    episode_0.parquet
    episode_1.parquet
    ...
  videos/
    observation.images.color/episode_0.mp4  ...
    observation.images.depth/episode_0.mp4  ...
  meta/
    tasks.jsonl       # global task index across all merged episodes
    episodes.jsonl    # one line per episode
    info.json         # updated totals + total_seconds field
    stats.json        # recomputed from all merged parquet data
    modality.json     # copied from first source (schema unchanged)
```

`info.json` includes a `total_seconds` field (`total_frames / fps`) for quick dataset duration checks.

`stats.json` is fully recomputed (min/max/mean/std/quantiles per feature dimension) from the merged parquets. Video pixel stats are excluded — those require decoding the MP4s and are not recomputed here.

---

## Time Synchronization

ROS bags are asynchronous. The converter resamples all streams to a master clock (`target_freq`):

- **High-frequency data** (joints, IMU): linear interpolation to each clock tick.
- **Images**: zero-order hold (nearest previous frame).
- **Strict alignment**: if any camera is missing for a timestep, that timestep is dropped entirely — every frame in the output is guaranteed complete.
