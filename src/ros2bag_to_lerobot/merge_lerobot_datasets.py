#!/usr/bin/env python3
"""
merge_lerobot_datasets.py
--------------------------
Filters and concatenates per-bag LeRobot datasets in lerobot_dataset_storage
into a single merged dataset, selected by task label.

Output layout (flat, no chunk subdirs):
  merged_dataset/
    data/
      episode_0.parquet
      episode_1.parquet
      ...
    videos/
      observation.images.color/
        episode_0.mp4
        episode_1.mp4
        ...
      observation.images.depth/
        episode_0.mp4
        ...
    meta/
      tasks.jsonl
      episodes.jsonl
      info.json        (copied + updated from first source)
      modality.json    (copied from first source)

Usage:
    python merge_lerobot_datasets.py
    python merge_lerobot_datasets.py --config path/to/merge_config.yaml
"""

import argparse
import json
import shutil
from pathlib import Path

import numpy as np
import yaml
import pandas as pd


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def load_config(config_path: str) -> dict:
    with open(config_path) as f:
        return yaml.safe_load(f)


def read_jsonl(path: Path) -> list[dict]:
    records = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line:
                records.append(json.loads(line))
    return records


def get_dataset_tasks(
    dataset_dir: Path,
) -> tuple[list[str], dict[int, str]]:
    """
    Returns (episode_task_names, {task_index: task_name}).

    episode_task_names is the full `tasks` list from the first episode in
    episodes.jsonl (e.g. ["Do nothing", "Wave", "valid"]).  This is used for
    matching — a dataset is included if *any* entry in this list is a
    requested label, not just the first one.

    Falls back to all task names from tasks.jsonl if episodes.jsonl is absent.
    """
    tasks_file    = dataset_dir / "meta" / "tasks.jsonl"
    episodes_file = dataset_dir / "meta" / "episodes.jsonl"

    if not tasks_file.exists():
        return [], {}

    tasks: dict[int, str] = {}
    for rec in read_jsonl(tasks_file):
        tasks[rec["task_index"]] = rec["task"]

    episode_task_names: list[str] = []
    if episodes_file.exists():
        episodes = read_jsonl(episodes_file)
        if episodes:
            episode_task_names = episodes[0].get("tasks", [])

    if not episode_task_names:
        episode_task_names = list(tasks.values())

    return episode_task_names, tasks


# ---------------------------------------------------------------------------
# Dataset discovery
# ---------------------------------------------------------------------------

def find_matching_datasets(
    storage_dir: Path,
    task_labels: list[str],
) -> list[tuple[Path, str, dict[int, str], list[str]]]:
    """
    Scan storage_dir for dataset folders where ANY task in the episode's
    tasks list matches a requested label — not just the first/primary one.
    This catches episodes like ["Do nothing", "Wave", "valid"] when "Wave"
    is requested.

    Returns (dataset_dir, matched_label, local_tasks, full_episode_tasks)
    sorted by (position in task_labels, folder name):
        Wave ep1, Wave ep2, … Salute ep1, Salute ep2, …
    """
    label_order = {label: i for i, label in enumerate(task_labels)}
    matches: list[tuple[Path, str, dict[int, str], list[str]]] = []

    for dataset_dir in sorted(storage_dir.iterdir()):
        if not dataset_dir.is_dir():
            continue
        episode_tasks, local_tasks = get_dataset_tasks(dataset_dir)
        # Find the first requested label that appears in this episode's tasks
        matched_label: str | None = None
        for label in task_labels:
            if label in episode_tasks:
                matched_label = label
                break
        if matched_label is not None:
            matches.append((dataset_dir, matched_label, local_tasks, episode_tasks))

    matches.sort(key=lambda x: (label_order[x[1]], x[0].name))
    return matches


# ---------------------------------------------------------------------------
# Stats computation
# ---------------------------------------------------------------------------

# Columns that are dataset bookkeeping, not robot features — skip in stats.
_META_COLUMNS = {"episode_index", "frame_index", "task_index", "index"}


def _to_matrix(series: pd.Series) -> np.ndarray | None:
    """
    Convert a pandas Series whose elements are scalars or 1-D arrays into a
    2-D float64 numpy matrix of shape (N, D).  Returns None if the column
    cannot be converted (e.g. strings).
    """
    try:
        sample = series.iloc[0]
        if isinstance(sample, str):
            return None
        if hasattr(sample, "__len__"):
            # array-valued column (e.g. observation.state has D=29)
            matrix = np.array(series.tolist(), dtype=np.float64)   # (N, D)
        else:
            matrix = np.array(series.values, dtype=np.float64).reshape(-1, 1)
        return matrix
    except (ValueError, TypeError):
        return None


def compute_stats(output_dir: Path) -> dict:
    """
    Load every episode_*.parquet in output_dir/data, then compute per-feature
    statistics matching the LeRobot stats.json schema:
        min, max, mean, std  — shape (D,) → stored as list of length D
        count                — always [N]  (single-element list)
        q01, q10, q50, q90, q99 — shape (D,)

    Video features (observation.images.*) live in separate .mp4 files and are
    not present as parquet columns — they are excluded automatically.
    Metadata columns (episode_index, frame_index, task_index) are skipped.
    """
    data_dir = output_dir / "data"
    parquet_files = sorted(data_dir.glob("episode_*.parquet"))
    if not parquet_files:
        return {}

    print("[stats] Loading merged parquets for stats computation…")

    # Accumulate per-column row lists across all files
    column_rows: dict[str, list[np.ndarray]] = {}
    for pf in parquet_files:
        df = pd.read_parquet(pf)
        for col in df.columns:
            if col in _META_COLUMNS:
                continue
            matrix = _to_matrix(df[col])
            if matrix is None:
                continue
            if col not in column_rows:
                column_rows[col] = []
            column_rows[col].append(matrix)

    stats: dict = {}
    for col, matrices in column_rows.items():
        mat = np.concatenate(matrices, axis=0)   # (N_total, D)
        n   = mat.shape[0]
        stats[col] = {
            "min":   mat.min(axis=0).tolist(),
            "max":   mat.max(axis=0).tolist(),
            "mean":  mat.mean(axis=0).tolist(),
            "std":   mat.std(axis=0, ddof=0).tolist(),
            "count": [n],
            "q01":   np.quantile(mat, 0.01, axis=0).tolist(),
            "q10":   np.quantile(mat, 0.10, axis=0).tolist(),
            "q50":   np.quantile(mat, 0.50, axis=0).tolist(),
            "q90":   np.quantile(mat, 0.90, axis=0).tolist(),
            "q99":   np.quantile(mat, 0.99, axis=0).tolist(),
        }
        print(f"  {col}: shape ({n}, {mat.shape[1]})")

    return stats


def compute_relative_stats(output_dir: Path) -> dict:
    """
    Compute statistics of relative actions: (action - observation.state)
    per frame, matching the GR00T relative_stats.json schema.

    For joint-based robots like Booster T1, the relative action is simply
    the delta between the commanded position and the current joint position.
    Stats keys: mean, std, min, max, q01, q99 — each a list of length D.
    """
    data_dir = output_dir / "data"
    parquet_files = sorted(data_dir.glob("episode_*.parquet"))
    if not parquet_files:
        return {}

    print("[rel_stats] Computing relative action statistics…")
    rel_actions: list[np.ndarray] = []

    for pf in parquet_files:
        df = pd.read_parquet(pf)
        if "action" not in df.columns or "observation.state" not in df.columns:
            continue
        actions = np.array(df["action"].tolist(),            dtype=np.float32)  # (N, D)
        states  = np.array(df["observation.state"].tolist(), dtype=np.float32)  # (N, D)
        rel_actions.append(actions - states)

    if not rel_actions:
        return {}

    mat = np.concatenate(rel_actions, axis=0)  # (N_total, D)
    return {
        "action": {
            "mean": mat.mean(axis=0).tolist(),
            "std":  mat.std(axis=0).tolist(),
            "min":  mat.min(axis=0).tolist(),
            "max":  mat.max(axis=0).tolist(),
            "q01":  np.quantile(mat, 0.01, axis=0).tolist(),
            "q99":  np.quantile(mat, 0.99, axis=0).tolist(),
        }
    }


# ---------------------------------------------------------------------------
# Core merge
# ---------------------------------------------------------------------------

def merge_datasets(config: dict) -> None:
    storage_dir = Path(config["storage_dir"])
    output_dir  = Path(config["output_dir"])
    task_labels: list[str] = config["task_filter"]["labels"]

    # ---- Discover matching source datasets --------------------------------
    matches = find_matching_datasets(storage_dir, task_labels)
    if not matches:
        print(f"[merge] No datasets found matching tasks: {task_labels}")
        return

    print(f"[merge] Found {len(matches)} matching dataset(s):")
    for d, t, _, ep_tasks in matches:
        print(f"  [{t}]  {d.name}  (episode tasks: {ep_tasks})")

    # ---- Wipe and recreate output dir to avoid stale artifacts ------------
    if output_dir.exists():
        print(f"[merge] Removing existing output: {output_dir}")
        shutil.rmtree(output_dir)
    output_dir.mkdir(parents=True)
    (output_dir / "data").mkdir()
    (output_dir / "meta").mkdir()

    # ---- Iterate over matched datasets ------------------------------------
    global_episode_idx = 0
    episodes_meta: list[dict] = []
    global_task_map: dict[str, int] = {}   # task_str -> global task_index
    next_task_idx = 0
    total_frames = 0
    video_keys: set[str] = set()
    source_info: dict | None = None
    source_modality: dict | None = None

    for dataset_dir, matched_label, local_tasks, _ in matches:
        src_episodes = read_jsonl(dataset_dir / "meta" / "episodes.jsonl")

        for src_ep in src_episodes:
            src_ep_idx: int = src_ep["episode_index"]
            chunk_idx = src_ep_idx // 1000  # chunks_size default = 1000
            src_parquet = (
                dataset_dir / "data"
                / f"chunk-{chunk_idx:03d}"
                / f"file-{src_ep_idx:03d}.parquet"
            )

            if not src_parquet.exists():
                print(f"  [WARN] parquet not found, skipping: {src_parquet}")
                continue

            # -- Remap parquet: episode_index, per-frame task_index ----------
            df = pd.read_parquet(src_parquet)
            df["episode_index"] = global_episode_idx

            if "task_label" in df.columns:
                # Per-frame task_index from the recorded task_label string
                for label in df["task_label"].unique():
                    if label not in global_task_map:
                        global_task_map[label] = next_task_idx
                        next_task_idx += 1
                df["task_index"] = df["task_label"].map(global_task_map)
            else:
                # Fallback for bags converted before task_label was added
                if matched_label not in global_task_map:
                    global_task_map[matched_label] = next_task_idx
                    next_task_idx += 1
                df["task_index"] = global_task_map[matched_label]

            out_parquet = output_dir / "data" / f"episode_{global_episode_idx}.parquet"
            df.to_parquet(out_parquet, index=False)
            length = len(df)
            total_frames += length

            # -- Copy videos -------------------------------------------------
            videos_dir = dataset_dir / "videos"
            if videos_dir.exists():
                for video_key_dir in sorted(videos_dir.iterdir()):
                    if not video_key_dir.is_dir():
                        continue
                    vkey = video_key_dir.name
                    video_keys.add(vkey)

                    src_video = (
                        video_key_dir
                        / f"chunk-{chunk_idx:03d}"
                        / f"file-{src_ep_idx:03d}.mp4"
                    )
                    if not src_video.exists():
                        print(f"  [WARN] video not found: {src_video}")
                        continue

                    out_video_dir = output_dir / "videos" / vkey
                    out_video_dir.mkdir(parents=True, exist_ok=True)
                    out_video = out_video_dir / f"episode_{global_episode_idx}.mp4"
                    shutil.copy2(src_video, out_video)

            # -- Record meta -------------------------------------------------
            episodes_meta.append({
                "episode_index": global_episode_idx,
                "tasks": src_ep.get("tasks", [matched_label]),
                "length": length,
            })
            print(
                f"  ep{global_episode_idx:04d}  [{matched_label}]  "
                f"{dataset_dir.name}  ({length} frames)"
            )
            global_episode_idx += 1

        # Keep meta templates from first source
        if source_info is None:
            info_path = dataset_dir / "meta" / "info.json"
            if info_path.exists():
                with open(info_path) as f:
                    source_info = json.load(f)
        if source_modality is None:
            mod_path = dataset_dir / "meta" / "modality.json"
            if mod_path.exists():
                with open(mod_path) as f:
                    source_modality = json.load(f)

    # ---- Write meta -------------------------------------------------------
    fps: float = source_info.get("fps", 50) if source_info else 50.0

    # meta/tasks.jsonl — one entry per unique task string (global task map)
    with open(output_dir / "meta" / "tasks.jsonl", "w") as f:
        for task_str, task_idx in sorted(global_task_map.items(), key=lambda x: x[1]):
            f.write(json.dumps({"task_index": task_idx, "task": task_str}) + "\n")

    # meta/episodes.jsonl
    with open(output_dir / "meta" / "episodes.jsonl", "w") as f:
        for ep in episodes_meta:
            f.write(json.dumps(ep) + "\n")

    # meta/info.json  (update counts + flatten paths + add total_seconds)
    if source_info is not None:
        source_info["total_episodes"] = global_episode_idx
        source_info["total_frames"]   = total_frames
        source_info["total_tasks"]    = len(global_task_map)
        source_info["total_videos"]   = len(video_keys) * global_episode_idx
        source_info["total_chunks"]   = 1
        source_info["total_seconds"]  = round(total_frames / fps, 3)
        source_info["data_path"]  = "data/episode_{episode_index}.parquet"
        source_info["video_path"] = "videos/{video_key}/episode_{episode_index}.mp4"
        with open(output_dir / "meta" / "info.json", "w") as f:
            json.dump(source_info, f, indent=4)

    # meta/modality.json  (straight copy — schema unchanged)
    if source_modality is not None:
        with open(output_dir / "meta" / "modality.json", "w") as f:
            json.dump(source_modality, f, indent=4)

    # meta/stats.json  (recomputed from all merged parquet data)
    stats = compute_stats(output_dir)
    if stats:
        with open(output_dir / "meta" / "stats.json", "w") as f:
            json.dump(stats, f, indent=4)

    # meta/relative_stats.json  (action - observation.state deltas, GR00T format)
    rel_stats = compute_relative_stats(output_dir)
    if rel_stats:
        with open(output_dir / "meta" / "relative_stats.json", "w") as f:
            json.dump(rel_stats, f, indent=4)

    # ---- Summary ----------------------------------------------------------
    total_seconds = round(total_frames / fps, 1)
    print(f"\n[merge] Done.")
    print(f"  Episodes : {global_episode_idx}")
    print(f"  Frames   : {total_frames}  ({total_seconds}s @ {fps}fps)")
    print(f"  Tasks    : {list(global_task_map.keys())}")
    print(f"  Output   : {output_dir.resolve()}")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Filter and concatenate LeRobot per-bag datasets by task label."
    )
    parser.add_argument(
        "--config",
        default="merge_config.yaml",
        help="Path to merge_config.yaml (default: merge_config.yaml)",
    )
    args = parser.parse_args()

    config = load_config(args.config)
    merge_datasets(config)


if __name__ == "__main__":
    main()
