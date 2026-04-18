"""
Booster Humanoid ROS2 (db3) to LeRobot Converter
------------------------------------------------
Generates:
1. LeRobot Dataset (Parquet w/ Images as Video, Depth as Tensors)
2. meta/episodes.jsonl
3. meta/tasks.jsonl
4. meta/modality.json
"""

import shutil
import yaml
import json
import numpy as np
import pandas as pd
import cv2
import re
from pathlib import Path
from tqdm import tqdm
import gc
import copy


# ROS Bag Libraries
from rosbags.highlevel import AnyReader
from rosbags.typesys import get_types_from_msg, get_typestore, Stores

# LeRobot
from lerobot.datasets.lerobot_dataset import LeRobotDataset

# ==============================================================================
# 1. SETUP & CONFIG LOADING
# ==============================================================================


def load_config(config_path="config.yaml"):
    with open(config_path, "r") as f:
        cfg = yaml.safe_load(f)

    if "split_episodes_by_bag" not in cfg["settings"]:
        cfg["settings"]["split_episodes_by_bag"] = True

    joint_groups = cfg["joint_groups"]

    joint_order = []
    for group in joint_groups.values():
        joint_order.extend(group)

    processed_map = {}
    for topic, specs in cfg["topic_map"].items():
        processed_map[topic] = (specs[0], specs[1], tuple(specs[2]))

    return cfg, joint_order, processed_map, joint_groups


# ==============================================================================
# 2. CONVERTER CLASS
# ==============================================================================


class BoosterBagConverter:
    def __init__(self, config_path="config.yaml"):
        self.cfg, self.joint_order, self.topic_map, self.joint_groups = load_config(
            config_path
        )
        self.settings = self.cfg["settings"]

        # [FIX] Automatically rename non-image keys to avoid LeRobot Stats Crash
        self._sanitize_keys()

        self.typestore = get_typestore(Stores.ROS2_HUMBLE)
        self._register_custom_types()

        # Buffer containers
        self.vector_data = {}
        self.string_data = {}
        self.start_t = 0
        self.end_t = 0

        self.active_image_keys = []
        self.active_image_shapes = {}
        self.image_decode_fail_counts = {}

        # --- Metadata Tracking ---
        self._reset_dataset_state()

        # Ad-hoc flag to add task_label to json
        self.append_task_label_at_end = None

    def _sanitize_keys(self):
        """
        LeRobot crashes during stats aggregation if a key starts with 'observation.images'
        but the data is not an image (e.g. camera_info vector [9]).
        This function moves them to 'observation.cameras' automatically.
        """
        new_map = {}
        for topic, (key, is_img, shape) in self.topic_map.items():
            if "observation.images" in key and not is_img:
                new_key = key.replace("observation.images", "observation.camera")
                print(
                    f"[Auto-Fix] Renaming '{key}' -> '{new_key}' to prevent LeRobot stats error."
                )
                new_map[topic] = (new_key, is_img, shape)
            else:
                new_map[topic] = (key, is_img, shape)
        self.topic_map = new_map

    def _reset_dataset_state(self):
        self.episode_stats = []
        self.known_tasks = {}
        self.next_task_idx = 0
        self.current_ep_idx_global = 0
        self.current_ep_tasks = set()
        self.current_ep_length = 0
        self.frame_task_labels: list[str] = []

    def _register_custom_types(self):
        custom_types = {}
        for type_name, definition in self.cfg.get("msg_definitions", {}).items():
            try:
                new_types = get_types_from_msg(definition, type_name)
                custom_types.update(new_types)
            except Exception as e:
                print(f"[Warning] Could not parse {type_name}: {e}")
        self.typestore.register(custom_types)

    # --- Handlers ---
    def _handle_image(self, msg):
        # Support for CompressedImage
        if hasattr(msg, "format") and hasattr(msg, "data"):
            # Check for compressedDepth
            # Format usually: "16UC1; compressedDepth"
            if "compressedDepth" in msg.format:
                # compressedDepth has a 12-byte header
                if len(msg.data) > 12:
                    np_arr = np.frombuffer(msg.data[12:], np.uint8)
                    img = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)

                    if img is None:
                        # Fallback or error
                        raise ValueError("Failed to decode compressedDepth image")

                    # It should be 16UC1 (uint16)
                    if img.dtype == np.uint16:
                        # Pack into 3 channels [High, Low, Zero]
                        high = (img >> 8).astype(np.uint8)
                        low = (img & 0xFF).astype(np.uint8)
                        zeros = np.zeros_like(high)
                        img = np.stack([high, low, zeros], axis=-1)
                        return img
                    else:
                        pass  # Fall through to standard handling if not uint16

            # Standard Compressed Image (JPEG/PNG)
            np_arr = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # Reads as BGR
            if img is None:
                raise ValueError("Failed to decode compressed image")

            # Convert BGR to RGB
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            return img

        # Standard Raw Image Parsing
        dtype = np.uint8
        channels = 3

        if "16UC1" in msg.encoding or "mono16" in msg.encoding:
            if getattr(msg, "is_bigendian", 0):
                dtype = np.dtype(">u2")
            else:
                dtype = np.dtype("<u2")
            channels = 1
        elif "mono8" in msg.encoding:
            channels = 1

        img = np.frombuffer(msg.data, dtype=dtype)

        if channels == 1:
            img = img.reshape((msg.height, msg.width))
            if img.dtype == np.uint16 or img.dtype == np.dtype(">u2") or img.dtype == np.dtype("<u2"):
                if img.dtype != np.uint16:
                    img = img.astype(np.uint16)
                # [CLEVER MEMORY FIX] High/Low byte splitting
                high = (img >> 8).astype(np.uint8)
                low = (img & 0xFF).astype(np.uint8)
                zeros = np.zeros_like(high)
                img = np.stack([high, low, zeros], axis=-1)
            else:
                img = np.expand_dims(img, -1)
                img = np.repeat(img, 3, axis=-1)
        else:
            img = img.reshape((msg.height, msg.width, channels))
            if "bgr8" in msg.encoding:
                img = img[..., ::-1]
        return img

    def _handle_joint_state(self, msg):
        current_snapshot = dict(zip(msg.name, msg.position))
        return np.array(
            [current_snapshot.get(j, 0.0) for j in self.joint_order], dtype=np.float32
        )

    def _handle_float_array(self, msg):
        return np.array(msg.data, dtype=np.float32)

    def _handle_int_array(self, msg):
        return np.array(msg.data, dtype=np.int32)

    def _handle_int32(self, msg):
        return np.array([msg.data], dtype=np.int32)

    def _handle_bool(self, msg):
        return np.array([msg.data], dtype=np.bool_)

    def _handle_string(self, msg):
        return str(msg.data)

    def _handle_tf_message(self, msg):
        if hasattr(msg, "transforms") and len(msg.transforms) > 0:
            tf = msg.transforms[0].transform
            t = tf.translation
            r = tf.rotation
            return np.array([t.x, t.y, t.z, r.x, r.y, r.z, r.w], dtype=np.float32)
        return np.zeros(7, dtype=np.float32)

    def _handle_camera_info(self, msg):
        return np.array(msg.k, dtype=np.float32)

    def _handle_standard_imu(self, msg):
        return np.array(
            [
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w,
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z,
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
            ],
            dtype=np.float32,
        )

    def get_lerobot_features(self):
        features = {}
        use_videos = self.settings["use_videos"]
        target_freq = self.settings["target_freq"]

        features["timestamp"] = {"dtype": "float32", "shape": (1,), "names": None}

        for _, (key, is_img, shape) in self.topic_map.items():
            if key == "episode_index":
                features[key] = {
                    "dtype": "int32",
                    "shape": shape,
                }
            if key == "task_label": # cannot be added into .parquet else error in line: dataset.add_frame(item)
                # features[key] = {
                #     "dtype": "string",
                #     "shape": shape,
                # }
                self.append_task_label_at_end = key
                continue
            if key == "is_done":
                features[key] = {
                    "dtype": "bool",
                    "shape": shape,
                }
            if key in features:
                continue

            if is_img:
                if key not in self.active_image_keys:
                    continue

                is_depth = "depth" in key

                features[key] = {
                    "dtype": "video" if use_videos else "image",
                    "shape": shape,
                    "names": ["height", "width", "channel"],
                }

                if use_videos:
                    features[key]["info"] = {
                        "video.fps": float(target_freq),
                        "video.codec": "h264", # using libx264
                        "video.pix_fmt": "yuv420p",
                        "video.is_depth_map": is_depth,
                        "has_audio": False
                    }
            else:
                names_list = None
                if key in ["observation.state", "observation.velocity"]:
                    names_list = self.joint_order
                elif key == "observation.head_rpy":
                    names_list = [
                        "roll",
                        "pitch",
                        "yaw",
                    ],
                elif key == "observation.imu":
                    names_list = [
                        "orientation.x",
                        "orientation.y",
                        "orientation.z",
                        "orientation.w",
                        "angular_velocity.x",
                        "angular_velocity.y",
                        "angular_velocity.z",
                        "linear_acceleration.x",
                        'linear_acceleration.y',
                        "linear_acceleration.z",
                    ]
                elif key == "observation.hand_states":
                    names_list = [
                        "left_hand.pos",
                        "right_hand.pos",
                    ]
                elif key == "observation.tf_pose":
                    names_list = [
                        "translation.x", 
                        "translation.y", 
                        "translation.z", 
                        "rotation.x", 
                        "rotation.y", 
                        "rotation.z", 
                        "rotation.w",
                    ]
                elif key == "observation.camera.color_info":
                    names_list = [
                        "focal_length.fx", 
                        "0", 
                        "printipal_point.cx", 
                        "0", 
                        "focal_length.fy", 
                        "printipal_point.cy", 
                        "0",
                        "0",
                        "intrinsic_camera_matrix.K",
                    ]
                elif key == "observation.camera.depth_info":
                    names_list = [
                        "focal_length.fx", 
                        "0", 
                        "printipal_point.cx", 
                        "0", 
                        "focal_length.fy", 
                        "printipal_point.cy", 
                        "0",
                        "0",
                        "intrinsic_camera_matrix.K",
                    ]

                features[key] = {
                    "dtype": "float32",
                    "shape": shape,
                    "names": names_list,
                }

        features["action"] = {
            "dtype": "float32",
            "shape": (len(self.joint_order),),
            "names": self.joint_order,
        }
        features["frame_index"] = {
            "dtype": "int64",
            "shape": [1],
        }
        features["index"] = {
            "dtype": "int64",
            "shape": [1],
        }
        features["task_index"] = {
            "dtype": "int64",
            "shape": [1],
        }

        return features

    def _load_vectors_pass(self, bag_path):
        """Loads non-image data for a single bag directory."""
        self.vector_data = {}
        self.string_data = {}

        # print(f"   [Pass 1] Reading Vectors from: {bag_path.name}")

        vector_buffers = {}
        string_buffers = {}

        vector_topics = [
            t for t, v in self.topic_map.items() if not v[1] and v[0] != "task_label"
        ]
        string_topics = [t for t, v in self.topic_map.items() if v[0] == "task_label"]
        image_topics = [t for t, v in self.topic_map.items() if v[1]]

        found_image_topics = set()

        with AnyReader([bag_path], default_typestore=self.typestore) as reader:
            self.start_t = reader.start_time
            self.end_t = reader.end_time

            for conn in reader.connections:
                if conn.topic in image_topics:
                    found_image_topics.add(conn.topic)

            if not self.active_image_keys:
                self.active_image_keys = [
                    self.topic_map[t][0] for t in found_image_topics
                ]
                for t in found_image_topics:
                    key, _, shape = self.topic_map[t]
                    self.active_image_shapes[key] = shape

            connections = [
                x
                for x in reader.connections
                if x.topic in vector_topics + string_topics
            ]

            for connection, timestamp, rawdata in reader.messages(
                connections=connections
            ):
                topic = connection.topic
                key, _, _ = self.topic_map[topic]
                t_sec = (timestamp - self.start_t) * 1e-9

                try:
                    msg = reader.deserialize(rawdata, connection.msgtype)

                    if key == "task_label":
                        data = self._handle_string(msg)
                        if key not in string_buffers:
                            string_buffers[key] = {"t": [], "d": []}
                        string_buffers[key]["t"].append(t_sec)
                        string_buffers[key]["d"].append(data)
                        continue

                    data = None
                    if key == "observation.state":
                        data = self._handle_joint_state(msg)
                    elif key == "action":
                        data = self._handle_joint_state(msg)  # Explicit action topic
                    elif key == "observation.velocity":
                        data = self._handle_float_array(msg)
                    elif key == "observation.head_rpy":
                        data = self._handle_float_array(msg)
                    elif key == "observation.hand_states":
                        data = self._handle_int_array(msg)
                    elif key == "observation.imu":
                        data = self._handle_standard_imu(msg)
                    elif "info" in key:
                        data = self._handle_camera_info(msg)
                    elif key == "episode_index":
                        data = self._handle_int32(msg)
                    elif key == "is_done":
                        data = self._handle_bool(msg)
                    elif "TFMessage" in connection.msgtype:
                        data = self._handle_tf_message(msg)
                    else:
                        continue

                    if data is not None:
                        if key not in vector_buffers:
                            vector_buffers[key] = {"t": [], "d": []}
                        vector_buffers[key]["t"].append(t_sec)
                        vector_buffers[key]["d"].append(data)

                        # Fallback: If 'action' is not explicitly mapped, use observation.state
                        if key == "observation.state":
                            # Check if 'action' is explicitly mapped in topic_map
                            is_explicit = any(
                                v[0] == "action" for v in self.topic_map.values()
                            )

                            if not is_explicit:
                                if "action" not in vector_buffers:
                                    vector_buffers["action"] = {"t": [], "d": []}
                                vector_buffers["action"]["t"].append(t_sec)
                                vector_buffers["action"]["d"].append(data)

                except Exception:
                    pass

        for key, buf in vector_buffers.items():
            if not buf["t"]:
                continue
            ts = np.array(buf["t"])
            ds = np.array(buf["d"])
            _, u_idx = np.unique(ts, return_index=True)
            self.vector_data[key] = (ts[u_idx], ds[u_idx])

        for key, buf in string_buffers.items():
            if not buf["t"]:
                continue
            self.string_data[key] = (np.array(buf["t"]), buf["d"])

    def get_interpolated_data(self, key, t_query):
        if key in self.string_data:
            ts, ds = self.string_data[key]
            idx = np.searchsorted(ts, t_query, side="right") - 1
            idx = max(0, idx)
            return ds[idx]

        if key not in self.vector_data:
            return None

        ts, ds = self.vector_data[key]

        if key == "episode_index":
            idx = np.searchsorted(ts, t_query, side="left")
            idx = np.clip(idx, 0, len(ts) - 1)
            if idx > 0 and (t_query - ts[idx - 1] < ts[idx] - t_query):
                idx = idx - 1
            return ds[idx]

        if key == "is_done":
            idx = np.searchsorted(ts, t_query, side="right") - 1
            idx = max(0, idx) # handle Boolean
            return ds[idx]

        dim = ds.shape[1]
        result = np.zeros(dim, dtype=np.float32)
        for i in range(dim):
            result[i] = np.interp(t_query, ts, ds[:, i])
        return result

    def process_bag_data(self, bag_path, dataset, target_timestamps, prev_ep_idx_arg):
        """Processes frames for a single bag into the given dataset."""
        self.image_decode_fail_counts = {}
        last_known_images = {
            k: np.zeros(self.active_image_shapes[k], dtype=np.uint8)
            for k in self.active_image_keys
        }
        seen_image_keys = set()
        local_prev_ep = prev_ep_idx_arg
        cursor_idx = 0

        image_topics = [t for t, v in self.topic_map.items() if v[1]]
        topic_to_key = {t: self.topic_map[t][0] for t in image_topics}

        num_frames = len(target_timestamps)
        pbar = tqdm(total=num_frames, desc=f"Writing Frames", unit="frame")

        with AnyReader([bag_path], default_typestore=self.typestore) as reader:
            connections = [x for x in reader.connections if x.topic in image_topics]

            for connection, timestamp, rawdata in reader.messages(
                connections=connections
            ):
                if cursor_idx >= num_frames:
                    break

                t_msg = (timestamp - self.start_t) * 1e-9
                key = topic_to_key[connection.topic]

                try:
                    msg = reader.deserialize(rawdata, connection.msgtype)
                    last_known_images[key] = self._handle_image(msg)
                    seen_image_keys.add(key)
                except Exception as e:
                    fail_key = connection.topic
                    self.image_decode_fail_counts[fail_key] = (
                        self.image_decode_fail_counts.get(fail_key, 0) + 1
                    )
                    if self.image_decode_fail_counts[fail_key] <= 5:
                        print(
                            f"[Image Decode Warning] topic={fail_key} msgtype={connection.msgtype} "
                            f"error={e}"
                        )
                    continue

                # Avoid writing frames until every expected image stream has at least one valid frame.
                if len(seen_image_keys) < len(self.active_image_keys):
                    continue

                while (
                    cursor_idx < num_frames and t_msg >= target_timestamps[cursor_idx]
                ):
                    target_t = target_timestamps[cursor_idx]

                    ep_vector = self.get_interpolated_data("episode_index", target_t)
                    if ep_vector is not None:
                        curr_ep_idx = int(round(ep_vector.item()))
                    else:
                        curr_ep_idx = local_prev_ep if local_prev_ep is not None else 0

                    if local_prev_ep is not None and curr_ep_idx != local_prev_ep:
                        dataset.save_episode()
                        self._finish_episode_metadata()

                    self._write_frame(dataset, last_known_images, target_t, curr_ep_idx)

                    local_prev_ep = curr_ep_idx
                    cursor_idx += 1
                    pbar.update(1)

        pbar.close()
        for topic, count in self.image_decode_fail_counts.items():
            if count > 0:
                print(f"[Image Decode Summary] topic={topic} failures={count}")
        return local_prev_ep

    def process_all_bags(self):

        root_path = Path(self.settings["bag_dir"])
        output_root = Path(self.settings["output_root"])
        dataset_name_base = self.settings["dataset_name"]
        split_by_bag = self.settings.get("split_episodes_by_bag", True)

        bag_paths = sorted([p.parent for p in root_path.rglob("metadata.yaml")])
        if not bag_paths:
            print(f"No ROS2 bags found in {root_path}")
            return

        print(
            f"Found {len(bag_paths)} bags. Mode: {'One Dataset Per Bag' if split_by_bag else 'Merged Continuous Dataset'}"
        )

        # Load schema from first bag to create global features
        print("Peeking schema...")
        self._load_vectors_pass(bag_paths[0])
        features = self.get_lerobot_features()

        for i, bag_path in enumerate(bag_paths):
            print(f"\n--- Processing {i+1}/{len(bag_paths)}: {bag_path.name} ---")

            if split_by_bag:
                self._reset_dataset_state()
                self._load_vectors_pass(bag_path)

                bag_suffix = re.sub(r"[^A-Za-z0-9._-]+", "_", bag_path.name)
                bag_dataset_name = f"{dataset_name_base}_{bag_suffix}"
                bag_dataset_root = output_root / bag_dataset_name
                if bag_dataset_root.exists():
                    shutil.rmtree(bag_dataset_root)

                # Check for bag-specific task label
                bag_task_label = "operate_booster"
                if "task_label" in self.string_data:
                    _, labels = self.string_data["task_label"]
                    if len(labels) > 0:
                        bag_task_label = labels[0]

                dataset = LeRobotDataset.create(
                    repo_id=bag_dataset_name,
                    root=bag_dataset_root,
                    robot_type=self.settings["robot_type"],
                    fps=self.settings["target_freq"],
                    use_videos=self.settings["use_videos"],
                    features=features,
                )

                duration_sec = (self.end_t - self.start_t) * 1e-9
                num_frames = int(duration_sec * self.settings["target_freq"])
                timestamps = np.linspace(0, duration_sec, num_frames)

                self.process_bag_data(
                    bag_path, dataset, timestamps, prev_ep_idx_arg=None
                )

                # Only save if we actually have data in the buffer
                if self.current_ep_length > 0:
                    dataset.save_episode()
                    self._finish_episode_metadata()

                # === CRITICAL FIX: Flush video writers and finalize the dataset ===
                if hasattr(dataset, "consolidate"):
                    dataset.consolidate()

                # === CRITICAL FIX: Force Close Dataset ===
                # This ensures the ParquetWriter writes the footer and releases the file handle
                del dataset
                gc.collect()

                # Patch parquets: add per-frame task_label string column
                if self.frame_task_labels:
                    label_iter = iter(self.frame_task_labels)
                    for pf in sorted((bag_dataset_root / "data" / "chunk-000").glob("file-*.parquet")):
                        df_patch = pd.read_parquet(pf)
                        n = len(df_patch)
                        df_patch["task_label"] = [next(label_iter) for _ in range(n)]
                        df_patch.to_parquet(pf, index=False)
                    self.frame_task_labels = []

                bag_data_dir = bag_dataset_root / "data" / "chunk-000"
                if not bag_data_dir.exists():
                    print(
                        f"Warning: No data generated for bag {bag_path.name}. Skipping."
                    )
                    if bag_dataset_root.exists():
                        shutil.rmtree(bag_dataset_root)
                    continue

                if bag_task_label and bag_task_label != "operate_booster":
                    for stat in self.episode_stats:
                        if "operate_booster" in stat["tasks"]:
                            stat["tasks"].remove("operate_booster")
                        if bag_task_label not in stat["tasks"]:
                            stat["tasks"].append(bag_task_label)

                    if bag_task_label not in self.known_tasks:
                        self.known_tasks[bag_task_label] = self.next_task_idx
                        self.next_task_idx += 1

                total_frames = sum(stat["length"] for stat in self.episode_stats)
                total_episodes = len(self.episode_stats)

                self._save_custom_metadata(bag_dataset_root)
                print("Generating dataset statistics...")
                self._generate_info_json(
                    bag_dataset_root, features, total_episodes, total_frames
                )
                self._generate_stats_json(bag_dataset_root, features)

                print(f"Saved standalone dataset: {bag_dataset_root}")

            else:
                pass

    def _write_frame(self, dataset, image_data, t_curr, curr_ep_idx):
        item = {}
        # item["timestamp"] = np.array([t_curr], dtype=np.float32)

        for key in self.vector_data:
            if key == "episode_index":
                continue
            val = self.get_interpolated_data(key, t_curr)
            if val is not None:
                if key == "is_done":
                    item[key] = val.astype(np.bool_)
                else:
                    item[key] = val.astype(np.float32)

        # Zero-fill numeric features declared in topic_map but missing from
        # this bag (e.g. /hand_states absent in older recordings).
        # Excludes: image topics, string topics (string_data), episode_index.
        for _, (key, is_image, shape) in self.topic_map.items():
            if not is_image and key not in self.string_data and key != "episode_index" and key not in item:
                item[key] = np.zeros(shape, dtype=np.float32)

        for k, v in image_data.items():
            item[k] = v

        task_label = "operate_booster"
        if "task_label" in self.string_data:
            found_task = self.get_interpolated_data("task_label", t_curr)
            if found_task:
                task_label = found_task

        if task_label not in self.known_tasks:
            self.known_tasks[task_label] = self.next_task_idx
            self.next_task_idx += 1

        self.current_ep_tasks.add(task_label)
        self.current_ep_length += 1
        self.frame_task_labels.append(task_label)

        item["task"] = task_label
        dataset.add_frame(item)

        return task_label

    def _finish_episode_metadata(self):
        if self.current_ep_length == 0:
            return

        tasks_list = list(self.current_ep_tasks)
        if "valid" not in tasks_list:
            tasks_list.append("valid")

        self.episode_stats.append(
            {
                "episode_index": self.current_ep_idx_global,
                "tasks": tasks_list,
                "length": self.current_ep_length,
            }
        )

        self.current_ep_idx_global += 1
        self.current_ep_length = 0
        self.current_ep_tasks = set()

    def _save_custom_metadata(self, output_path):
        print("\nGenerating Metadata Files...")

        meta_dir = output_path / "meta"
        meta_dir.mkdir(parents=True, exist_ok=True)

        tasks_path = meta_dir / "tasks.jsonl"
        with open(tasks_path, "w") as f:
            sorted_tasks = sorted(self.known_tasks.items(), key=lambda x: x[1])
            for t_label, t_idx in sorted_tasks:
                line = {"task_index": t_idx, "task": t_label}
                f.write(json.dumps(line) + "\n")

        episodes_path = meta_dir / "episodes.jsonl"
        with open(episodes_path, "w") as f:
            for ep_data in self.episode_stats:
                f.write(json.dumps(ep_data) + "\n")

        state_modality = {}
        action_modality = {}

        curr_idx = 0
        for group_name, joint_list in self.joint_groups.items():
            count = len(joint_list)
            entry = {"start": curr_idx, "end": curr_idx + count}
            state_modality[group_name] = entry
            action_modality[group_name] = entry
            curr_idx += count

        video_modality = {}
        for key in self.active_image_keys:
            friendly_name = key.split(".")[-1]
            video_modality[friendly_name] = {"original_key": key}

        modality_data = {
            "state": state_modality,
            "action": action_modality,
            "video": video_modality,
            "annotation": {"human.action.task_description": {"original_key": "task_label"}, "human.validity": {}},
        }

        modality_path = meta_dir / "modality.json"
        with open(modality_path, "w") as f:
            json.dump(modality_data, f, indent=4)

        print(f"Metadata generation complete. Files saved in: {meta_dir}")

    def _generate_info_json(self, output_path, features, total_episodes, total_frames):
        """Generate info.json with dataset metadata."""
        meta_dir = output_path / "meta"
        meta_dir.mkdir(parents=True, exist_ok=True)
        ### task_label / human annotations cannot be added into LeRobot dataset owing to feature mismatch in 'frame' issue, missing features: {'task_label'}
        ### Thus, appending manually into info.json
        if self.append_task_label_at_end:
            info_features = copy.deepcopy(features)
            info_features[self.append_task_label_at_end] = {"dtype": "string", "shape": [1]}
        else:
            info_features = copy.deepcopy(features)

        # Build video paths template for image features
        video_keys = [k for k, v in info_features.items() if v.get("dtype") == "video"]

        info = {
            "codebase_version": "v2.1",
            "robot_type": self.settings["robot_type"],
            "fps": self.settings["target_freq"],
            "total_episodes": total_episodes,
            "total_frames": total_frames,
            "total_tasks": len(self.known_tasks),
            "total_videos": len(video_keys) * total_episodes,
            "total_chunks": 1,
            "chunks_size": 1000,
            "data_path": "data/chunk-{episode_chunk:03d}/file-{episode_index:03d}.parquet",
            "video_path": "videos/{video_key}/chunk-{episode_chunk:03d}/file-{episode_index:03d}.mp4",
            "features": info_features,
        }

        info_path = meta_dir / "info.json"
        with open(info_path, "w") as f:
            json.dump(info, f, indent=4)

        print(f"Generated info.json at: {info_path}")

    def _generate_stats_json(self, output_path, features):
        """Generate stats.json by computing statistics from parquet files."""
        meta_dir = output_path / "meta"
        data_dir = output_path / "data" / "chunk-000"

        if not data_dir.exists():
            print("Warning: No data directory found, skipping stats generation.")
            return

        # Find all parquet files
        parquet_files = sorted(data_dir.glob("*.parquet"))
        if not parquet_files:
            print("Warning: No parquet files found, skipping stats generation.")
            return

        print(f"Computing statistics from {len(parquet_files)} parquet files...")

        lowdim_features = [
            key
            for key, feat in features.items()
            if "float" in str(feat.get("dtype", "")).lower()
        ]
        if not lowdim_features:
            print("Warning: No float features found, skipping stats generation.")
            return

        stats_path = meta_dir / "stats.json"

        def _stats_file_is_valid(path, feature_keys):
            if not path.exists():
                return False
            try:
                with open(path, "r") as f:
                    existing_stats = json.load(f)
                for feature in feature_keys:
                    if feature not in existing_stats:
                        return False
                    if not isinstance(existing_stats[feature], dict):
                        return False
                    for stat_name in ["mean", "std", "min", "max", "q01", "q99"]:
                        if stat_name not in existing_stats[feature]:
                            return False
                return True
            except Exception:
                return False

        if _stats_file_is_valid(stats_path, lowdim_features):
            print(f"stats.json already valid at: {stats_path}")
            return

        all_data = {feature: [] for feature in lowdim_features}

        # Import internally to ensure availability
        try:
            import pyarrow.parquet as pq
        except ImportError:
            print("Error: pyarrow not found. Cannot generate stats.")
            return

        for pq_file in tqdm(parquet_files, desc="Reading parquet files"):
            try:
                table = pq.read_table(pq_file)
                df = table.to_pandas()

                for feature in lowdim_features:
                    if feature not in df.columns:
                        continue

                    for value in df[feature].values:
                        if value is None:
                            continue
                        all_data[feature].append(np.asarray(value, dtype=np.float32))
            except Exception as e:
                print(f"Warning: Could not read {pq_file}: {e}")
                continue

        # Compute statistics
        stats = {}
        for key, values in all_data.items():
            try:
                if not values:
                    continue

                arr = np.vstack([np.atleast_1d(v) for v in values]).astype(np.float32)

                stats[key] = {
                    "mean": arr.mean(axis=0).tolist(),
                    "std": arr.std(axis=0).tolist(),
                    "min": arr.min(axis=0).tolist(),
                    "max": arr.max(axis=0).tolist(),
                    "q01": np.quantile(arr, 0.01, axis=0).tolist(),
                    "q99": np.quantile(arr, 0.99, axis=0).tolist(),
                }
            except Exception as e:
                print(f"Warning: Could not compute stats for {key}: {e}")
                continue

        with open(stats_path, "w") as f:
            json.dump(stats, f, indent=4)

        print(f"Generated stats.json at: {stats_path}")


if __name__ == "__main__":
    converter = BoosterBagConverter()
    converter.process_all_bags()
