import json
import re
from pathlib import Path
from typing import Optional, Tuple

import numpy as np
import pyarrow.parquet as pq
from datasets import Dataset

from control_py.utils.loguru_settings import setup_loguru

setup_loguru(log_folder_path="log", show_on_terminal=True, terminal_level="DEBUG")

ACTION = "action"


def filter_episode(example, episode_idx: int):
    return example["episode_index"] == episode_idx


def map_dataset_action_to_arm_index(
    dataset_action_name: str,
    robot_type: str,
) -> Optional[Tuple[str, int]]:
    """
    Map dataset action names to (arm_side, joint_index).

    Supported formats:
        - openarm_left_joint1
        - unitree_left_joint1
        - left_joint_1 / left_joint1

    Only maps arm joints 1-7.
    Hand / finger / gripper joints are skipped.
    """

    name = dataset_action_name.lower()

    if re.search(
        r"(gripper|thumb|index|middle|ring|pinky|metacarpal|proximal|distal)",
        name,
    ):
        return None

    robot_type = robot_type.lower()
    patterns = []

    if robot_type in ("openarm", "unitree"):
        patterns.append(rf"{robot_type}_(left|right)_joint(\d+)")

    patterns.extend([
        r"(left|right)_joint_?(\d+)",
    ])

    for pattern in patterns:
        matched = re.match(pattern, name)
        if not matched:
            continue

        arm_side, joint_str = matched.groups()
        joint_num = int(joint_str)

        if 1 <= joint_num <= 7:
            return arm_side, joint_num - 1

        return None

    return None


def detect_degree_dataset(
    actions,
    joint_action_indices,
    action_key,
    max_frames: int = 10,
    rad_threshold: float = 4.0,
) -> bool:
    """
    Detect whether dataset actions are in degrees.
    """

    vals = []
    num_frames = min(max_frames, len(actions))

    for i in joint_action_indices:
        for j in range(num_frames):
            try:
                vals.append(abs(float(actions[j][action_key][i])))
            except (KeyError, IndexError, TypeError):
                continue

    return max(vals) > rad_threshold if vals else False


def action_to_arm_cmd(action, mapping, convert_deg=False):
    pos = [0.0] * 7
    for ds_idx, joint_idx in mapping.items():
        value = float(action[ds_idx])
        if convert_deg or abs(value) > 4.0:
            value = np.deg2rad(value)
        pos[joint_idx] = value
    return pos


def _to_rad(value):
    value = float(value)
    return np.deg2rad(value) if abs(value) > 4.0 else value


def action_to_hand_cmd(action, hand_indices, use_named_hand, handside):
    fallback = 14 if handside == "left" else 20
    if use_named_hand:
        vec = [_to_rad(action[i]) for i in hand_indices]
    else:
        vec = [_to_rad(v) for v in action[fallback:fallback + 6]]

    return vec[:6]


class SimpleLeRobotDataset:
    def __init__(self, root, episodes=None):
        """
        root: lerobot dataset root
        episodes: [episode_idx]
        """
        self.root = Path(root)
        self.episodes = episodes

        self._load_info()
        self._load_features()
        self.hf_dataset = self._load_hf_dataset()

        if self.episodes is not None:
            self.hf_dataset = self.hf_dataset.filter(
                lambda x: x["episode_index"] in self.episodes
            )

    def _load_info(self):
        info_path = self.root / "meta" / "info.json"
        with open(info_path, "r") as f:
            self.info = json.load(f)
        self._fps = self.info["fps"]

    def _load_features(self):
        self._features = self.info["features"]

    def _load_hf_dataset(self):
        data_dir = self.root / "data"
        parquet_files = list(data_dir.rglob("*.parquet"))

        if not parquet_files:
            raise RuntimeError("No parquet files found in dataset")

        tables = []
        for parquet_file in parquet_files:
            tables.append(pq.read_table(parquet_file))

        table = tables[0].combine_chunks()
        for chunk_table in tables[1:]:
            table = table.append_table(chunk_table)

        return Dataset(table)

    @property
    def fps(self):
        return self._fps

    @property
    def features(self):
        return self._features

    def __len__(self):
        return len(self.hf_dataset)
