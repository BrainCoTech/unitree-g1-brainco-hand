from functools import partial

import yaml
from loguru import logger

from control_py.state_manager.common.lerobot_dataset import (
    ACTION,
    SimpleLeRobotDataset,
    detect_degree_dataset,
    filter_episode,
    map_dataset_action_to_arm_index,
)


class ReplayStatesHandler:
    # 读取回放数据集配置，确定回放文件和 episode。
    def _replay_config(self, dataset_idx: int):
        config_file = "src/control_py/control_py/state_manager/eeg/datasets_config.yaml"
        with open(config_file, "r") as f:
            datasets_cfg = yaml.safe_load(f)

        replay_params = datasets_cfg["replay"]
        datasets_folder = replay_params["DatasetFolder"]
        datasets_list_cfg = replay_params["ReplayDatasets"]

        selected_dataset = datasets_list_cfg.get(
            f"dataset{dataset_idx}",
            datasets_list_cfg["dataset0"],
        )

        self.episode_idx = int(selected_dataset["episode_idx"])
        self.dataset_path = f"{datasets_folder}/{selected_dataset['file']}"

        logger.debug(
            f"Replay dataset:\n file: {self.dataset_path}\n episode: {self.episode_idx}"
        )

    # 初始化回放数据集、时间步长和连续帧状态。
    def _replay_initialization(self):
        self.replay_dataset = SimpleLeRobotDataset(
            self.dataset_path,
            episodes=[self.episode_idx],
        )
        self.replay_frames = self.replay_dataset.hf_dataset.filter(
            partial(filter_episode, episode_idx=self.episode_idx)
        )
        if len(self.replay_frames) == 0:
            raise RuntimeError("No frames found")

        self.replay_actions = self.replay_frames.select_columns(ACTION)
        self.replay_fps = self.replay_dataset.fps
        self.replay_dt = 1.0 / self.replay_fps

        self.replay_frame_idx = 0
        self.replay_num_frames = len(self.replay_frames)
        self.replay_frame_pos = float(self.replay_frame_idx)

    # 构建回放数据的左右手臂动作索引映射。
    def _arm_action_mapping(self, action_names):
        self.left_arm_mapping, self.right_arm_mapping = {}, {}

        for i, name in enumerate(action_names):
            res = map_dataset_action_to_arm_index(name, "unitree")
            if res:
                side, joint_idx = res
                if side == "left":
                    self.left_arm_mapping[i] = joint_idx
                else:
                    self.right_arm_mapping[i] = joint_idx

        if not self.left_arm_mapping and not self.right_arm_mapping:
            raise ValueError(
                f"No mappable arm actions found in dataset. Action names: {action_names}"
            )

    # 构建回放数据的手指动作索引映射，并检测角度单位。
    def _hand_action_mapping(self, action_names):
        name_to_idx = {n: i for i, n in enumerate(action_names)}

        self.left_hand_indices = [name_to_idx.get(n) for n in self.hand.left_hand_joints]
        self.right_hand_indices = [name_to_idx.get(n) for n in self.hand.right_hand_joints]
        self.use_named_hand = all(
            i is not None for i in (self.left_hand_indices + self.right_hand_indices)
        )

        if not self.use_named_hand:
            missing = [
                n
                for n, i in zip(
                    self.hand.left_hand_joints + self.hand.right_hand_joints,
                    self.left_hand_indices + self.right_hand_indices,
                )
                if i is None
            ]
            logger.warning(
                "Some hand action names not found in dataset ACTION names, "
                f"will fallback to slicing [14:26]. Missing={missing}"
            )

        joint_indices = list(self.left_arm_mapping.keys()) + list(
            self.right_arm_mapping.keys()
        )
        self.convert_deg = detect_degree_dataset(
            self.replay_actions,
            joint_indices,
            ACTION,
        )

        logger.info(f"ReplayNode ready: {self.replay_num_frames} frames @ {self.replay_fps} FPS")
