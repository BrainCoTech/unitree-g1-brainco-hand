from loguru import logger
from control_py.utils.loguru_settings import setup_loguru, logger_with_params
setup_loguru(log_folder_path="log", show_on_terminal=True, terminal_level='DEBUG')


logger_with_params("Importing {}. Please wait...", "MediaPipe", level="INFO")
import mediapipe as mp
logger.info("Import MediaPipe Done.")

import time
import cv2
import numpy as np
import pyrealsense2 as rs


class VisionHand:
    def __init__(self):
        logger.info("Initializing VisionHand ...")

        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles

        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            model_complexity=1,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        )

        self.buffer_threshold_valid_gesture = 5  # 有效手势 buffer > 3 帧
        self.buffer_threshold_valid_coord = 3    # 坐标值有效 > 2 帧
        self.gesture_buffer = []
        self.gesture_avg_stable = None
        self.gesture_stable_start_time = None
        self.gesture_logged_stable_second = 0
        self.last_hand_detected = False
        self.last_gesture_name = None
        self.hand_coordinate_camera_buffer = []
        self.hand_coordinate_camera_avg = None
        self.last_hand_pixel = None

        logger.info("VisionHand Initialization Success")

    def close(self):
        if self.hands is not None:
            self.hands.close()
            self.hands = None

    def reset_gesture_buffer(self):
        self.gesture_buffer = []
        self.gesture_avg_stable = None
        self.gesture_stable_start_time = None
        self.gesture_logged_stable_second = 0
        self.last_gesture_name = None

    def reset_hand_coordinate(self):
        self.hand_coordinate_camera_buffer = []
        self.hand_coordinate_camera_avg = None
        self.last_hand_pixel = None

    @staticmethod
    def _distance(point_a, point_b) -> float:
        return np.hypot(point_a.x - point_b.x, point_a.y - point_b.y)

    @staticmethod
    def _clamp_pixel(value: int, upper: int) -> int:
        return max(0, min(value, upper - 1))

    def _thumb_open(self, landmarks, handedness_label: str) -> bool:
        wrist = landmarks[0]
        thumb_mcp = landmarks[2]
        thumb_ip = landmarks[3]
        thumb_tip = landmarks[4]
        index_mcp = landmarks[5]
        pinky_mcp = landmarks[17]

        palm_center_x = np.mean([wrist.x, index_mcp.x, pinky_mcp.x])
        tip_farther_from_palm = abs(thumb_tip.x - palm_center_x) > abs(thumb_ip.x - palm_center_x)
        tip_farther_from_index = self._distance(thumb_tip, index_mcp) > self._distance(thumb_ip, index_mcp)
        tip_farther_from_wrist = self._distance(thumb_tip, wrist) > self._distance(thumb_mcp, wrist)

        if handedness_label == "Left":
            thumb_direction_open = thumb_tip.x >= thumb_ip.x - 0.01
        elif handedness_label == "Right":
            thumb_direction_open = thumb_tip.x <= thumb_ip.x + 0.01
        else:
            thumb_direction_open = True

        return tip_farther_from_palm and tip_farther_from_index and tip_farther_from_wrist and thumb_direction_open

    def _finger_open(self, landmarks, tip_idx: int, pip_idx: int, mcp_idx: int) -> bool:
        wrist = landmarks[0]
        tip = landmarks[tip_idx]
        pip = landmarks[pip_idx]
        mcp = landmarks[mcp_idx]

        tip_above_pip = tip.y < pip.y
        tip_farther_from_wrist = self._distance(tip, wrist) > self._distance(pip, wrist)
        pip_farther_than_mcp = self._distance(pip, wrist) >= self._distance(mcp, wrist) * 0.95

        return (tip_above_pip and tip_farther_from_wrist) or (tip_farther_from_wrist and pip_farther_than_mcp)

    def _finger_folded(self, landmarks, tip_idx: int, pip_idx: int, mcp_idx: int) -> bool:
        wrist = landmarks[0]
        tip = landmarks[tip_idx]
        pip = landmarks[pip_idx]
        mcp = landmarks[mcp_idx]

        tip_below_pip = tip.y >= pip.y - 0.01
        tip_not_farther_from_wrist = self._distance(tip, wrist) <= self._distance(pip, wrist) * 1.05
        tip_close_to_mcp = self._distance(tip, mcp) < self._distance(pip, mcp) * 1.15

        return tip_below_pip and tip_not_farther_from_wrist and tip_close_to_mcp

    def _thumb_folded(self, landmarks, handedness_label: str) -> bool:
        wrist = landmarks[0]
        thumb_tip = landmarks[4]
        thumb_ip = landmarks[3]
        index_mcp = landmarks[5]
        pinky_mcp = landmarks[17]

        palm_center_x = np.mean([wrist.x, index_mcp.x, pinky_mcp.x])
        tip_near_palm = abs(thumb_tip.x - palm_center_x) <= abs(thumb_ip.x - palm_center_x) + 0.015
        tip_not_farther_from_wrist = self._distance(thumb_tip, wrist) <= self._distance(thumb_ip, wrist) * 1.05

        if handedness_label == "Left":
            thumb_wrapped = thumb_tip.x <= thumb_ip.x + 0.01
        elif handedness_label == "Right":
            thumb_wrapped = thumb_tip.x >= thumb_ip.x - 0.01
        else:
            thumb_wrapped = True

        return tip_near_palm and tip_not_farther_from_wrist and thumb_wrapped

    def _is_fist(self, hand_landmarks, handedness_label: str) -> bool:
        landmarks = hand_landmarks.landmark

        folded_fingers = [
            self._finger_folded(landmarks, 8, 6, 5),
            self._finger_folded(landmarks, 12, 10, 9),
            self._finger_folded(landmarks, 16, 14, 13),
            self._finger_folded(landmarks, 20, 18, 17),
        ]
        thumb_folded = self._thumb_folded(landmarks, handedness_label)

        return sum(folded_fingers) >= 3 and thumb_folded

    def _is_five_fingers_open(self, hand_landmarks, handedness_label: str) -> bool:
        landmarks = hand_landmarks.landmark

        if self._is_fist(hand_landmarks, handedness_label):
            return False

        open_count = sum([
            self._thumb_open(landmarks, handedness_label),
            self._finger_open(landmarks, 8, 6, 5),
            self._finger_open(landmarks, 12, 10, 9),
            self._finger_open(landmarks, 16, 14, 13),
            self._finger_open(landmarks, 20, 18, 17),
        ])

        # 放宽条件：只要不是握拳，并且至少有一根手指明显伸出，就视为目标手势
        return open_count >= 1

    def _get_palm_center_pixel(self, hand_landmarks, image_width: int, image_height: int):
        landmarks = hand_landmarks.landmark
        palm_indices = [0, 5, 9, 13, 17]
        ux = int(np.mean([landmarks[idx].x for idx in palm_indices]) * image_width)
        uy = int(np.mean([landmarks[idx].y for idx in palm_indices]) * image_height)

        ux = self._clamp_pixel(ux, image_width)
        uy = self._clamp_pixel(uy, image_height)
        return ux, uy

    def _get_valid_depth_distance(self, depth_frame, ux: int, uy: int, image_width: int, image_height: int):
        if depth_frame is None:
            return 0.0

        distances = []
        for radius in range(4):
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    px = self._clamp_pixel(ux + dx, image_width)
                    py = self._clamp_pixel(uy + dy, image_height)
                    distance = depth_frame.get_distance(px, py)
                    if distance > 0:
                        distances.append(distance)
            if distances:
                break

        if not distances:
            return 0.0

        return float(np.median(distances))

    def _update_hand_coordinate_camera_avg(self, coordinate_camera):
        if coordinate_camera is None or max(np.abs(coordinate_camera)) <= 0:
            self.reset_hand_coordinate()
            return

        self.hand_coordinate_camera_buffer.append(coordinate_camera)
        if len(self.hand_coordinate_camera_buffer) > self.buffer_threshold_valid_coord:
            self.hand_coordinate_camera_buffer.pop(0)
            self.hand_coordinate_camera_avg = list(np.average(self.hand_coordinate_camera_buffer, axis=0))

    def _locate_hand_coordinate(self, hand_landmarks, color_image, depth_frame, depth_intrin):
        if color_image is None or depth_frame is None or depth_intrin is None:
            self.reset_hand_coordinate()
            return

        image_height, image_width = color_image.shape[:2]
        ux, uy = self._get_palm_center_pixel(hand_landmarks, image_width, image_height)
        distance = self._get_valid_depth_distance(depth_frame, ux, uy, image_width, image_height)

        if distance <= 0:
            self.reset_hand_coordinate()
            return

        coordinate_camera = rs.rs2_deproject_pixel_to_point(depth_intrin, [ux, uy], distance)
        self.last_hand_pixel = [ux, uy]
        self._update_hand_coordinate_camera_avg(coordinate_camera)

        cv2.circle(color_image, (ux, uy), 5, (255, 255, 255), 3)
        if self.hand_coordinate_camera_avg is not None:
            coord_text = str(np.round(self.hand_coordinate_camera_avg, 3).tolist())
            cv2.putText(
                color_image,
                coord_text,
                (ux + 10, uy + 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )

    def _update_gesture_buffer(self, gesture_name):
        if gesture_name is None:
            self.reset_gesture_buffer()
            return None

        if self.gesture_buffer and self.gesture_buffer[-1] != gesture_name:
            self.gesture_buffer = [gesture_name]
            self.gesture_stable_start_time = time.monotonic()
            self.gesture_logged_stable_second = 0
        else:
            if not self.gesture_buffer:
                self.gesture_stable_start_time = time.monotonic()
                self.gesture_logged_stable_second = 0
            self.gesture_buffer.append(gesture_name)

        if self.gesture_stable_start_time is not None:
            stable_second = int(time.monotonic() - self.gesture_stable_start_time)
            while stable_second > self.gesture_logged_stable_second:
                self.gesture_logged_stable_second += 1
                logger.debug(f"稳定{self.gesture_logged_stable_second}s")

        if len(self.gesture_buffer) > self.buffer_threshold_valid_gesture:
            self.gesture_avg_stable = gesture_name
            return gesture_name

        return None

    def recognize_gesture(self, color_image, depth_frame=None, depth_intrin=None):
        if color_image is None:
            self.last_hand_detected = False
            self.reset_gesture_buffer()
            self.reset_hand_coordinate()
            return None

        rgb_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        rgb_image.flags.writeable = False
        results = self.hands.process(rgb_image)
        rgb_image.flags.writeable = True

        gesture_name = None
        self.last_hand_detected = bool(results.multi_hand_landmarks)
        selected_hand_landmarks = None

        if results.multi_hand_landmarks:
            handedness_list = results.multi_handedness or []
            for index, hand_landmarks in enumerate(results.multi_hand_landmarks):
                handedness_label = "Unknown"
                if index < len(handedness_list):
                    handedness_label = handedness_list[index].classification[0].label

                self.mp_drawing.draw_landmarks(
                    color_image,
                    hand_landmarks,
                    self.mp_hands.HAND_CONNECTIONS,
                    self.mp_drawing_styles.get_default_hand_landmarks_style(),
                    self.mp_drawing_styles.get_default_hand_connections_style(),
                )

                if self._is_five_fingers_open(hand_landmarks, handedness_label):
                    gesture_name = "五指张开"
                    selected_hand_landmarks = hand_landmarks
                    break
        else:
            self.reset_hand_coordinate()

        if selected_hand_landmarks is not None:
            self._locate_hand_coordinate(selected_hand_landmarks, color_image, depth_frame, depth_intrin)
        else:
            self.reset_hand_coordinate()

        self.last_gesture_name = gesture_name
        stable_gesture = self._update_gesture_buffer(gesture_name)

        if gesture_name is not None:
            cv2.putText(
                color_image,
                gesture_name,
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

        return stable_gesture
