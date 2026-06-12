from loguru import logger
from control_py.utils.loguru_settings import setup_loguru, logger_with_params
setup_loguru(log_folder_path="log", show_on_terminal=True, terminal_level='DEBUG') 


import time, os

logger_with_params("Importing {}. Please wait...", "ultralytics YOLO", level="INFO")
from ultralytics import YOLO
logger.info(f"Import ultralytics YOLO Done.")

import pyrealsense2 as rs
import cv2
import numpy as np



def cross_(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    return np.cross(a, b)


def error_logging(err):
    while True:
        logger.error(err)
        time.sleep(1)


class Eye:

    def __init__(self, model_path, model_name, classes, classes_id, yolo_model=None):

        logger.info(f"Initializing Eye ...")

        self.depth_frame = None
        self.depth_intrin = None
        self.depth_image = None
        self.color_image = None

        self.obj_detect = False
        self.obj_property = {}
        self.obj_coordinate_camera_buffer = {}
        self.obj_coordinate_camera_avg = {}
        self.obj_border_buffer = {}
        self.obj_border_avg = {}
        self.obj_border_avg_stable = {}
        self.stop_thread_image_stream = False
        # https://gist.github.com/rcland12/dc48e1963268ff98c8b2c4543e7a9be8
        self.classes = classes
        self.classes_id = classes_id

        self.buffer_threshold_valid_border = 1 # 有效边界框 buffer > 3 帧
        self.buffer_threshold_valid_coord = 1  # 坐标值有效 > 2 帧

        # ===== Configure RealSense =====
        self.pipeline = rs.pipeline()
        config = rs.config()

        # ===== Set Device =====
        device_index = 0
        device = rs.context().devices[device_index].get_info(rs.camera_info.serial_number)
        config.enable_device(device)

        # ====== Enable Streams ======
        self.border_x, self.border_y, self.fps = 640, 360, 30 # 1280, 720, 30
        config.enable_stream(rs.stream.depth, self.border_x, self.border_y, rs.format.z16, self.fps)
        config.enable_stream(rs.stream.color, self.border_x, self.border_y, rs.format.bgr8, self.fps)

        # ====== Start Streams ======
        profile = self.pipeline.start(config)
        align_to = rs.stream.color
        self.align = rs.align(align_to)

        # ====== Get depth Scale ======
        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        # ====== Get depth Intrinsics ======
        depth_stream = profile.get_stream(rs.stream.depth)
        depth_intrinsics = depth_stream.as_video_stream_profile().get_intrinsics()

        # Load YOLO-Seg Model
        yolo_path = f'{model_path}{model_name}.pt'
        if yolo_model is not None:
            self.yolo_model = yolo_model
            logger.info(f"Reuse preloaded model {model_name}")
            logger.info("Eye Initialization Success")
        else:
            if not os.path.exists(yolo_path):
                logger.error(f"Model file not found: {yolo_path}")
                raise FileNotFoundError(yolo_path)

            try:
                self.yolo_model = YOLO(yolo_path)
                logger.info(f"Model {model_name} loaded successfully")
                logger.info("Eye Initialization Success")

            except Exception as e:
                logger.error(f"Failed to load YOLO model: {yolo_path}")
                raise

        # thread_image_stream = threading.Thread(target=lambda: self.image_stream())
        # thread_image_stream.start()

    def get_images(self):
        # 获取相机图像流
        frames = self.pipeline.wait_for_frames()  # 等待获取图像帧
        aligned_frames = self.align.process(frames)  # 获取对齐帧
        self.depth_frame = aligned_frames.get_depth_frame()  # 获取对齐帧中的depth帧
        color_frame = aligned_frames.get_color_frame()  # 获取对齐帧中的color帧

        # 相机参数的获取
        intr = color_frame.profile.as_video_stream_profile().intrinsics  # 获取相机内参
        self.depth_intrin = self.depth_frame.profile.as_video_stream_profile().intrinsics  # 获取深度参数（像素坐标系转相机坐标系会用到）
        '''camera_parameters = {'fx': intr.fx, 'fy': intr.fy,
                             'ppx': intr.ppx, 'ppy': intr.ppy,
                             'height': intr.height, 'width': intr.width,
                             'depth_scale': profile.get_device().first_depth_sensor().get_depth_scale()
                             }'''

        self.depth_image = np.asanyarray(self.depth_frame.get_data())  # 深度图（默认16位）
        self.color_image = np.asanyarray(color_frame.get_data())  # RGB图

    def image_stream(self):

        # 预先加载模型
        self.yolo_model.predict(np.ones((1, 1, 3)))
        cv2.namedWindow('detection', flags=cv2.WINDOW_NORMAL)

        try:
            while True:
                self.get_images()

                if not self.depth_image.any() or not self.color_image.any():
                    continue

                if self.obj_detect:
                    self.yolo_object_detection()

                cv2.imshow('detection', self.color_image)
                # Press esc or 'q' to close the image window
                key = cv2.waitKey(1)
                if key & 0xFF == ord('q') or key == 27:
                    cv2.destroyAllWindows()
                    break
        finally:
            self.pipeline.stop()

    def close_to_border(self, obj_border_points):
        # 判断物体边界是否靠近图像边框
        x_limit = self.border_x * 0.01
        y_limit = self.border_y * 0.01
        for point in obj_border_points:
            if x_limit < point[0] < (self.border_x - x_limit) and y_limit < point[1] < (self.border_y - y_limit):
                continue
            else:
                return True
        return False
    
    def obj_close_to_border(self, obj_border_points, obj_name: str = '') -> bool:
        """
        判断物体是否靠近图像边界，并打印具体位置
        """
        x_limit = self.border_x * 0.01
        y_limit = self.border_y * 0.01

        too_close = False

        for x, y in obj_border_points:
            if x <= x_limit:
                logger.warning(f"[{obj_name}] too close to LEFT  (x={x:.1f}, limit={x_limit:.1f})")
                too_close = True
            elif x >= self.border_x - x_limit:
                logger.warning(f"[{obj_name}] too close to RIGHT (x={x:.1f}, limit={self.border_x - x_limit:.1f})")
                too_close = True

            if y <= y_limit:
                logger.warning(f"[{obj_name}] too close to TOP   (y={y:.1f}, limit={y_limit:.1f})")
                too_close = True
            elif y >= self.border_y - y_limit:
                logger.warning(f"[{obj_name}] too close to BOTTOM(y={y:.1f}, limit={self.border_y - y_limit:.1f})")
                too_close = True

        return too_close

    def initialize_obj_coordinate(self, item_list):
        for item in item_list:
            if item in self.obj_coordinate_camera_avg.keys():
                print(f"------------------------------ Initialize obj vision -------------------------------")
                self.obj_coordinate_camera_buffer.pop(item, 0)
                self.obj_coordinate_camera_avg.pop(item, 0)
                self.obj_border_buffer.pop(item, 0)
                self.obj_border_avg.pop(item, 0)
                self.obj_border_avg_stable.pop(item, 0)

    def yolo_object_detection(self):
        
        # 使用 YOLOv11 进行目标检测
        results = self.yolo_model.predict(self.color_image, classes=self.classes_id, conf=0.7, device=0,
                                          agnostic_nms=True, retina_masks=True, verbose=False)[0]

        try:
            box_list = results.boxes.data.cpu().tolist()
            mask_list = results.masks.xy
        except:
            logger.info("No object")
            return

        self.color_image = results.plot()
        
        obj_property = {}
        for i, box in enumerate(box_list):
            index = int(box[5])
            name = results.names[index]
            conf = box[4]  # 置信度
            if name not in self.classes:
                print(f"{self.classes} not detected")
            else:
                print(f"Detected {name} with confidence {conf:.2f} at box {box[0:4]}")
                # only detect obj in classes
                obj_border = box[0:4].copy()
                x1, y1, x2, y2 = obj_border
                obj_border_points = [[x1, y1], [x1, y2], [x2, y1], [x2, y2]]

                if not self.obj_close_to_border(obj_border_points, name): # 目标不靠近边界
                    # 通过边界框计算中心点坐标
                    # ux = int((x1 + x2) / 2)
                    # uy = int((y1 + y2) / 2)

                    # 通过mask计算中心坐标
                    # moments = cv2.moments(mask_list[i][::-1])
                    # ux = int(moments['m10'] / moments['m00'])
                    # uy = int(moments['m01'] / moments['m00'])
                    # print(ux, uy)

                    # mask下边缘点往上一点的中心点作为判断点
                    bottom_point_ymax = mask_list[i][np.argsort(mask_list[i][:, 1])[-1]][1]
                    bottom_point_ymin = mask_list[i][np.argsort(mask_list[i][:, 1])[-10]][1]
                    index_ymin = np.argwhere(mask_list[i][:, 1] >= bottom_point_ymin)
                    index_ymin = [item for row in index_ymin for item in row]
                    bottom_points = mask_list[i][index_ymin]
                    bottom_point_xmax = bottom_points[np.argsort(bottom_points[:, 0])[-1]][0]
                    bottom_point_xmin = bottom_points[np.argsort(bottom_points[:, 0])[0]][0]
                    ux = int((bottom_point_xmax + bottom_point_xmin) / 2)
                    uy = int((bottom_point_ymax + bottom_point_ymin) / 2)

                    coordinate_camera = rs.rs2_deproject_pixel_to_point(self.depth_intrin, [ux, uy],
                                                                        self.depth_frame.get_distance(
                                                                            ux, uy))

                    formatted_coordinate_camera = f"({coordinate_camera[0]:.2f}, {coordinate_camera[1]:.2f}, {coordinate_camera[2]:.2f})"
                    # 展示检测界面
                    cv2.circle(self.color_image, (ux, uy), 4, (255, 255, 255), 5)
                    # cv2.putText(self.color_image, str(formatted_coordinate_camera), (ux + 20, uy + 10), 0, 1,
                    #             [225, 255, 255], thickness=2, lineType=cv2.LINE_AA)

                    obj_property.update({name: {'border': obj_border,
                                                'coordinate_camera': coordinate_camera,
                                                'move': False}})

        
        
        # delete not detected obj
        for item in self.classes:
            if item in self.obj_property.keys() and item not in obj_property.keys():
                self.obj_property.pop(item, 0)
                self.obj_border_buffer.pop(item, 0)
                self.obj_border_avg.pop(item, 0)
                self.obj_border_avg_stable.pop(item, 0)
                self.obj_coordinate_camera_buffer.pop(item, 0)
                self.obj_coordinate_camera_avg.pop(item, 0)

        self.obj_property = obj_property

        self.update_obj_border()

    def log_obj_states(self):

        states = []

        for obj_name in self.obj_property:

            if self.obj_coordinate_camera_avg.get(obj_name):
                state = "stable"

            elif obj_name in self.obj_coordinate_camera_buffer:
                buf_len = len(self.obj_coordinate_camera_buffer[obj_name])
                state = f"coord_check {buf_len}/{self.buffer_threshold_valid_coord}"

            elif obj_name in self.obj_border_avg:
                buf_len = len(self.obj_border_buffer.get(obj_name, []))
                state = f"border_check {buf_len}/{self.buffer_threshold_valid_border}"

            else:
                state = "detected"

            states.append(f"[{obj_name}] ({state})")

        logger.info(" | ".join(states))
    
    
    def update_obj_border(self):
        self.log_obj_states()
        for obj_name, obj_property in self.obj_property.items():
            obj_border = obj_property['border']
            if obj_name not in self.obj_border_buffer.keys():
                self.obj_border_buffer.update({obj_name: [[]]})
                self.obj_border_avg.update({obj_name: []})
            else:
                obj_border_buffer = self.obj_border_buffer[obj_name].copy()
                obj_border_buffer.append(obj_border)
                self.obj_border_buffer[obj_name] = obj_border_buffer

                buf_len = len(self.obj_border_buffer[obj_name])
                if buf_len <= self.buffer_threshold_valid_border:
                    # logger.debug(f"[{obj_name}] border_buf {buf_len}/{self.buffer_threshold_valid_border}")
                    pass
                else:
                    self.obj_border_buffer[obj_name].pop(0)
                    self.obj_border_avg[obj_name] = list(np.average(self.obj_border_buffer[obj_name], axis=0))

                    if obj_name not in self.obj_border_avg_stable.keys():
                        self.obj_border_avg_stable.update({obj_name: self.obj_border_avg[obj_name]})
                        self.update_obj_coordinate_camera_avg(obj_name)
                        # print("update!")
                    else:
                        if np.min(np.abs(np.array(self.obj_border_avg[obj_name]) - np.array(
                                self.obj_border_avg_stable[obj_name]))) > 1:
                            # 实时边界框和稳定边界框的最小差大于某个数判断为物体运动
                            self.obj_property[obj_name]['move'] = True
                            # print(f"{obj_name} move! distance change: {np.min(np.abs(np.array(self.obj_border_avg[obj_name]) - np.array(self.obj_border_avg_stable[obj_name])))}")
                            self.obj_border_avg.pop(obj_name, 0)
                            self.obj_border_buffer.pop(obj_name, 0)
                            self.obj_border_avg_stable.pop(obj_name, 0)
                            self.obj_coordinate_camera_buffer.pop(obj_name, 0)
                            self.obj_coordinate_camera_avg.pop(obj_name, 0)
                        else:
                            # obj steadily not move, update coordinate buffer and avg
                            if np.max(np.std(self.obj_border_buffer[obj_name], axis=0)) <= 1:
                                # 边界buffer最大标准差小于某个数判断为物体为稳定
                                if not self.obj_coordinate_camera_avg[obj_name]:
                                    # 坐标未更新完
                                    self.update_obj_coordinate_camera_avg(obj_name)
                                else:
                                    self.obj_property[obj_name]['move'] = False

    def update_obj_coordinate_camera_avg(self, obj_name):
        obj_property = self.obj_property[obj_name]
        obj_coordinate_camera = obj_property['coordinate_camera']

        if obj_name not in self.obj_coordinate_camera_buffer.keys():
            self.obj_coordinate_camera_buffer.update({obj_name: [[]]})
            self.obj_coordinate_camera_avg.update({obj_name: []})
        else:
            if max(np.abs(obj_coordinate_camera)) > 0:
                obj_coordinate_camera_buffer = self.obj_coordinate_camera_buffer[obj_name].copy()
                obj_coordinate_camera_buffer.append(obj_coordinate_camera)
                self.obj_coordinate_camera_buffer[obj_name] = obj_coordinate_camera_buffer

                buf_len = len(self.obj_coordinate_camera_buffer[obj_name])
                if buf_len > self.buffer_threshold_valid_coord:
                    self.obj_coordinate_camera_buffer[obj_name].pop(0)
                    self.obj_coordinate_camera_avg[obj_name] = list(np.average(self.obj_coordinate_camera_buffer[obj_name], axis=0))
                    # logger.info(f"[{obj_name}] coord_avg updated: {self.obj_coordinate_camera_avg[obj_name]}")


if __name__ == '__main__':
    eye = Eye()
    eye.obj_detect = True
    while True:
        # print(eye.obj_coordinate_camera_avg)
        time.sleep(1. / 30)
