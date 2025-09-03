#!/usr/bin/env python3
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
import rospy
import math
import rospkg, os
import cv2
from object_detect.srv import DetectCoffeeSupply, DetectCoffeeSupplyResponse


# =============================================================================
# Class: RealSenseCamera
# 目的: 封裝所有與 Intel RealSense 攝影機相關的操作。
# =============================================================================

class RealSenseCamera:
    def __init__(self):
        self.pipeline = rs.pipeline()
        self.align = None
        self.started = False
        try:
            config = rs.config()
            config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
            config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)
            self.pipeline.start(config)
            self.align = rs.align(rs.stream.color)
            self.started = True
            rospy.loginfo("RealSense Camera Initialized.")
        except Exception as e:
            rospy.logerr(f"[Camera] Failed to start RealSense pipeline: {e}")
            self.started = False

    def get_aligned_images(self):
        if not self.started:
            return None, None, None
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        if not depth_frame or not color_frame:
            return None, None, None
        depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
        img_color = np.asanyarray(color_frame.get_data())
        return depth_intrin, img_color, depth_frame

    def stop(self):
        try:
            if self.started:
                self.pipeline.stop()
                self.started = False
                rospy.loginfo("RealSense Camera Stopped.")
        except Exception as e:
            rospy.logwarn(f"[Camera] Stop pipeline warning: {e}")


# =============================================================================
# Class: ObjectDetector
# 目的: 封裝 YOLOv8 物件偵測和 3D 座標計算相關邏輯
# =============================================================================

class ObjectDetector:
    def __init__(self, model_path, conf_thresh=0.65):
        self.model = YOLO(model_path)
        self.conf_thresh = conf_thresh
        rospy.loginfo(f"Object Detector Initialized with model: {model_path}")

    def get_median_depth(self, depth_frame, u, v, sample_size=5):
        half_size = sample_size // 2
        depth_values = []
        height, width = depth_frame.get_height(), depth_frame.get_width()
        for du in range(-half_size, half_size + 1):
            for dv in range(-half_size, half_size + 1):
                x, y = int(u) + du, int(v) + dv
                if 0 <= x < width and 0 <= y < height:
                    d = depth_frame.get_distance(x, y)
                    if d > 0:
                        depth_values.append(d)
        return float(np.median(depth_values)) if depth_values else None

    def _safe_class_name(self, names, class_id):
        if isinstance(names, dict):
            return names.get(class_id, str(class_id))
        if isinstance(names, list):
            return names[class_id] if 0 <= class_id < len(names) else str(class_id)
        return str(class_id)

    def detect(self, img_color, aligned_depth_frame, depth_intrin):
        try:
            results = self.model.predict([img_color], conf=self.conf_thresh, verbose=False)
        except Exception as e:
            rospy.logerr(f"[YOLO] Inference failed: {e}")
            return [], []  # 回傳空結果

        detections = []

        for result in results:
            if result.boxes is None or result.boxes.xyxy is None:
                continue

            boxes_xyxy = result.boxes.xyxy.cpu().numpy()
            confs = result.boxes.conf.cpu().numpy()
            classes = result.boxes.cls.cpu().numpy().astype(int)
            names = result.names

            for i in range(len(boxes_xyxy)):
                box = boxes_xyxy[i]
                x1, y1, x2, y2 = box
                ux = (x1 + x2) / 2.0
                uy = (y1 + y2) / 2.0

                surface_dis = self.get_median_depth(aligned_depth_frame, ux, uy)
                if surface_dis is None:
                    continue

                center_camera_xyz = rs.rs2_deproject_pixel_to_point(
                    depth_intrin, (int(ux), int(uy)), float(surface_dis)
                )
                center_camera_xyz = [round(v, 3) for v in center_camera_xyz]
                world_x = center_camera_xyz[0]
                world_y = center_camera_xyz[2]
                world_z = -center_camera_xyz[1]
                world_xyz = (world_x, world_y, world_z)

                class_id = int(classes[i])
                class_name = self._safe_class_name(names, class_id)
                conf = float(confs[i])

                detections.append({
                    'class_id': class_id,
                    'class_name': class_name,
                    'conf': round(conf, 3),
                    'xyz': tuple(world_xyz)
                })

        return detections


# =============================================================================
# Class: App
# 目的：偵測咖啡並回傳 Service
# =============================================================================

class App:
    def __init__(self, model_path):
        rospy.loginfo("Initializing Coffee Supply Server ......")
        self.detector = ObjectDetector(model_path)
        self.coffee_supply_service = rospy.Service('CoffeeSupply', DetectCoffeeSupply, self.coffee_command)
        rospy.loginfo("Service 'CoffeeSupply' is ready.")
        self.positions = {}

    def get_relative_position(self, from_name, to_name):
        from_pos = self.positions.get(from_name.lower())
        to_pos = self.positions.get(to_name.lower())
        if from_pos is None or to_pos is None:
            return None
        return tuple(round(to_pos[i] - from_pos[i], 3) for i in range(3))

    def get_all_relative(self):
        target = None
        if 'black' in self.positions:
            target = 'black'
        elif 'white' in self.positions:
            target = 'white'
        else:
            return None
        return {
            'home_to_target': self.get_relative_position('home', target),
            'tree_to_target': self.get_relative_position('tree', target),
            'target_name': target
        }

    def coffee_command(self, req):
        camera = None
        try:
            camera = RealSenseCamera()
            if not camera.started:
                rospy.logwarn("Camera failed to start.")
                return DetectCoffeeSupplyResponse(success=False, target_name="", table="")

            depth_intrin, img_color, depth_frame = camera.get_aligned_images()
            if depth_intrin is None or img_color is None or depth_frame is None:
                rospy.logwarn("Camera frame is invalid.")
                return DetectCoffeeSupplyResponse(success=False, target_name="", table="")

            detections = self.detector.detect(img_color, depth_frame, depth_intrin)

            for det in detections:
                self.positions[det['class_name'].lower()] = det['xyz']

            rels = self.get_all_relative()
            if not rels:
                rospy.loginfo("Cannot find the supply card (missing 'home'/'tree' or target).")
                return DetectCoffeeSupplyResponse(success=False, target_name="", table="")

            def calc_distance(vec):
                if vec is None:
                    return None
                return round(math.sqrt(sum(x**2 for x in vec)), 3)

            dist_tree = calc_distance(rels['tree_to_target'])
            dist_home = calc_distance(rels['home_to_target'])

            tol = 0.03
            def near(a, b):
                return (a is not None) and (b is not None) and (abs(a - b) < tol)

            if near(dist_tree, 0.16) and near(dist_home, 0.07):
                return DetectCoffeeSupplyResponse(True, rels['target_name'], 1)
            elif near(dist_tree, 0.075) and near(dist_home, 0.155):
                return DetectCoffeeSupplyResponse(True, rels['target_name'], 2) if                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           
            elif near(dist_tree, 0.205) and near(dist_home, 0.15):
                return DetectCoffeeSupplyResponse(True, rels['target_name'], 3)
            elif near(dist_tree, 0.155) and near(dist_home, 0.2):
                return DetectCoffeeSupplyResponse(True, rels['target_name'], 4)
            else:
                rospy.loginfo("Supply card not matched with any table condition.")
                return DetectCoffeeSupplyResponse(success=False, target_name=rels['target_name'], table="")

        except Exception as e:
            rospy.logerr(f"[App] coffee_command exception: {e}")
            return DetectCoffeeSupplyResponse(success=False, target_name="", table="")
        finally:
            if camera:
                camera.stop()

    def shutdown(self):
        if self.camera:
            self.camera.stop()
        cv2.destroyAllWindows()
        rospy.loginfo("Coffee Taker Server Shutdown.")

if __name__ == '__main__':
    try:
        rospy.init_node('coffee_supply_server_node')
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('object_detect')
        model_path = os.path.join(package_path, 'scripts', 'coffee_supply.pt')
        server = App(model_path)
        rospy.on_shutdown(server.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Failed to start Coffee Supply Server: {e}")