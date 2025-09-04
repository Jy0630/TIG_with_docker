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
# 功能: 封裝 Intel RealSense 攝影機操作
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
# 功能: 封裝 YOLOv8 物件偵測與 3D 座標計算
# =============================================================================

class ObjectDetector:
    def __init__(self, model_path, conf_thresh=0.4, sample_size=5):
        self.model = YOLO(model_path)
        self.conf_thresh = conf_thresh
        self.sample_size = sample_size
        rospy.loginfo(f"Object Detector Initialized with model: {model_path}")

    def get_median_depth(self, depth_frame, u, v):
        half_size = self.sample_size // 2
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
            return []

        detections = []
        for result in results:
            if result.boxes is None or result.boxes.xyxy is None:
                continue

            boxes_xyxy = result.boxes.xyxy.cpu().numpy()
            confs = result.boxes.conf.cpu().numpy()
            classes = result.boxes.cls.cpu().numpy().astype(int)
            names = result.names

            for i in range(len(boxes_xyxy)):
                x1, y1, x2, y2 = boxes_xyxy[i]
                ux, uy = (x1 + x2) / 2.0, (y1 + y2) / 2.0

                surface_dis = self.get_median_depth(aligned_depth_frame, ux, uy)
                if surface_dis is None:
                    # 如果沒有深度就跳過（保留你目前對 coffee 準度的邏輯）
                    continue

                center_camera_xyz = rs.rs2_deproject_pixel_to_point(
                    depth_intrin, (int(ux), int(uy)), float(surface_dis)
                )
                center_camera_xyz = [round(v, 3) for v in center_camera_xyz]
                world_xyz = (center_camera_xyz[0], center_camera_xyz[2], -center_camera_xyz[1])

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
# 功能: 偵測咖啡，並提供 ROS Service 回傳
# =============================================================================

class App:
    def __init__(self, model_path):
        rospy.loginfo("Initializing Coffee Supply Server ......")
        self.detector = ObjectDetector(model_path)
        self.coffee_supply_service = rospy.Service(
            'CoffeeSupply', DetectCoffeeSupply, self.coffee_command
        )
        self.positions = {}

    # 計算相對位置
    def get_relative_position(self, from_name, to_name):
        from_pos = self.positions.get(from_name.lower())
        to_pos = self.positions.get(to_name.lower())
        if from_pos is None or to_pos is None:
            return None
        return tuple(round(to_pos[i] - from_pos[i], 3) for i in range(3))

    # 取得相對座標
    def get_all_relative(self):
        target = None
        for candidate in ['black', 'white']:
            if candidate in self.positions:
                target = candidate
                break
        if not target:
            return None
        return {
            'home_to_target': self.get_relative_position('home', target),
            'tree_to_target': self.get_relative_position('tree', target),
            'target_name': target
        }

    # ROS Service callback
    def coffee_command(self, req):
        self.camera = RealSenseCamera()
        if not self.camera or not self.camera.started:
            rospy.logwarn("Camera not initialized or failed to start.")
            return DetectCoffeeSupplyResponse(success=False, target_name="", table=0)

        # 每次呼叫清空上一輪結果
        self.positions = {}
        frames_to_capture = 20
        detections_all = []

        try:
            # 多幀抓取（同時保留最後一個有效的 depth_intrin/frames 以備 debug 或 fallback）
            last_depth_intrin = None
            last_img_color = None
            last_depth_frame = None

            for _ in range(frames_to_capture):
                depth_intrin, img_color, depth_frame = self.camera.get_aligned_images()
                if depth_intrin is None or img_color is None or depth_frame is None:
                    rospy.logwarn("Camera frame is invalid.")
                    rospy.sleep(0.02)
                    continue

                last_depth_intrin = depth_intrin
                last_img_color = img_color
                last_depth_frame = depth_frame

                detections = self.detector.detect(img_color, depth_frame, depth_intrin)
                # debug: 印出每幀偵測到的 class
                if detections:
                    names_in_frame = [d['class_name'] for d in detections]
                    rospy.logdebug(f"Frame detections: {names_in_frame}")
                detections_all.extend(detections)
                rospy.sleep(0.05)

            if not detections_all:
                rospy.loginfo("No detections in captured frames.")
                return DetectCoffeeSupplyResponse(success=False, target_name="", table=0)

            # 多幀合併並分群
            object_groups = {}
            distance_threshold = 0.05  # 5 cm
            for det in detections_all:
                name = det['class_name'].lower()
                xyz = np.array(det['xyz'])
                if name not in object_groups:
                    object_groups[name] = []

                placed = False
                for group in object_groups[name]:
                    group_center = np.median(np.array(group), axis=0)
                    if np.linalg.norm(group_center - xyz) < distance_threshold:
                        group.append(xyz)
                        placed = True
                        break
                if not placed:
                    object_groups[name].append([xyz])

            # debug: 列出有哪些類別被偵測到與每個類別群組數量
            for k, groups in object_groups.items():
                sizes = [len(g) for g in groups]
                rospy.loginfo(f"[Groups] class={k}, groups={len(groups)}, sizes={sizes}")

            # 選出最多幀的咖啡群組
            target_name = None
            target_xyz = None
            for candidate in ['black', 'white']:
                if candidate in object_groups:
                    groups = object_groups[candidate]
                    groups.sort(key=lambda g: len(g), reverse=True)
                    target_name = candidate
                    target_xyz = np.median(np.array(groups[0]), axis=0)
                    break

            if not target_name:
                rospy.loginfo("No coffee (black/white) group found.")
                return DetectCoffeeSupplyResponse(success=False, target_name="", table=0)

            # 更新 coffee 位置
            self.positions[target_name] = tuple(target_xyz)

            # ---------- 新增：同時把 tree 與 home 的位置也更新到 self.positions（若存在） ----------
            for landmark in ['tree', 'home']:
                if landmark in object_groups:
                    groups = object_groups[landmark]
                    groups.sort(key=lambda g: len(g), reverse=True)
                    lm_xyz = np.median(np.array(groups[0]), axis=0)
                    self.positions[landmark] = tuple(lm_xyz)
                    rospy.loginfo(f"Updated landmark {landmark}: {self.positions[landmark]}")
                else:
                    rospy.logdebug(f"Landmark {landmark} not found in grouped detections.")
            # ------------------------------------------------------------------------------

            rels = self.get_all_relative()
            if not rels:
                rospy.loginfo("Cannot find target or reference points.")
                return DetectCoffeeSupplyResponse(success=False, target_name=target_name, table=0)

            # 計算距離
            def calc_distance(vec):
                if vec is None:
                    return None
                return round(math.sqrt(sum(x ** 2 for x in vec)), 3)

            dist_tree = calc_distance(rels['tree_to_target'])
            dist_home = calc_distance(rels['home_to_target'])

            tol = 0.05
            def near(a, b):
                return (a is not None) and (b is not None) and (abs(a - b) < tol)

            rospy.loginfo(f"Distances: tree_to_target={dist_tree}, home_to_target={dist_home}")

            # 判斷桌號
            if near(dist_tree, 0.16) and near(dist_home, 0.07):
                table = 1
            elif near(dist_tree, 0.075) and near(dist_home, 0.155):
                table = 2
            elif near(dist_tree, 0.205) and near(dist_home, 0.15):
                table = 3
            elif near(dist_tree, 0.155) and near(dist_home, 0.2):
                table = 4
            else:
                rospy.loginfo("Supply card not matched with any table condition.")
                table = 0

            success = table != 0
            return DetectCoffeeSupplyResponse(success=success, target_name=target_name, table=table)

        except Exception as e:
            rospy.logerr(f"[App] coffee_command exception: {e}")
            return DetectCoffeeSupplyResponse(success=False, target_name="", table=0)

        finally:
            # 保持原本 behaviour：不論如何都停止 camera，或視需要改成不停止（若想一次只啟動一次）
            if self.camera:
                self.camera.stop()

    def shutdown(self):
        if self.camera:
            self.camera.stop()
        cv2.destroyAllWindows()
        rospy.loginfo("Coffee Supply Server Shutdown.")


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
