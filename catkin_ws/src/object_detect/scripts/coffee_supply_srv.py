#!/usr/bin/env python3
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
import rospy
import math
import rospkg, os
from object_detect.srv import DetectCoffeeSupply, DetectCoffeeSupplyResponse

# =============================================================================
# Class: RealSenseCamera
# =============================================================================
class RealSenseCamera:
    def __init__(self):
        self.pipeline = rs.pipeline()
        self.started = False
        try:
            config = rs.config()
            config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)
            self.pipeline.start(config)
            self.started = True
            rospy.loginfo("RealSense Camera Initialized (RGB only).")
        except Exception as e:
            rospy.logerr(f"[Camera] Failed to start RealSense pipeline: {e}")

    def get_color_image(self):
        if not self.started:
            return None
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return None
        return np.asanyarray(color_frame.get_data())

    def stop(self):
        if self.started:
            try:
                self.pipeline.stop()
                rospy.loginfo("RealSense Camera Stopped.")
            except Exception as e:
                rospy.logwarn(f"[Camera] Stop pipeline warning: {e}")
            self.started = False



# =============================================================================
# Class: ObjectDetector
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
        h, w = depth_frame.get_height(), depth_frame.get_width()
        for du in range(-half_size, half_size + 1):
            for dv in range(-half_size, half_size + 1):
                x, y = int(u) + du, int(v) + dv
                if 0 <= x < w and 0 <= y < h:
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

    def detect(self, img_color):
        try:
            results = self.model.predict([img_color], conf=self.conf_thresh, verbose=False)
        except Exception as e:
            rospy.logerr(f"[YOLO] Inference failed: {e}")
            return []

        detections = []
        for result in results:
            if not result.boxes or result.boxes.xyxy is None:
                continue
            boxes = result.boxes.xyxy.cpu().numpy()
            confs = result.boxes.conf.cpu().numpy()
            classes = result.boxes.cls.cpu().numpy().astype(int)
            names = result.names
            for i, box in enumerate(boxes):
                ux, uy = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2
                detections.append({
                    'class_id': int(classes[i]),
                    'class_name': self._safe_class_name(names, int(classes[i])),
                    'conf': round(float(confs[i]),3),
                    'xy': (round(ux,1), round(uy,1)),
                    'xyz': (round(ux,1), round(uy,1), 0.0)  # z 設為 0
                })
        return detections



# =============================================================================
# Class: App
# =============================================================================
class App:
    def __init__(self, model_path, cup_color_model_path):
        rospy.loginfo("Initializing Coffee Supply Server ......")
        self.detector = ObjectDetector(model_path)
        self.cup_color_detector = ObjectDetector(cup_color_model_path)
        self.coffee_supply_service = rospy.Service('CoffeeSupply', DetectCoffeeSupply, self.coffee_command)
        self.positions = {}
        self.camera = None

    # 相對位置
    def get_relative_position(self, from_name, to_name):
        from_pos = self.positions.get(from_name.lower())
        to_pos = self.positions.get(to_name.lower())
        if from_pos is None or to_pos is None:
            return None
        return tuple(round(to_pos[i] - from_pos[i], 3) for i in range(3))

    def get_all_relative(self):
        tree_to_target = self.get_relative_position('tree', 'black') or self.get_relative_position('tree', 'white')
        home_to_target = self.get_relative_position('home', 'black') or self.get_relative_position('home', 'white')
        return {'tree_to_target': tree_to_target, 'home_to_target': home_to_target}

    def coffee_command(self, req):
        self.camera = RealSenseCamera()
        if not self.camera.started:
            rospy.logwarn("Camera not initialized or failed to start.")
            return DetectCoffeeSupplyResponse(success=False, target_name="", table=0, cup_side="")

        self.positions = {}
        detections_all = []
        try:
            # --- 多幀咖啡牌偵測 ---
            for _ in range(5):
                img_color = self.camera.get_color_image()
                if img_color is None:
                    rospy.sleep(0.02)
                    continue
                detections_all.extend(self.detector.detect(img_color))
                rospy.sleep(0.05)

            if not detections_all:
                rospy.loginfo("No detections in captured frames.")
                return DetectCoffeeSupplyResponse(success=False, target_name="", table=0, cup_side="")

            # --- 分群 ---
            object_groups = {}
            distance_threshold = 50  # 用像素距離
            for det in detections_all:
                name = det['class_name'].lower()
                xy = np.array(det['xy'])  # 只用影像座標
                group_list = object_groups.setdefault(name, [])
                for group in group_list:
                    if np.linalg.norm(np.median(group, axis=0) - xy) < distance_threshold:
                        group.append(xy)
                        break
                else:
                    group_list.append([xy])

            # --- 選出 target ---
            target_name, target_xy = None, None
            for candidate in ['black', 'white']:
                if candidate in object_groups:
                    groups = object_groups[candidate]
                    largest_group = max(groups, key=len)
                    target_name = candidate
                    target_xy = np.median(np.array(largest_group), axis=0)
                    break

            if not target_name:
                rospy.loginfo("No coffee (black/white) group found.")
                return DetectCoffeeSupplyResponse(success=False, target_name="", table=0, cup_side="")

            self.positions[target_name] = tuple(target_xy)

            # --- landmark ---
            for lm in ['tree', 'home']:
                if lm in object_groups:
                    groups = object_groups[lm]
                    self.positions[lm] = tuple(np.median(np.array(max(groups, key=len)), axis=0))

            # --- 桌號判斷 ---
            rels = self.get_all_relative()
            def dist(vec): return round(np.linalg.norm(vec),1) if vec is not None else None
            tol = 20  # 影像座標容差
            def near(a,b): return a is not None and b is not None and abs(a-b)<tol

            dist_tree = dist(rels['tree_to_target'])
            dist_home = dist(rels['home_to_target'])
            table_lookup = {
                (160,70):1, (75,155):2, (205,150):3, (155,200):4  # 像素對應
            }
            table = 0
            for (dt, dh), t in table_lookup.items():
                if near(dist_tree, dt) and near(dist_home, dh):
                    table = t
                    break
            success = table != 0

            # --- 咖啡杯左右判斷 ---
            cup_positions = {}
            for _ in range(5):
                img_color = self.camera.get_color_image()
                if img_color is None:
                    rospy.sleep(0.02)
                    continue
                cup_dets = self.cup_color_detector.detect(img_color)
                for det in cup_dets:
                    name = det['class_name'].lower()
                    rel_x = det['xy'][0] - self.positions[target_name][0]  # 用影像座標 x
                    cup_positions[name] = 'left' if rel_x < 0 else 'right'
                rospy.sleep(0.03)

            white_side = cup_positions.get('white', 'unknown')
            black_side = cup_positions.get('black', 'unknown')
            rospy.loginfo(f"White coffee is on the {white_side} side and black coffee is on the {black_side} side")

            cup_side = cup_positions.get(target_name, "")

            return DetectCoffeeSupplyResponse(
                success=success,
                target_name=target_name,
                table=table,
                cup_side=cup_side
            )

        except Exception as e:
            rospy.logerr(f"[App] coffee_command exception: {e}")
            return DetectCoffeeSupplyResponse(success=False, target_name="", table=0, cup_side="")
        finally:
            if self.camera:
                self.camera.stop()

    def shutdown(self):
        if self.camera:
            self.camera.stop()
        rospy.loginfo("Coffee Supply Server Shutdown.")

# =============================================================================
# Main
# =============================================================================
if __name__ == '__main__':
    try:
        rospy.init_node('coffee_supply_server_node')
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('object_detect')
        model_path = os.path.join(pkg_path, 'scripts', 'coffee_supply.pt')
        cup_model_path = os.path.join(pkg_path, 'scripts', 'coffee.pt')
        server = App(model_path, cup_model_path)
        rospy.on_shutdown(server.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Failed to start Coffee Supply Server: {e}")