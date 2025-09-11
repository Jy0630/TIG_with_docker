# #!/usr/bin/env python3
# import numpy as np
# import pyrealsense2 as rs
# from ultralytics import YOLO
# import rospy
# import math
# import rospkg, os
# from object_detect.srv import DetectCoffeeSupply, DetectCoffeeSupplyResponse

# # =============================================================================
# # Class: RealSenseCamera
# # =============================================================================
# class RealSenseCamera:
#     def __init__(self):
#         self.pipeline = rs.pipeline()
#         self.align = None
#         self.started = False
#         try:
#             config = rs.config()
#             config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
#             config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)
#             self.pipeline.start(config)
#             self.align = rs.align(rs.stream.color)
#             self.started = True
#             rospy.loginfo("RealSense Camera Initialized.")
#         except Exception as e:
#             rospy.logerr(f"[Camera] Failed to start RealSense pipeline: {e}")

#     def get_aligned_images(self):
#         if not self.started:
#             return None, None, None
#         frames = self.pipeline.wait_for_frames()
#         aligned_frames = self.align.process(frames)
#         depth_frame = aligned_frames.get_depth_frame()
#         color_frame = aligned_frames.get_color_frame()
#         if not depth_frame or not color_frame:
#             return None, None, None
#         depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
#         img_color = np.asanyarray(color_frame.get_data())
#         return depth_intrin, img_color, depth_frame

#     def stop(self):
#         if self.started:
#             try:
#                 self.pipeline.stop()
#                 rospy.loginfo("RealSense Camera Stopped.")
#             except Exception as e:
#                 rospy.logwarn(f"[Camera] Stop pipeline warning: {e}")
#             self.started = False

# # =============================================================================
# # Class: ObjectDetector
# # =============================================================================
# class ObjectDetector:
#     def __init__(self, model_path, conf_thresh=0.4, sample_size=5):
#         self.model = YOLO(model_path)
#         self.conf_thresh = conf_thresh
#         self.sample_size = sample_size
#         rospy.loginfo(f"Object Detector Initialized with model: {model_path}")

#     def get_median_depth(self, depth_frame, u, v):
#         half_size = self.sample_size // 2
#         depth_values = []
#         h, w = depth_frame.get_height(), depth_frame.get_width()
#         for du in range(-half_size, half_size + 1):
#             for dv in range(-half_size, half_size + 1):
#                 x, y = int(u) + du, int(v) + dv
#                 if 0 <= x < w and 0 <= y < h:
#                     d = depth_frame.get_distance(x, y)
#                     if d > 0:
#                         depth_values.append(d)
#         return float(np.median(depth_values)) if depth_values else None

#     def _safe_class_name(self, names, class_id):
#         if isinstance(names, dict):
#             return names.get(class_id, str(class_id))
#         if isinstance(names, list):
#             return names[class_id] if 0 <= class_id < len(names) else str(class_id)
#         return str(class_id)

#     def detect(self, img_color, aligned_depth_frame, depth_intrin):
#         try:
#             results = self.model.predict([img_color], conf=self.conf_thresh, verbose=False)
#         except Exception as e:
#             rospy.logerr(f"[YOLO] Inference failed: {e}")
#             return []

#         detections = []
#         for result in results:
#             if not result.boxes or result.boxes.xyxy is None:
#                 continue
#             boxes = result.boxes.xyxy.cpu().numpy()
#             confs = result.boxes.conf.cpu().numpy()
#             classes = result.boxes.cls.cpu().numpy().astype(int)
#             names = result.names
#             for i, box in enumerate(boxes):
#                 ux, uy = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2
#                 depth = self.get_median_depth(aligned_depth_frame, ux, uy)
#                 if depth is None:
#                     continue
#                 xyz_camera = rs.rs2_deproject_pixel_to_point(depth_intrin, (int(ux), int(uy)), float(depth))
#                 world_xyz = (round(xyz_camera[0],3), round(xyz_camera[2],3), round(-xyz_camera[1],3))
#                 detections.append({
#                     'class_id': int(classes[i]),
#                     'class_name': self._safe_class_name(names, int(classes[i])),
#                     'conf': round(float(confs[i]),3),
#                     'xyz': world_xyz
#                 })
#         return detections

# # =============================================================================
# # Class: App
# # =============================================================================
# class App:
#     def __init__(self, model_path, cup_color_model_path):
#         rospy.loginfo("Initializing Coffee Supply Server ......")
#         self.detector = ObjectDetector(model_path)
#         self.cup_color_detector = ObjectDetector(cup_color_model_path)
#         self.coffee_supply_service = rospy.Service('CoffeeSupply', DetectCoffeeSupply, self.coffee_command)
#         self.positions = {}
#         self.camera = None

#     # 相對位置
#     def get_relative_position(self, from_name, to_name):
#         from_pos = self.positions.get(from_name.lower())
#         to_pos = self.positions.get(to_name.lower())
#         if from_pos is None or to_pos is None:
#             return None
#         return tuple(round(to_pos[i] - from_pos[i], 3) for i in range(3))

#     def get_all_relative(self):
#         tree_to_target = self.get_relative_position('tree', 'black') or self.get_relative_position('tree', 'white')
#         home_to_target = self.get_relative_position('home', 'black') or self.get_relative_position('home', 'white')
#         return {'tree_to_target': tree_to_target, 'home_to_target': home_to_target}

#     def coffee_command(self, req):
#         self.camera = RealSenseCamera()
#         if not self.camera.started:
#             rospy.logwarn("Camera not initialized or failed to start.")
#             return DetectCoffeeSupplyResponse(success=False, target_name="", table=0, cup_side="")

#         self.positions = {}
#         detections_all = []
#         try:
#             # --- 多幀咖啡牌偵測 ---
#             for _ in range(5):
#                 depth_intrin, img_color, depth_frame = self.camera.get_aligned_images()
#                 if depth_intrin is None:
#                     rospy.sleep(0.02)
#                     continue
#                 detections_all.extend(self.detector.detect(img_color, depth_frame, depth_intrin))
#                 rospy.sleep(0.05)

#             if not detections_all:
#                 rospy.loginfo("No detections in captured frames.")
#                 return DetectCoffeeSupplyResponse(success=False, target_name="", table=0, cup_side="")

#             # --- 分群 ---
#             object_groups = {}
#             distance_threshold = 0.1
#             for det in detections_all:
#                 name = det['class_name'].lower()
#                 xyz = np.array(det['xyz'])
#                 group_list = object_groups.setdefault(name, [])
#                 for group in group_list:
#                     if np.linalg.norm(np.median(group, axis=0) - xyz) < distance_threshold:
#                         group.append(xyz)
#                         break
#                 else:
#                     group_list.append([xyz])

#             # --- 選出 target ---
#             target_name, target_xyz = None, None
#             for candidate in ['black', 'white']:
#                 if candidate in object_groups:
#                     groups = object_groups[candidate]
#                     largest_group = max(groups, key=len)
#                     target_name = candidate
#                     target_xyz = np.median(np.array(largest_group), axis=0)
#                     break

#             if not target_name:
#                 rospy.loginfo("No coffee (black/white) group found.")
#                 return DetectCoffeeSupplyResponse(success=False, target_name="", table=0, cup_side="")

#             self.positions[target_name] = tuple(target_xyz)

#             # --- landmark ---
#             for lm in ['tree', 'home']:
#                 if lm in object_groups:
#                     groups = object_groups[lm]
#                     self.positions[lm] = tuple(np.median(np.array(max(groups, key=len)), axis=0))

#             # --- 桌號判斷 ---
#             rels = self.get_all_relative()
#             def dist(vec): return round(math.sqrt(sum(x**2 for x in vec)),3) if vec else None
#             tol = 0.05
#             def near(a,b): return a is not None and b is not None and abs(a-b)<tol

#             dist_tree = dist(rels['tree_to_target'])
#             dist_home = dist(rels['home_to_target'])
#             table_lookup = {
#                 (0.16,0.07):1, (0.075,0.155):2, (0.205,0.15):3, (0.155,0.2):4
#             }
#             table = 0
#             for (dt, dh), t in table_lookup.items():
#                 if near(dist_tree, dt) and near(dist_home, dh):
#                     table = t
#                     break
#             success = table != 0

#             # --- 咖啡杯左右判斷 ---
#             cup_positions = {}
#             for _ in range(5):
#                 depth_intrin, img_color, depth_frame = self.camera.get_aligned_images()
#                 if depth_intrin is None:
#                     rospy.sleep(0.02)
#                     continue
#                 cup_dets = self.cup_color_detector.detect(img_color, depth_frame, depth_intrin)
#                 for det in cup_dets:
#                     name = det['class_name'].lower()
#                     rel_x = det['xyz'][0] - self.positions[target_name][0]
#                     cup_positions[name] = 'left' if rel_x < 0 else 'right'
#                 rospy.sleep(0.03)

#             white_side = cup_positions.get('white', 'unknown')
#             black_side = cup_positions.get('black', 'unknown')
#             rospy.loginfo(f"White coffee is on the {white_side} side and black coffee is on the {black_side} side")

#             cup_side = cup_positions.get(target_name, "")

#             return DetectCoffeeSupplyResponse(
#                 success=success,
#                 target_name=target_name,
#                 table=table,
#                 cup_side=cup_side
#             )
    
#         except Exception as e: 
#             rospy.logerr(f"[App] coffee_command exception: {e}") 
#             return DetectCoffeeSupplyResponse(success=False, target_name="", table=0, cup_side="")
#         finally: 
#             if self.camera: 
#                 self.camera.stop() 
#     def shutdown(self):
#         if self.camera:
#             self.camera.stop()
#         rospy.loginfo("Coffee Supply Server Shutdown.")

# # =============================================================================
# # Main
# # =============================================================================
# if __name__ == '__main__':
#     try:
#         rospy.init_node('coffee_supply_server_node')
#         rospack = rospkg.RosPack()
#         pkg_path = rospack.get_path('object_detect')
#         model_path = os.path.join(pkg_path, 'scripts', 'coffee_supply.pt')
#         cup_model_path = os.path.join(pkg_path, 'scripts', 'coffee.pt')
#         server = App(model_path, cup_model_path)
#         rospy.on_shutdown(server.shutdown)
#         rospy.spin()
#     except rospy.ROSInterruptException:
#         pass
#     except Exception as e:
#         rospy.logerr(f"Failed to start Coffee Supply Server: {e}")

#!/usr/bin/env python3
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
import rospy, math, os, rospkg
from object_detect.srv import DetectCoffeeSupply, DetectCoffeeSupplyResponse

def safe_median(values):
    if not values:
        return None
    m = float(np.median(values))
    return m if math.isfinite(m) else None

def dist(vec):
    return round(math.sqrt(sum(x**2 for x in vec)), 3) if vec else None

# =============================================================================
# RealSense Camera
# =============================================================================
class RealSenseCamera:
    def __init__(self, width=848, height=480, fps=30):
        self.pipeline = rs.pipeline()
        self.align = None
        self.started = False
        try:
            config = rs.config()
            config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
            config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
            self.pipeline.start(config)
            self.align = rs.align(rs.stream.color)
            self.started = True
            rospy.loginfo("RealSense Camera Initialized at startup.")
        except Exception as e:
            rospy.logerr(f"[Camera] Failed to start pipeline: {e}")

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
        if self.started:
            try:
                self.pipeline.stop()
                rospy.loginfo("RealSense Camera Stopped.")
            except Exception as e:
                rospy.logwarn(f"[Camera] Stop warning: {e}")
            self.started = False

# =============================================================================
# YOLO Detector
# =============================================================================
class ObjectDetector:
    def __init__(self, model_path, conf_thresh=0.4, sample_size=5):
        self.model = YOLO(model_path)
        self.conf_thresh = conf_thresh
        self.sample_size = sample_size
        rospy.loginfo(f"Object Detector Loaded: {model_path}")

    def get_median_depth(self, depth_frame, u, v):
        half = self.sample_size // 2
        values = []
        h, w = depth_frame.get_height(), depth_frame.get_width()
        for du in range(-half, half + 1):
            for dv in range(-half, half + 1):
                x, y = int(u) + du, int(v) + dv
                if 0 <= x < w and 0 <= y < h:
                    d = depth_frame.get_distance(x, y)
                    if d > 0:
                        values.append(d)
        return safe_median(values)

    def detect(self, img_color, depth_frame, depth_intrin):
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
                depth = self.get_median_depth(depth_frame, ux, uy)
                if depth is None:
                    continue
                xyz_camera = rs.rs2_deproject_pixel_to_point(depth_intrin, (int(ux), int(uy)), float(depth))
                world_xyz = (round(xyz_camera[0],3), round(xyz_camera[2],3), round(-xyz_camera[1],3))
                class_name = names[int(classes[i])] if int(classes[i]) in names else str(classes[i])
                detections.append({'class_name': class_name.lower(), 'conf': round(float(confs[i]),3), 'xyz': world_xyz})
        return detections

# =============================================================================
# Coffee Supply Server
# =============================================================================
class CoffeeSupplyApp:
    def __init__(self, model_path, cup_model_path):
        rospy.loginfo("Starting Coffee Supply Server...")
        self.camera = RealSenseCamera()  # ✅ 在初始化階段啟動
        self.detector = ObjectDetector(model_path)
        self.cup_detector = ObjectDetector(cup_model_path)
        self.positions = {}
        self.distance_threshold = rospy.get_param('~distance_threshold', 0.1)
        self.tol = rospy.get_param('~tolerance', 0.05)
        self.table_lookup = {
            (0.16,0.07):1, (0.075,0.155):2, (0.205,0.15):3, (0.155,0.2):4
        }
        self.service = rospy.Service('CoffeeSupply', DetectCoffeeSupply, self.coffee_command)

    def capture_detections(self, detector, frames=25, delay=0.01):
        detections = []
        for _ in range(frames):
            depth_intrin, img_color, depth_frame = self.camera.get_aligned_images()
            if depth_intrin is None:
                rospy.sleep(delay/2)
                continue
            detections.extend(detector.detect(img_color, depth_frame, depth_intrin))
            rospy.sleep(delay)
        return detections

    def group_objects(self, detections):
        groups = {}
        for det in detections:
            name = det['class_name']
            xyz = np.array(det['xyz'])
            cluster_list = groups.setdefault(name, [])
            for cluster in cluster_list:
                if np.linalg.norm(np.median(cluster, axis=0) - xyz) < self.distance_threshold:
                    cluster.append(xyz)
                    break
            else:
                cluster_list.append([xyz])
        return groups

    def get_relative_position(self, a, b):
        if a not in self.positions or b not in self.positions:
            return None
        return tuple(round(self.positions[b][i] - self.positions[a][i],3) for i in range(3))

    def coffee_command(self, req):
        if not self.camera.started:
            rospy.logwarn("Camera not running.")
            return DetectCoffeeSupplyResponse(False,"",0,"")
        try:
            detections = self.capture_detections(self.detector)
            if not detections:
                return DetectCoffeeSupplyResponse(False,"",0,"")
            groups = self.group_objects(detections)

            target_name, target_xyz = None, None
            for cand in ['black','white']:
                if cand in groups:
                    target_name = cand
                    target_xyz = np.median(np.array(max(groups[cand], key=len)), axis=0)
                    break
            if not target_name:
                return DetectCoffeeSupplyResponse(False,"",0,"")

            self.positions[target_name] = tuple(target_xyz)
            for lm in ['tree','home']:
                if lm in groups:
                    self.positions[lm] = tuple(np.median(np.array(max(groups[lm], key=len)),axis=0))

            dist_tree = dist(self.get_relative_position('tree', target_name))
            dist_home = dist(self.get_relative_position('home', target_name))
            table = 0
            for (dt, dh), t in self.table_lookup.items():
                if dist_tree and dist_home and abs(dist_tree-dt)<self.tol and abs(dist_home-dh)<self.tol:
                    table = t
                    break

            cup_positions = {}
            for det in self.capture_detections(self.cup_detector, frames=15, delay=0.01):
                rel_x = det['xyz'][0] - self.positions[target_name][0]
                cup_positions[det['class_name']] = 'left' if rel_x < 0 else 'right'
            cup_side = cup_positions.get(target_name, "")
            return DetectCoffeeSupplyResponse(success=(table!=0), target_name=target_name, table=table, cup_side=cup_side)

        except Exception as e:
            rospy.logerr(f"[App] Exception: {e}")
            return DetectCoffeeSupplyResponse(False,"",0,"")

    def shutdown(self):
        if self.camera and self.camera.started:
            self.camera.stop()
        rospy.loginfo("Coffee Supply Server Shutdown.")

if __name__ == '__main__':
    try:
        rospy.init_node('coffee_supply_server_node')
        pkg_path = rospkg.RosPack().get_path('object_detect')
        model_path = os.path.join(pkg_path, 'scripts', 'coffee_supply.pt')
        cup_model_path = os.path.join(pkg_path, 'scripts', 'coffee.pt')
        server = CoffeeSupplyApp(model_path, cup_model_path)
        rospy.on_shutdown(server.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Failed to start Coffee Supply Server: {e}")