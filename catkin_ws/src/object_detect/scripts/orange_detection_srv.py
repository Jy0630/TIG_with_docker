#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
import rospy
from std_msgs.msg import Float64MultiArray
import os
import rospkg
from object_detect.srv import DetectOrangeGoal, DetectOrangeGoalResponse

# =============================================================================
# Class: RealSenseCamera (與攝影機硬體溝通)
# =============================================================================
class RealSenseCamera:
    def __init__(self):
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)
        self.profile = self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)
        print("RealSense Camera Initialized.")

    def get_aligned_images(self):
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
        self.pipeline.stop()
        print("RealSense Camera Stopped.")

# =============================================================================
# Class: ObjectDetector (執行YOLO偵測並計算3D座標)
# =============================================================================
class ObjectDetector:
    def __init__(self, model_path, conf_thresh=0.7):
        self.model = YOLO(model_path)
        self.conf_thresh = conf_thresh
        print(f"Object Detector Initialized with model: {model_path}")

    def get_median_depth(self, depth_frame, u, v, sample_size=5):
        half_size = sample_size // 2
        depth_values = []
        height, width = depth_frame.get_height(), depth_frame.get_width()
        for du in range(-half_size, half_size + 1):
            for dv in range(-half_size, half_size + 1):
                x, y = int(u) + du, int(v) + dv
                if 0 <= x < width and 0 <= y < height:
                    depth = depth_frame.get_distance(x, y)
                    if depth > 0:
                        depth_values.append(depth)
        return np.median(depth_values) if depth_values else None

    def estimate_object_radius(self, box_xyxy, surface_depth, depth_intrin):
        box_width_px = box_xyxy[2] - box_xyxy[0]
        box_height_px = box_xyxy[3] - box_xyxy[1]
        diameter_px = (box_width_px + box_height_px) / 2
        radius_px = diameter_px / 2
        fx = depth_intrin.fx
        radius_meters = (radius_px * surface_depth) / fx
        return radius_meters

    def detect(self, img_color, aligned_depth_frame, depth_intrin):
        results = self.model.predict([img_color], conf=self.conf_thresh, verbose=False)
        detections = []
        im_out = img_color.copy()
        for result in results:
            if result.boxes is None: continue
            boxes_xyxy = result.boxes.xyxy.cpu().numpy()
            confs = result.boxes.conf.cpu().numpy()
            classes = result.boxes.cls.cpu().numpy().astype(int)
            names = result.names
            for i in range(len(boxes_xyxy)):
                box = boxes_xyxy[i]
                ux, uy = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2
                surface_dis = self.get_median_depth(aligned_depth_frame, ux, uy)
                if surface_dis is None: continue
                object_radius = self.estimate_object_radius(box, surface_dis, depth_intrin)
                center_dis = surface_dis + object_radius
                center_camera_xyz = rs.rs2_deproject_pixel_to_point(depth_intrin, (ux, uy), center_dis)
                world_x, world_y, world_z = center_camera_xyz[0], center_camera_xyz[2], -center_camera_xyz[1]
                detections.append({
                    'class_name': names.get(int(classes[i]), str(int(classes[i]))),
                    'xyz': (world_x, world_y, world_z)
                })
                x1, y1, x2, y2 = box
                cv2.rectangle(im_out, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                label = f"{names.get(int(classes[i]))} ({float(confs[i]):.2f})"
                cv2.putText(im_out, label, (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        return im_out, detections

# =============================================================================
# Class: ObjectStorage (管理偵測到的物件，並找出目標對)
# =============================================================================
class ObjectStorage:
    def __init__(self, dist_threshold):
        self.stored_objects = {}
        self.object_counter = 1
        self.dist_threshold = dist_threshold

    def is_same_object(self, obj1, obj2):
        if obj1['class'] != obj2['class']: return False
        dist = np.linalg.norm(np.array(obj1['xyz']) - np.array(obj2['xyz']))
        return dist < self.dist_threshold

    def add_or_update_object(self, new_obj):
        for _, obj in self.stored_objects.items():
            if self.is_same_object(obj, new_obj): return
        new_id = f'object{self.object_counter:03d}'
        self.stored_objects[new_id] = {'class': new_obj['class'], 'xyz': [round(v, 3) for v in new_obj['xyz']]}
        self.object_counter += 1

    def process_and_store_clusters(self, coords_list, class_name, min_group_size=10):
        coords = np.array(coords_list)
        if len(coords) < min_group_size: return
        used = set()
        groups = []
        for i in range(len(coords)):
            if i in used: continue
            group = [i]
            for j in range(i + 1, len(coords)):
                if j in used: continue
                if np.linalg.norm(coords[i] - coords[j]) < self.dist_threshold:
                    group.append(j)
            groups.append(group)
            used.update(group)
        for group in groups:
            avg_coord = np.mean(coords[group], axis=0)
            is_new = True
            for obj in self.stored_objects.values():
                if obj['class'] == class_name and np.linalg.norm(np.array(obj['xyz']) - avg_coord) < self.dist_threshold:
                    is_new = False
                    break
            if is_new: self.add_or_update_object({'class': class_name, 'xyz': avg_coord.tolist()})

    def point_to_line_dist(self, p1, p2, p3):
        p1_2d = np.array(p1[0:2])
        p2_2d = np.array(p2[0:2])
        p3_2d = np.array(p3[0:2])
        num = np.abs(np.cross(p2_2d - p1_2d, p1_2d - p3_2d))
        den = np.linalg.norm(p2_2d - p1_2d)
        if den == 0: return float('inf')
        return num / den

    def find_object_pairs(self, min_dist=0.25, max_dist=0.45, 
                          use_trunk_validation=False, trunk_position=None, collinearity_threshold=0.1):
        obj_items = list(self.stored_objects.items())
        for i in range(len(obj_items)):
            id1, obj1 = obj_items[i]
            for j in range(i + 1, len(obj_items)):
                id2, obj2 = obj_items[j]

                if abs(obj1['xyz'][2] - obj2['xyz'][2]) > 0.1: continue
                dist = np.linalg.norm(np.array(obj1['xyz']) - np.array(obj2['xyz']))
                if not (min_dist <= dist <= max_dist): continue

                if use_trunk_validation:
                    if trunk_position is None:
                        rospy.logwarn_throttle(5, "Trunk validation enabled but trunk_position is not set.")
                        continue
                    
                    p1_xyz = obj1['xyz']
                    p2_xyz = obj2['xyz']
                    distance_to_trunk = self.point_to_line_dist(p1_xyz, p2_xyz, trunk_position)
                    
                    if distance_to_trunk > collinearity_threshold:
                        rospy.logdebug(f"Pair {id1}-{id2} failed trunk validation. Distance: {distance_to_trunk:.3f}m")
                        continue
                    rospy.loginfo(f"Pair {id1}-{id2} passed trunk validation! Distance: {distance_to_trunk:.3f}m")

                branch_diff = np.round(np.array(obj2['xyz']) - np.array(obj1['xyz']), 3)
                vector = np.array([-branch_diff[1], branch_diff[0], 0])
                norm = np.linalg.norm(vector)
                if norm == 0: continue
                unit_vector = vector / norm
                
                coord1 = np.array(obj1['xyz'])
                coord2 = np.array(obj2['xyz'])
                base_orange_coord = coord1 if np.linalg.norm(coord1) < np.linalg.norm(coord2) else coord2
                target_coord = base_orange_coord + unit_vector * 0.35
                
                # 1. Calculate the yaw of the target heading vector relative to the World +X' axis.
                yaw_vs_world_x = np.arctan2(unit_vector[1], unit_vector[0])
                
                # 2. Adjust the yaw to be relative to the World +Y' axis, which is our new "forward" (0 rad).
                #    This is equivalent to a -90 degree (-pi/2 rad) rotation of the reference axis.
                target_yaw = yaw_vs_world_x - (np.pi / 2.0)
                
                # 3. Normalize the angle to the range [-pi, pi] for consistency.
                target_yaw = (target_yaw + np.pi) % (2 * np.pi) - np.pi
                
                rospy.loginfo(f"Found a valid pair ({id1}-{id2}). Target coord: {target_coord.tolist()}, Target yaw (vs +Y'): {target_yaw:.2f} rad")
                return True, target_coord.tolist(), target_yaw
                
        return False, None, None

# =============================================================================
# Class: App (主應用程式，負責啟動服務並處理請求)
# =============================================================================
class App:
    def __init__(self, model_path):
        rospy.init_node("orange_detection_server")
        
        self.use_trunk_validation = rospy.get_param("~use_trunk_validation", True)
        trunk_x = rospy.get_param("~trunk_position_x", 0.5) 
        trunk_y = rospy.get_param("~trunk_position_y", 3.0)
        self.trunk_position_world = np.array([trunk_x, trunk_y, 0.0])
        self.collinearity_threshold = rospy.get_param("~collinearity_threshold", 0.1)

        if self.use_trunk_validation:
            rospy.loginfo("Trunk collinearity validation is ENABLED.")
            rospy.loginfo(f"Trunk Position (World-Aligned): {self.trunk_position_world.tolist()}")
            rospy.loginfo(f"Collinearity Threshold: {self.collinearity_threshold}m")
        else:
            rospy.loginfo("Trunk collinearity validation is DISABLED.")
        
        self.camera = RealSenseCamera()
        self.detector = ObjectDetector(model_path)
        self.storage = ObjectStorage(dist_threshold=0.15)
        self.wall_alignment_angle_rad = 0.0
        self.wall_data_sub = rospy.Subscriber('/wall_distances', Float64MultiArray, self.wall_angle_callback, queue_size=1)
        self.detect_service = rospy.Service('detect_orange_goal', DetectOrangeGoal, self.handle_detection_request)
        rospy.loginfo("Orange detection service ('detect_orange_goal') is ready.")
        rospy.spin()

    def wall_angle_callback(self, msg):
        if len(msg.data) > 6 and not np.isnan(msg.data[6]):
            self.wall_alignment_angle_rad = -np.deg2rad(msg.data[6])
        else:
            self.wall_alignment_angle_rad = 0.0

    def handle_detection_request(self, req):
        rospy.loginfo("Received orange detection request. Starting detection cycle...")
        self.storage.stored_objects.clear()
        self.storage.object_counter = 1
        temp_coords = {}
        start_time = time.time()
        while time.time() - start_time < 2.0 and not rospy.is_shutdown():
            depth_intrin, img_color, aligned_depth_frame = self.camera.get_aligned_images()
            if depth_intrin is None: continue
            im_out, detections = self.detector.detect(img_color, aligned_depth_frame, depth_intrin)
            
            angle_rad = self.wall_alignment_angle_rad
            cos_a, sin_a = np.cos(angle_rad), np.sin(angle_rad)
            for det in detections:
                x, y, z = det['xyz']
                x_aligned = x * cos_a - y * sin_a
                y_aligned = x * sin_a + y * cos_a
                
                cname = det['class_name']
                if cname not in temp_coords: temp_coords[cname] = []
                temp_coords[cname].append((x_aligned, y_aligned, z))
            
            cv2.imshow('Detection Service View', im_out)
            cv2.waitKey(1)
        
        rospy.loginfo("Data collection finished. Processing clusters using world-aligned coordinates.")
        for cname, coords in temp_coords.items():
            self.storage.process_and_store_clusters(coords, cname, min_group_size=10)
        
        success, target_coord, target_yaw = self.storage.find_object_pairs(
            use_trunk_validation=self.use_trunk_validation,
            trunk_position=self.trunk_position_world,
            collinearity_threshold=self.collinearity_threshold
        )
        
        response = DetectOrangeGoalResponse()
        if success:
            response.success = True
            response.message = "Successfully found a valid orange goal."
            response.target_x, response.target_y = target_coord[0], target_coord[1]
            response.target_final_yaw = target_yaw
        else:
            response.success = False
            response.message = "Could not find a valid orange pair after all filters."
        return response

if __name__ == '__main__':
    try:
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('object_detect') 
        model_path = os.path.join(package_path, 'scripts', 'orange.pt')
        App(model_path)
    except rospy.ROSInterruptException:
        rospy.loginfo("Orange detection service interrupted.")
    except Exception as e:
        rospy.logerr(f"An error occurred in orange_detection_srv: {e}")
