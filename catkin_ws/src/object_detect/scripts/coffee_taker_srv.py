#!/usr/bin/env python3
import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
import rospy
import os
import rospkg
from geometry_msgs.msg import Twist
from object_detect.srv import DetectCoffee, DetectCoffeeResponse

# =============================================================================
# Class: RealSenseCamera
# 目的: 封裝所有與 Intel RealSense 攝影機相關的操作。
# =============================================================================
class RealSenseCamera:
    def __init__(self):
        self.pipeline = rs.pipeline()
        self.started = False
        config = rs.config()
        config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)
        try:
            self.pipeline.start(config)
            self.started = True
            self.align = rs.align(rs.stream.color)
            rospy.loginfo("RealSense Camera Initialized.")
        except Exception as e:
            rospy.logerr(f"Failed to start RealSense Camera: {e}")
            self.started = False

    def stop(self):
        if self.started:
            self.pipeline.stop()
            self.started = False
            rospy.loginfo("RealSense Camera Stopped.")
        else:
            rospy.logwarn("RealSense pipeline was not started; skip stopping.")

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

# =============================================================================
# Class: ObjectDetector
# 目的: 封裝 YOLOv8 物件偵測和 3D 座標計算相關邏輯
# =============================================================================
class ObjectDetector:
    def __init__(self, model_path, conf_thresh=0.8):
        self.model = YOLO(model_path)
        self.conf_thresh = conf_thresh

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

    def detect(self, img_color, aligned_depth_frame, depth_intrin, target_class_name):
        results = self.model.predict([img_color], conf=self.conf_thresh, verbose=False)
        detections = []
        img_with_detections = img_color.copy()

        for result in results:
            if result.boxes is None or result.boxes.xyxy is None:
                continue
            boxes_xyxy = result.boxes.xyxy.cpu().numpy()
            classes = result.boxes.cls.cpu().numpy().astype(int)
            names = result.names

            for i in range(len(boxes_xyxy)):
                class_id = int(classes[i])
                class_name = names.get(class_id, str(class_id))
                if class_name != target_class_name:
                    continue
                box = boxes_xyxy[i]
                ux = (box[0] + box[2]) / 2
                uy = (box[1] + box[3]) / 2
                object_dis = self.get_median_depth(aligned_depth_frame, ux, uy)
                if object_dis is None:
                    continue
                center_camera_xyz = rs.rs2_deproject_pixel_to_point(depth_intrin, (ux, uy), object_dis)
                world_x = center_camera_xyz[0]
                world_y = center_camera_xyz[2]
                world_z = -center_camera_xyz[1]
                detections.append({
                    'class_name': class_name,
                    'xyz': (world_x, world_y, world_z)
                })

                # Draw detection box and text
                cv2.rectangle(img_with_detections, (int(box[0]), int(box[1])), 
                              (int(box[2]), int(box[3])), (0, 255, 0), 2)
                cv2.putText(img_with_detections, f"{class_name}: {object_dis:.2f}m", 
                            (int(box[0]), int(box[1])-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        return detections, img_with_detections

# =============================================================================
# Class: App
# 目的：偵測咖啡並回傳 Service
# =============================================================================
class App:
    def __init__(self, model_path):
        rospy.loginfo("Initializing Coffee Taker Server...")
        self.detector = ObjectDetector(model_path)  # 保留 YOLO 模型在初始化
        self.velocity_publisher = rospy.Publisher('dlv/cmd_vel', Twist, queue_size=10)
        self.previous_twist = Twist()
        self.coffee_service = rospy.Service('CoffeeTaker', DetectCoffee, self.handle_get_coffee_depth)
        rospy.loginfo("Service 'CoffeeTaker' is ready.")

    def is_twist_different(self, t1, t2):
        return any([
            t1.linear.x != t2.linear.x,
            t1.linear.y != t2.linear.y,
            t1.linear.z != t2.linear.z,
            t1.angular.x != t2.angular.x,
            t1.angular.y != t2.angular.y,
            t1.angular.z != t2.angular.z
        ])

    def handle_get_coffee_depth(self, req):
        try:
            camera = RealSenseCamera()
            if not camera.started:
                rospy.logerr("Camera failed to start.")
                return DetectCoffeeResponse(success=False, depth=0.0, step_motor="0")

            coffee_type = req.coffee_type
            rate = rospy.Rate(10)
            while not rospy.is_shutdown():
                depth_intrin, img_color, depth_frame = camera.get_aligned_images()
                if depth_intrin is None:
                    rospy.loginfo("No camera data.")
                    rate.sleep()
                    continue

                detections, img_with_detections = self.detector.detect(img_color, depth_frame, depth_intrin, coffee_type)
                cv2.imshow("Detection", img_with_detections)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    rospy.loginfo("Quit signal received from cv2 window.")
                    cv2.destroyAllWindows()
                    camera.stop()
                    return DetectCoffeeResponse(success=False, depth=0.0, step_motor="0")

                if detections:
                    closest_coffee = min(detections, key=lambda d: d['xyz'][1])
                    depth_y = closest_coffee['xyz'][1]
                    world_x = closest_coffee['xyz'][0]

                    twist_msg = Twist()
                    twist_msg.linear.y = 0.0
                    rospy.loginfo(f"Coffee X: {world_x:.3f}")
                    offset_left = -0.13
                    offset_right = 0.17
                    if world_x > offset_left + offset_right / 2:
                        step = 2
                        offset = offset_right
                    else:
                        step = 1
                        offset = offset_left
                    if abs(world_x - offset) < 0.015:
                        rospy.loginfo(f"Coffee is centered. Depth: {depth_y:.2f}m")
                        self.velocity_publisher.publish(twist_msg)
                        cv2.destroyAllWindows()
                        camera.stop()  # ⬅️ 偵測完成後關閉相機
                        return DetectCoffeeResponse(success=True, depth=float(depth_y), step_motor=step)
                    elif (world_x - offset) > 0:
                        rospy.loginfo("Coffee is to the right.")
                        twist_msg.linear.y = -0.1
                    elif (world_x - offset) < 0:
                        rospy.loginfo("Coffee is to the left.")
                        twist_msg.linear.y = 0.1

                    if self.is_twist_different(twist_msg, self.previous_twist):
                        self.velocity_publisher.publish(twist_msg)
                        self.previous_twist = twist_msg
                else:
                    rospy.loginfo("No Coffee detected.")

                rate.sleep()

            return DetectCoffeeResponse(success=False, depth=0.0, step_motor = "0")

        except Exception as e:
            rospy.logerr(f"An error occurred during detection: {e}")
            return DetectCoffeeResponse(success=False, depth=0.0, step_motor = "0")

    def shutdown(self):
        self.camera.stop()
        cv2.destroyAllWindows()
        rospy.loginfo("Coffee Taker Server Shutdown.")

if __name__ == '__main__':
    try:
        rospy.init_node('coffee_taker_server_node')
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('object_detect')
        model_path = os.path.join(package_path, 'scripts', 'coffee.pt')
        server = App(model_path)
        rospy.on_shutdown(server.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Failed to start Coffee Taker Server: {e}")
