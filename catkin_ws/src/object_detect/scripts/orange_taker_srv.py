#!/usr/bin/env python3
import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
import rospy
import os
import rospkg
from geometry_msgs.msg import Twist
from object_detect.srv import DetectOrange, DetectOrangeResponse

class RealSenseCamera:
    def __init__(self):
        self.pipeline = rs.pipeline()
        self.started = False  # 新增 flag
        config = rs.config()
        config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)
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
            self.started = False  # 停止後修改 flag
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

class ObjectDetector:
    def __init__(self, model_path, conf_thresh=0.7):
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

    def detect(self, img_color, aligned_depth_frame, depth_intrin):
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
                if class_name != 'orange':
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
                cv2.rectangle(img_with_detections, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (0, 255, 0), 2)
                cv2.putText(img_with_detections, f"{class_name}: {object_dis:.2f}m", (int(box[0]), int(box[1])-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        return detections, img_with_detections

class App:
    def __init__(self, model_path):
        rospy.loginfo("Initializing Orange Taker Server (no service mode)...")
        self.camera = RealSenseCamera()
        self.detector = ObjectDetector(model_path)
        self.velocity_publisher = rospy.Publisher('dlv/cmd_vel', Twist, queue_size=10)
        self.previous_twist = Twist()  
        rospy.loginfo("Orange Taker initialized. Starting detection loop...")
        self.orange_service = rospy.Service('OrangeTaker', DetectOrange, self.handle_get_orange_depth)
        rospy.loginfo("Service 'Orange Taker' is ready.")

    def is_twist_different(self, t1, t2):
        return any([
            t1.linear.x != t2.linear.x,
            t1.linear.y != t2.linear.y,
            t1.linear.z != t2.linear.z,
            t1.angular.x != t2.angular.x,
            t1.angular.y != t2.angular.y,
            t1.angular.z != t2.angular.z
        ])

    def handle_get_orange_depth(self, req):
        try:
            rate = rospy.Rate(10) 
            while not rospy.is_shutdown():
                depth_intrin, img_color, depth_frame = self.camera.get_aligned_images()
                if depth_intrin is None:
                    rospy.loginfo("No camera data.")
                    rate.sleep()
                    continue

                detections, img_with_detections = self.detector.detect(img_color, depth_frame, depth_intrin)
                cv2.imshow("Detection", img_with_detections)


                if detections:
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        rospy.loginfo("Quit signal received from cv2 window.")
                        self.shutdown()
                        break

                    closest_orange = min(detections, key=lambda d: d['xyz'][1])
                    depth_y = closest_orange['xyz'][1]
                    world_x = closest_orange['xyz'][0]

                    twist_msg = Twist()
                    twist_msg.linear.x = 0.0
                    twist_msg.linear.y = 0.0
                    twist_msg.linear.z = 0.0
                    twist_msg.angular.x = 0.0
                    twist_msg.angular.y = 0.0
                    twist_msg.angular.z = 0.0

                    if abs(world_x) < 0.1:
                        rospy.loginfo(f"Orange is centered. Depth: {depth_y:.2f}m")
                        self.velocity_publisher.publish(twist_msg)  
                        self.shutdown()
                        return DetectOrangeResponse(success=True, depth=float(depth_y))

                    elif world_x > 0:
                        rospy.loginfo("Orange is to the right.")
                        twist_msg.linear.x = 0.1
                        twist_msg.angular.z = -0.2

                    elif world_x < 0:
                        rospy.loginfo("Orange is to the left.")
                        twist_msg.linear.x = 0.1
                        twist_msg.angular.z = 0.2

                    if self.is_twist_different(twist_msg, self.previous_twist):
                        self.velocity_publisher.publish(twist_msg)
                        self.previous_twist = twist_msg
                    else:
                        rospy.loginfo("Same twist. Not publishing.")

                else:
                    rospy.loginfo("No oranges detected.")

                rate.sleep()
            self.shutdown()
            return DetectOrangeResponse(success=False, depth=0.0)

        except Exception as e:
            rospy.logerr(f"An error occurred during detection: {e}")
            self.shutdown()
            return DetectOrangeResponse(success=False, depth=0.0)

    def shutdown(self):
        self.camera.stop()
        cv2.destroyAllWindows()
        rospy.loginfo("Orange Taker Server Shutdown.")

if __name__ == '__main__':
    try:
        rospy.init_node('orange_taker_server_node')
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('object_detect')
        model_path = os.path.join(package_path, 'scripts', 'orange.pt')
        server = App(model_path)
        rospy.on_shutdown(server.shutdown)
        rospy.spin()  

    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Failed to start Orange Taker Server: {e}")