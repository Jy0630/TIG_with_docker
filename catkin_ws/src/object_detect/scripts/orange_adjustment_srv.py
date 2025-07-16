#!/usr/bin/env python3
import time
import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
import rospy
import os
import rospkg
from object_detect.srv import DetectOrangeAdjustment, DetectOrangeAdjustmentResponse

class RealSenseCamera:
    def __init__(self):
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)
        rospy.loginfo("RealSense Camera Initialized.")

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
        rospy.loginfo("RealSense Camera Stopped.")


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
            if result.boxes is None:
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
                surface_dis = self.get_median_depth(aligned_depth_frame, ux, uy)
                if surface_dis is None:
                    continue
                object_radius = self.estimate_object_radius(box, surface_dis, depth_intrin)
                center_dis = surface_dis + object_radius
                center_camera_xyz = rs.rs2_deproject_pixel_to_point(depth_intrin, (ux, uy), center_dis)
                world_x = center_camera_xyz[0]
                world_y = center_camera_xyz[2]  # depth
                world_z = -center_camera_xyz[1]
                detections.append({
                    'class_name': class_name,
                    'xyz': (world_x, world_y, world_z)
                })

                # Draw detection box and text
                cv2.rectangle(img_with_detections, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (0, 255, 0), 2)
                cv2.putText(img_with_detections, f"{class_name}: {surface_dis:.2f}m", (int(box[0]), int(box[1])-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        return detections, img_with_detections


class OrangeDetectionServer:
    def __init__(self, model_path):
        rospy.loginfo("Initializing Orange Detection Server...")
        self.camera = RealSenseCamera()
        self.detector = ObjectDetector(model_path)
        self.service = rospy.Service('get_orange_depth', DetectOrangeAdjustment, self.handle_get_orange_depth)
        rospy.loginfo("Service 'get_orange_depth' is ready.")

    def handle_get_orange_depth(self, req):
        rospy.loginfo("Received request to find an orange.")
        try:
            all_detections = []
            for _ in range(5):
                depth_intrin, img_color, depth_frame = self.camera.get_aligned_images()
                if depth_intrin is None:
                    rospy.sleep(0.1)
                    continue
                detections, img_with_detections = self.detector.detect(img_color, depth_frame, depth_intrin)

                # Show image with detections
                cv2.imshow("Orange Detection", img_with_detections)
                cv2.waitKey(1)

                if detections:
                    all_detections.extend(detections)
                rospy.sleep(0.1)

            if not all_detections:
                rospy.logwarn("No oranges detected after 5 attempts.")
                cv2.destroyAllWindows()
                return DetectOrangeAdjustmentResponse(success=False, depth=0.0, x_value=0.0)
            
            for d in all_detections:
                rospy.loginfo(f"Detected orange at X: {d['xyz'][0]:.3f}, Y(depth): {d['xyz'][1]:.3f}")

            closest_orange = min(all_detections, key=lambda d: d['xyz'][1])
            depth_y = closest_orange['xyz'][1]
            world_x = closest_orange['xyz'][0]
            rospy.loginfo(f"Closest orange found. Depth (Y-axis): {depth_y:.3f} meters, X: {world_x:.3f} meters.")

            cv2.destroyAllWindows()
            return DetectOrangeAdjustmentResponse(success=True, depth=float(depth_y), x_value=float(world_x))

        except Exception as e:
            rospy.logerr(f"An error occurred during detection: {e}")
            cv2.destroyAllWindows()
            return DetectOrangeAdjustmentResponse(success=False, depth=0.0, x_value=0.0)

    def shutdown(self):
        self.camera.stop()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        rospy.init_node('orange_depth_server_node')
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('object_detect')
        model_path = os.path.join(package_path, 'scripts', 'orange.pt')
        server = OrangeDetectionServer(model_path)
        rospy.on_shutdown(server.shutdown)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Failed to start Orange Detection Server: {e}")
