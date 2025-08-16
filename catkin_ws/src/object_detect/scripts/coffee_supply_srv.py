import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
import rospy
import math
import rospkg, os
from object_detect.srv import DetectCoffeeSupply, DetectCoffeeSupplyResponse


# =============================================================================
# Class: RealSenseCamera
# 目的: 封裝所有與 Intel RealSense 攝影機相關的操作。
#      包含初始化、取得對齊的影像幀、以及關閉攝影機。
# =============================================================================

class RealSenseCamera:
    def __init__(self):
        """
        初始化函式：
        - 建立節點。
        - 建立 RealSense pipeline 和 config 物件。
        - 設定要啟用的影像流 (深度和彩色)。
        - 啟動 pipeline。
        - 建立 align 物件，用於將深度圖對齊到彩色圖的視角。
        """
        self.pipeline = rs.pipeline()
        config = rs.config()
        # 啟用 640x480 的深度和彩色影像流，幀率為 30fps
        config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)
        self.pipeline.start(config)
        # 建立對齊物件，將深度圖對齊到彩色圖的座標系
        self.align = rs.align(rs.stream.color)
        rospy.loginfo("RealSense Camera Initialized.")

    def get_aligned_images(self):
        """
        取得對齊後的影像幀。
        - 等待並取得一組新的 frames。
        - 執行對齊處理。
        - 分別取出深度幀和彩色幀。
        - 檢查幀是否有效。
        - 取得深度攝影機的內部參數 (intrinsics)，這對於 3D 座標轉換至關重要。
        - 將彩色幀轉換為 OpenCV 能處理的 NumPy array 格式。
        """
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not depth_frame or not color_frame:
            return None, None, None

        # 取得深度攝影機的內部參數 (焦距 fx, fy, 主點 ppx, ppy 等)
        depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
        img_color = np.asanyarray(color_frame.get_data())
        return depth_intrin, img_color, depth_frame

    def stop(self):
        """
        停止 pipeline，釋放攝影機資源。
        """
        self.pipeline.stop()
        rospy.loginfo("RealSense Camera Stopped.")

# =============================================================================
# Class: ObjectDetector
# 目的: 封裝所有與 YOLOv8 物件偵測和 3D 座標計算相關的邏輯。
# =============================================================================

class ObjectDetector:
    def __init__(self, model_path, conf_thresh=0.65):
        """
        初始化函式：
        - 載入指定的 YOLOv8 模型。
        - 設定信心度閾值。
        """
        self.model = YOLO(model_path)
        self.conf_thresh = conf_thresh
        rospy.loginfo(f"Object Detector Initialized with model: {model_path}")

    def get_median_depth(self, depth_frame, u, v, sample_size=5):
        """
        【輔助函式1】取得穩定的深度值。
        目的：單一像素的深度值可能不準確或有噪點，因此取物體中心點周圍一個小區域
             (sample_size x sample_size) 的深度值，並計算其中位數，以獲得更穩定的結果。
        """
        half_size = sample_size // 2
        depth_values = []
        # 取得影像的寬高，確保取樣範圍不會超出邊界
        height, width = depth_frame.get_height(), depth_frame.get_width()
        for du in range(-half_size, half_size + 1):
            for dv in range(-half_size, half_size + 1):
                x, y = int(u) + du, int(v) + dv
                if 0 <= x < width and 0 <= y < height:
                    depth = depth_frame.get_distance(x, y)
                    if depth > 0:  # 過濾掉無效的深度值 (0)
                        depth_values.append(depth)
        # 如果 list 中有值，則回傳中位數；否則回傳 None
        return np.median(depth_values) if depth_values else None

    def estimate_object_radius(self, box_xyxy, surface_depth, depth_intrin):
        """
        【輔助函式2】根據2D邊界框估算物體半徑 (單位：公尺)。
        原理：利用相機的針孔模型，物體的實際大小與它在影像上的像素大小、深度、相機焦距有關。
              公式: Real Size = (Pixel Size * Depth) / Focal Length
        """
        # 計算邊界框在影像上的寬度和高度 (單位：像素)
        box_width_px = box_xyxy[2] - box_xyxy[0]
        box_height_px = box_xyxy[3] - box_xyxy[1]
        
        # 假設物體為球體，使用平均直徑來估算像素半徑
        diameter_px = (box_width_px + box_height_px) / 2
        radius_px = diameter_px / 2
        
        # 從內部參數中取得相機的水平焦距 (fx)
        fx = depth_intrin.fx
        
        # 應用公式，將像素半徑轉換為公尺單位的實際半徑
        radius_meters = (radius_px * surface_depth) / fx
        
        return radius_meters

    def detect(self, img_color, aligned_depth_frame, depth_intrin):
        """
        執行物件偵測並計算每個物體中心點的三維座標。
        """
        # 執行 YOLOv8 推理
        results = self.model.predict([img_color], conf=self.conf_thresh, verbose=False)
        detections = []
        im_out = img_color.copy() # 複製一份原始影像，用於繪製結果

        for result in results:
            if result.boxes is None:
                continue # 如果沒有偵測到任何物體，則跳過

            # 提取偵測結果的資訊
            boxes_xyxy = result.boxes.xyxy.cpu().numpy()
            confs = result.boxes.conf.cpu().numpy()
            classes = result.boxes.cls.cpu().numpy().astype(int)
            names = result.names

            for i in range(len(boxes_xyxy)):
                box = boxes_xyxy[i]
                # 計算 bounding box 的中心點 (u, v)
                ux = (box[0] + box[2]) / 2
                uy = (box[1] + box[3]) / 2
                
                # --- 核心計算邏輯 ---
                surface_dis = self.get_median_depth(aligned_depth_frame, ux, uy)
                if surface_dis is None:
                    continue # 若無法取得有效深度，跳過此物體             
                center_dis = surface_dis
                center_camera_xyz = rs.rs2_deproject_pixel_to_point(depth_intrin, (ux, uy), center_dis)
                center_camera_xyz = [round(v, 2) for v in center_camera_xyz] # 四捨五入到小數點後三位
                world_x = center_camera_xyz[0]
                world_y = center_camera_xyz[2]  # 將攝影機的 Z 軸 (深度) 對應到世界座標的 Y 軸 (向前)
                world_z = -center_camera_xyz[1] # 將攝影機的 Y 軸 (向下) 對應到世界座標的 Z 軸 (向上)
                world_xyz = (world_x, world_y, world_z)

                class_id = int(classes[i])
                class_name = names.get(class_id, str(class_id))
                conf = float(confs[i])
                
                # 將所有偵測資訊打包成一個字典，並存入 list
                detections.append({
                    'class_id': class_id,
                    'class_name': class_name,
                    'conf': round(conf, 3),
                    'xyz': tuple(world_xyz) # 儲存的是物體中心的XYZ座標
                })

                # --- 繪圖區 ---
                x1, y1, x2, y2 = box
                # 繪製 bounding box (綠色框)
                cv2.rectangle(im_out, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                # 繪製中心點 (紅色實心圓)
                cv2.circle(im_out, (int(ux), int(uy)), 4, (0, 0, 255), -1)
                
                # 準備要顯示的標籤文字
                label = f"{class_name} ({conf:.2f})"
                label_xyz = f"Center XYZ(m): {world_xyz}" # 顯示的是中心點的XYZ
                
                # 繪製標籤文字的背景，讓文字更清晰
                (w, h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
                cv2.rectangle(im_out, (int(x1), int(y1) - 20), (int(x1) + w, int(y1)), (0, 255, 0), -1)
                # 繪製類別和信心度文字
                cv2.putText(im_out, label, (int(x1), int(y1) - 5), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
                # 繪製三維座標文字
                cv2.putText(im_out, label_xyz, (int(x1), int(y2) + 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
                
        return im_out, detections
    
# =============================================================================
# Class: App
# 目的：偵測圖卡指令
# =============================================================================

class App:
    def __init__(self, model_path):
        rospy.loginfo("Initializing Coffee Supply Server ......")
        self.detector = ObjectDetector(model_path)
        rospy.loginfo("Coffee Supply initialized. Starting detection loop...")
        self.coffee_supply_service = rospy.Service('CoffeeSupply', DetectCoffeeSupply, self.coffee_command)
        rospy.loginfo("Service 'CoffeeSupply' is ready.")
        self.positions = {}



    def get_relative_position(self, from_name, to_name):
        from_pos = self.positions.get(from_name.lower())
        to_pos = self.positions.get(to_name.lower())
        if from_pos is None or to_pos is None:
            return None
        rel = tuple(round(to_pos[i] - from_pos[i], 3) for i in range(3))
        return rel

    def get_all_relative(self):
        target = None
        if 'black' in self.positions:
            target = 'black'
        elif 'white' in self.positions:
            target = 'white'
        else:
            return None
        result = {
            'home_to_target': self.get_relative_position('home', target),
            'tree_to_target': self.get_relative_position('tree', target),
            'target_name': target
        }
        return result

    def coffee_command(self, req):
        camera = None
        try:
            camera = RealSenseCamera()

            depth_intrin, img_color, depth_frame = camera.get_aligned_images()
            if depth_intrin is None or img_color is None or depth_frame is None:
                rospy.logwarn("Camera frame is invalid.")
                return DetectCoffeeSupplyResponse(success=False, target_name="", table="")

            _, detections = self.detector.detect(img_color, depth_frame, depth_intrin)

            for det in detections:
                name = det['class_name'].lower()
                self.positions[name] = det['xyz']

            rels = self.get_all_relative()
            if not rels:
                rospy.loginfo("Can not find the supply card")
                return DetectCoffeeSupplyResponse(success=False, target_name="", table="")

            def calc_distance(vec):
                if vec is None:
                    return None
                return round(math.sqrt(sum(x**2 for x in vec)), 3)

            dist_tree = calc_distance(rels['tree_to_target'])
            dist_home = calc_distance(rels['home_to_target'])

            if (dist_tree is not None and dist_home is not None and
                abs(dist_tree - 0.16) < 0.01 and abs(dist_home - 0.07) < 0.01):
                return DetectCoffeeSupplyResponse(True, rels['target_name'], "1")
            elif (dist_tree is not None and dist_home is not None and
                abs(dist_tree - 0.075) < 0.01 and abs(dist_home - 0.155) < 0.01):
                return DetectCoffeeSupplyResponse(True, rels['target_name'], "2")
            elif (dist_tree is not None and dist_home is not None and
                abs(dist_tree - 0.205) < 0.01 and abs(dist_home - 0.15) < 0.01):
                return DetectCoffeeSupplyResponse(True, rels['target_name'], "3")
            elif (dist_tree is not None and dist_home is not None and
                abs(dist_tree - 0.155) < 0.01 and abs(dist_home - 0.2) < 0.01):
                return DetectCoffeeSupplyResponse(True, rels['target_name'], "4")

            return DetectCoffeeSupplyResponse(False, "", "")

        except Exception as e:
            rospy.logerr(f"An error occurred during detection: {e}")
            return DetectCoffeeSupplyResponse(False, "", "")

        finally:
            if camera:
                camera.stop()
            cv2.destroyAllWindows()

    def shutdown(self):
        cv2.destroyAllWindows()
        rospy.loginfo("Coffee Supply Server Shutdown.")

if __name__ == '__main__':
    try:
        rospy.init_node('coffee_supply_srv')
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