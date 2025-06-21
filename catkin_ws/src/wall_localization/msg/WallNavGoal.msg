# --- 目標值 (單位: 公尺, 度) ---
# Define the target distances and angle for the robot to navigate towards.
float64 target_front_distance
float64 target_rear_distance
float64 target_left_distance
float64 target_right_distance
# 注意：target_angle 是'角度偏移量'，即希望機器人從當前姿態旋轉多少度
float64 target_angle

# --- 控制開關 ---
# 決定哪些自由度是本次任務的控制目標。
# 設為 True 的項目，Navigator 會嘗試將其控制到目標值。
# 設為 False 的項目，將被完全忽略。
bool control_front
bool control_rear
bool control_left
bool control_right
bool control_angle
