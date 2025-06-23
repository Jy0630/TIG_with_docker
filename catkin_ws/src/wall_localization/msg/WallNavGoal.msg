# --- 目標值 (單位: 公尺, 度) ---
# Define the target distances and angle for the robot to navigate towards.
# 若某個目標值不需控制，請將其設為 -1.0。
# 例如：若只想控制前方距離為 0.5 公尺，則設定 target_front_distance = 0.5，其餘 target_*_distance 設為 -1.0。
float64 target_front_distance
float64 target_rear_distance
float64 target_left_distance
float64 target_right_distance

# 注意：target_angle 是'角度偏移量'，即希望機器人從當前姿態旋轉多少度。
# 若不需旋轉，請設為 -1.0。
# 特殊對齊模式：當 target_angle = 0.0 且 align_to_wall 有效時觸發。
float64 target_angle

# 特殊對齊模式：指定要對齊的牆面。
# 僅在 target_angle = 0.0 時有效。
# 可選值: "front", "rear", "left", "right"。若不使用則留空。
string align_to_wall

