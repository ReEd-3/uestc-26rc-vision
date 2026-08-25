uint8 BLUE=0
uint8 RED=1

uint8 color
float32 x_m        # 相对相机：前为正（对应 z_forward_mm）
float32 y_m        # 相对相机：左为正（对应 -x_right_mm）
float32 yaw_rad    # 正负方向与当前 yaw_deg 保持一致
