import cv2
from cvzone.FaceDetectionModule import FaceDetector
#有死区的视角转换

# 步骤 1: 摄像头物理参数
# 电脑摄像头的视场角 (FOV) 参数
# CAMERA_FOV_H = 61.93  # 水平视角
# CAMERA_FOV_V = 77.32  # 垂直视角
CAMERA_FOV_H = 60  # 水平视角 
CAMERA_FOV_V = 45  # 垂直视角

# 步骤 2: 控制参数
# 1. 死区 (Deadzone): 角度小于此值时，电机不动作，防止在中心位置高频抖动。
YAW_DEADZONE_ANGLE = 2.0   # 水平方向的死区范围 (+/- 2.0度)
PITCH_DEADZONE_ANGLE = 2.0 # 垂直方向的死区范围 (+/- 2.0度)

# 2. 增益 (Gain): 调整电机的响应灵敏度。大于1更灵敏，小于1更平滑。
YAW_GAIN = 0.4
PITCH_GAIN = 0.4

# 3. 反向 (Invert): 如果电机转动方向反了，只需将 1 改为 -1，无需修改算法。
YAW_INVERT = 1   # 1 表示不反向, -1 表示反向
PITCH_INVERT = 1 # 1 表示不反向, -1 表示反向

def main():
    # 1. 初始化
    try:
        cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
        if not cap.isOpened():
            print("摄像头打开失败")
            return
        detector = FaceDetector(minDetectionCon=0.7)
    except Exception as e:
        print(f"初始化失败: {e}")
        return

    print("开始计算电机控制指令，按 'Esc' 键退出")

    # 2. 主循环
    while True:
        success, frame = cap.read()
        if not success:
            break
        
        frame = cv2.flip(frame, 1)
        h, w, _ = frame.shape
        center_x, center_y = w // 2, h // 2
        frame, bboxs = detector.findFaces(frame, draw=False)

        # 初始化最终的电机控制指令
        control_yaw = 0.0
        control_pitch = 0.0

        if bboxs:
            # a. 获取人脸位置和原始偏移
            bbox = bboxs[0]
            face_cx, face_cy = bbox["center"]
            offset_x = face_cx - center_x
            offset_y = face_cy - center_y

            # 步骤 3: 计算原始角度 (与之前相同)
            degrees_per_pixel_x = CAMERA_FOV_H / w
            degrees_per_pixel_y = CAMERA_FOV_V / h

            raw_angle_yaw = offset_x * degrees_per_pixel_x
            raw_angle_pitch = -offset_y * degrees_per_pixel_y

            #  步骤 4: 应用控制参数 (死区, 增益, 反向) 
            # i. 应用水平(Yaw)死区
            if abs(raw_angle_yaw) > YAW_DEADZONE_ANGLE:
                control_yaw = raw_angle_yaw * YAW_GAIN * YAW_INVERT
            else:
                control_yaw = 0.0 # 在死区内，不产生动作

            # ii. 应用垂直(Pitch)死区
            if abs(raw_angle_pitch) > PITCH_DEADZONE_ANGLE:
                control_pitch = raw_angle_pitch * PITCH_GAIN * PITCH_INVERT
            else:
                control_pitch = 0.0 # 在死区内，不产生动作

            # iii. 在终端打印出给电机的最终指令
            print(f"电机指令 -> Yaw: {control_yaw:.2f} 度, Pitch: {control_pitch:.2f} 度")

            # 绘制调试信息
            x, y, box_w, box_h = bbox['bbox']
            cv2.rectangle(frame, (x, y), (x + box_w, y + box_h), (0, 255, 0), 2)
            cv2.line(frame, (center_x, center_y), (face_cx, face_cy), (0, 255, 255), 2)
            cv2.circle(frame, (face_cx, face_cy), 5, (0, 255, 255), -1)
        else:
            print("未检测到人脸")
        
        cv2.circle(frame, (center_x, center_y), 5, (255, 0, 0), -1)
        
        # 更新屏幕上的文本，显示最终的控制指令
        text = f"Control Yaw: {control_yaw:.2f} | Pitch: {control_pitch:.2f}"
        cv2.putText(frame, text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        # 3. 显示画面
        cv2.imshow("Motor Control Calculator", frame)

        # 4. 按 Esc 键退出
        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            break

    # 5. 释放资源
    cap.release()
    cv2.destroyAllWindows()
    print("--- 程序已退出。 ---")

if __name__ == "__main__":
    main()
