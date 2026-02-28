import time
import numpy as np
import cv2
from cvzone.FaceDetectionModule import FaceDetector
from DM_CAN import *
from linear import *
from import_math import *
import threading # 导入线程模块

# 全局变量，用于线程间通信
latest_frame = None
is_running = True

# 参数设置 
CAMERA_FOV_H = 83
CAMERA_FOV_V = 55
YAW_DEADZONE_ANGLE = 1.0
PITCH_DEADZONE_ANGLE = 1.0
YAW_GAIN = 0.8
PITCH_GAIN = 0.8
YAW_INVERT = -1
PITCH_INVERT = 1
MIN_YAW_ANGLE = -90.0
MAX_YAW_ANGLE = 90.0
MIN_PITCH_ANGLE = -25.0
MAX_PITCH_ANGLE = 25.0

# 摄像头读取线程 
def frame_reader_task(cap):
    """
    这个函数在一个独立的线程中运行，它的唯一任务就是不停地读取摄像头的最新帧。
    """
    global latest_frame, is_running
    while is_running:
        success, frame = cap.read()
        if success:
            latest_frame = frame
        else:
            # 如果读取失败，稍微等待一下
            time.sleep(0.01)
    print("摄像头读取线程已停止。")


def setup_robot_and_motors():

    # 初始化
    Motor1 = Motor(DM_Motor_Type.DM4340, 0x01, 0x11)
    Motor2 = Motor(DM_Motor_Type.DM4340, 0x02, 0x12)
    Motor3 = Motor(DM_Motor_Type.DM4340, 0x03, 0x13)
    motors = [Motor1, Motor2, Motor3]

    robot = NeckMechanism(
        l_crank=13.5,
        l_rod=118,
        w_platform=22.41,
        h_platform=24.23,
        d_platform=24.05,
        w_motor=35.58,
        d_motor=24.05,
        z_motor_offset=-97.57
    )

    ctrl, serial_dev = init_motors('/dev/ttyACM0', motors)
    enable_motors(ctrl, motors)
    kp_list = [13, 13, 13]
    kd_list = [1, 1, 1]
    dt_val = 0.001
    
    # 初始化电机到零位姿
    zero_targets = [0, 2.64, 2.85]
    move_to_target_linear(
        ctrl, motors, zero_targets, kp_list, kd_list,
        desired_velocity=0.5, dt=dt_val
    )
    # 初始化摄像头和人脸检测器
    cap = None
    detector = None
    reader_thread = None
    try:
        cap = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L2)
        if not cap.isOpened():
            raise IOError("摄像头打开失败。")
         # 在这里获取并打印FPS 
        fps = cap.get(cv2.CAP_PROP_FPS)
        print(f"摄像头报告的理论帧率 (FPS): {fps}")
        # 尝试设置一个小的缓冲区，这是一个有益的补充
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        detector = FaceDetector(minDetectionCon=0.7)

        # 启动摄像头读取线程
        reader_thread = threading.Thread(target=frame_reader_task, args=(cap,))
        reader_thread.daemon = True # 守护线程，主程序退出同时它也退出
        reader_thread.start()
        
        # 确保第一帧已经被读取
        time.sleep(1.0)
    except Exception as e:
        print(f"错误: 初始化过程中断: {e}")
        cleanup_and_exit(ctrl, motors, serial_dev, cap)
        return None, None, None, None, None, None, None, None, None, None
    return robot, ctrl, serial_dev, motors, kp_list, kd_list, dt_val, cap, detector, reader_thread

def run_main_task(robot, ctrl, motors, kp_list, kd_list, dt_val, cap, detector):
    current_yaw = 0.0
    current_pitch = 0.0
    print("\n步骤 2: 进入人脸追踪主循环 (按 'Esc' 键退出)")
    try:
        while True:
            # --- 核心改动：从全局变量获取最新帧 ---
            if latest_frame is None:
                # 如果还没有收到任何帧，就短暂等待并继续
                time.sleep(0.01)
                continue
            
            # 使用 .copy() 来确保我们处理的是一个完整的帧，避免线程冲突
            frame = latest_frame.copy()

            # --- 后续所有逻辑与之前优化后的版本完全相同 ---
            frame = cv2.flip(frame, 1)
            h, w, _ = frame.shape
            center_x, center_y = w // 2, h // 2
            
            scale = 0.5
            small_frame = cv2.resize(frame, (0, 0), fx=scale, fy=scale)
            _, bboxs = detector.findFaces(small_frame, draw=False)

            yaw_change = 0.0
            pitch_change = 0.0

            if bboxs:
                face_center_small = bboxs[0]["center"]
                face_cx = int(face_center_small[0] / scale)
                face_cy = int(face_center_small[1] / scale)
                bbox_scaled = [int(val / scale) for val in bboxs[0]["bbox"]]
                cv2.rectangle(frame, (bbox_scaled[0], bbox_scaled[1]), (bbox_scaled[0] + bbox_scaled[2], bbox_scaled[1] + bbox_scaled[3]), (255, 0, 255), 2)

                offset_x = face_cx - center_x
                offset_y = face_cy - center_y
                degrees_per_pixel_x = CAMERA_FOV_H / w
                degrees_per_pixel_y = CAMERA_FOV_V / h
                raw_angle_yaw = offset_x * degrees_per_pixel_x
                raw_angle_pitch = -offset_y * degrees_per_pixel_y

                if abs(raw_angle_yaw) > YAW_DEADZONE_ANGLE:
                    yaw_change = raw_angle_yaw * YAW_GAIN * YAW_INVERT
                if abs(raw_angle_pitch) > PITCH_DEADZONE_ANGLE:
                    pitch_change = raw_angle_pitch * PITCH_GAIN * PITCH_INVERT

            new_target_yaw = current_yaw + yaw_change
            new_target_pitch = current_pitch + pitch_change
            new_target_yaw = max(MIN_YAW_ANGLE, min(MAX_YAW_ANGLE, new_target_yaw))
            new_target_pitch = max(MIN_PITCH_ANGLE, min(MAX_PITCH_ANGLE, new_target_pitch))
            current_yaw = new_target_yaw
            current_pitch = new_target_pitch

            my_pose_command = [current_yaw, current_pitch, 0]
            set_neck_pose(robot, ctrl, motors, kp_list, kd_list, dt_val, my_pose_command)
            
            text = f"Target Angle -> Yaw: {current_yaw:.2f}, Pitch: {current_pitch:.2f}"
            cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            cv2.imshow("Face Tracking Robot Control", frame)

            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                print("\n检测到 'Esc' 按键，准备退出。")
                break
                
    except KeyboardInterrupt:
        print("\n检测到 Ctrl+C，准备退出。")
        pass

def cleanup_and_exit(ctrl, motors, serial_dev, cap):
    global is_running
    print("\n--- 步骤 3: 清理资源并安全退出 ---")
    
    # --- 核心改动：通知读取线程停止 ---
    print("正在停止摄像头读取线程...")
    is_running = False
    # 给线程一点时间来干净地退出
    time.sleep(0.5)

    if ctrl and motors:
        print("失能所有电机...")
        disable_all(ctrl, motors)
    if serial_dev and serial_dev.is_open:
        print("关闭串口...")
        serial_dev.close()
    
    if cap:
        print("释放摄像头...")
        cap.release()
    cv2.destroyAllWindows()
    print("--- 程序已安全退出。 ---")


if __name__ == "__main__":
    robot, ctrl, serial_dev, motors, kp, kd, dt, cap, detector, thread = setup_robot_and_motors()
    if all(v is not None for v in [robot, ctrl, serial_dev, motors, cap, detector, thread]):
        try:
            run_main_task(robot, ctrl, motors, kp, kd, dt, cap, detector)
        finally:
            cleanup_and_exit(ctrl, motors, serial_dev, cap)

