import cv2

def open_camera(device_id="/dev/video0"):
    cap = cv2.VideoCapture(device_id, cv2.CAP_V4L2)
    if not cap.isOpened():
        print("摄像头打开失败")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print("读取视频帧失败")
            break

        cv2.imshow("摄像头预览", frame)
        if cv2.waitKey(1) & 0xFF == 27:  # 按ESC退出
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    open_camera()
