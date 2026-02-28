import cv2
from cvzone.FaceDetectionModule import FaceDetector

def main():
    # 1. 初始化：加载 cvzone 检测器并打开摄像头
    try:
        cap = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L2)
        if not cap.isOpened():
            print("摄像头打开失败")
            return
        
        # 使用 cvzone 的人脸检测器，置信度阈值设为 0.7
        detector = FaceDetector(minDetectionCon=0.7)

    except Exception as e:
        print(f"初始化失败: {e}")
        return

    print("--- 开始计算人脸偏移量，按 'Esc' 键退出 ---")

    # 2. 主循环
    while True:
        # 读取一帧画面
        success, frame = cap.read()
        if not success:
            print("读取视频帧失败")
            break

        # a. 准备工作：画面水平翻转（像照镜子）
        frame = cv2.flip(frame, 1)

        # b. 获取画面的高度、宽度和中心点
        h, w, _ = frame.shape
        print(f"画面宽度: {w} px, 画面高度: {h} px")
        center_x, center_y = w // 2, h // 2

        # c. 使用 cvzone 检测器检测人脸 (不让它自动绘制)
        frame, bboxs = detector.findFaces(frame, draw=False)

        # d. 核心计算：如果检测到人脸
        if bboxs:
            # i. 只取第一个检测到的人脸
            bbox = bboxs[0]
            face_cx, face_cy = bbox["center"]

            # ii. 计算人脸中心点相对于画面中心点的偏移量
            offset_x = face_cx - center_x
            offset_y = face_cy - center_y

            # iii. 在终端打印出核心的偏移量数据
            #      我们希望“向上”为正，所以对 offset_y 取反
            print(f"核心数据 -> Offset X: {offset_x}, Offset Y: {-offset_y}")

            # --- (可选) 绘制和你第二个脚本一样的调试信息 ---
            x, y, box_w, box_h = bbox['bbox']
            cv2.rectangle(frame, (x, y), (x + box_w, y + box_h), (0, 255, 0), 2)
            cv2.circle(frame, (face_cx, face_cy), 5, (0, 255, 255), -1)
            cv2.circle(frame, (center_x, center_y), 5, (255, 0, 0), -1)
            cv2.line(frame, (center_x, center_y), (face_cx, face_cy), (0, 255, 255), 2)
            cv2.putText(frame, f"Offset: ({offset_x}, {-offset_y})", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        else:
            print("未检测到人脸")
            cv2.circle(frame, (center_x, center_y), 5, (255, 0, 0), -1)

        # 3. 显示画面
        cv2.imshow("Offset Calculator (Optimized)", frame)

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
