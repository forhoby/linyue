#!/usr/bin/env python3
import cv2
from cvzone.FaceDetectionModule import FaceDetector
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge
#视角转化

CAMERA_FOV_H = 60  # 摄像头的水平视角 (Horizontal Field of View) in degrees
CAMERA_FOV_V = 45  # 摄像头的垂直视角 (Vertical Field of View) in degrees

class CameraNode(Node):
    def __init__(self):
        super().__init__("VideoAngle_node")
        # 创建图像发布者
        self.image_publisher_ = self.create_publisher(Image, "camera/image", 10)
        # 创建角度发布者
        self.angle_publisher_ = self.create_publisher(Float64MultiArray, "face/angle", 10)
        # 创建CvBridge用于转换图像格式
        self.bridge = CvBridge()
        # 初始化摄像头
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("摄像头打开失败")
            return
        
        # 使用 cvzone 的人脸检测器，置信度阈值设为 0.7
        self.detector = FaceDetector(minDetectionCon=0.7)
        
        # 创建窗口显示图像
        cv2.namedWindow("Camera", cv2.WINDOW_AUTOSIZE)
        
        # 开始主循环
        self.main_loop()
    
    def main_loop(self):
        print("--- 开始计算人脸偏移角度，按 'Esc' 键退出 ---")
        
        while rclpy.ok():
            # 读取一帧画面
            success, frame = self.cap.read()
            if not success:
                self.get_logger().error("读取视频帧失败")
                break

            # a. 准备工作：画面水平翻转（像照镜子）
            frame = cv2.flip(frame, 1)

            # b. 获取画面的高度、宽度和中心点
            h, w, _ = frame.shape
            center_x, center_y = w // 2, h // 2

            # c. 使用 cvzone 检测器检测人脸 (不让它自动绘制)
            frame, bboxs = self.detector.findFaces(frame, draw=False)

            # d. 核心计算：如果检测到人脸
            if bboxs:
                # i. 只取第一个检测到的人脸
                bbox = bboxs[0]
                face_cx, face_cy = bbox["center"]

                # ii. 计算人脸中心点相对于画面中心点的像素偏移量
                offset_x = face_cx - center_x
                offset_y = face_cy - center_y # 注意：OpenCV的Y轴向下为正

                # --- 新增：步骤 2: 将像素偏移量转换为角度 ---
                # 计算每像素对应的角度
                degrees_per_pixel_x = CAMERA_FOV_H / w
                degrees_per_pixel_y = CAMERA_FOV_V / h

                # 计算头部需要转动的偏航角(Yaw)和俯仰角(Pitch)
                # 偏航角(Yaw): 水平转动，我们定义向右为正
                angle_yaw = offset_x * degrees_per_pixel_x
                
                # 俯仰角(Pitch): 垂直转动，我们定义向上为正
                # 因为OpenCV的Y轴向下是正，所以offset_y向上时为负。我们需要一个负号来校正
                angle_pitch = -offset_y * degrees_per_pixel_y

                # iii. 在终端打印出核心的角度数据
                print(f"核心数据 -> Yaw: {angle_yaw:.2f} 度, Pitch: {angle_pitch:.2f} 度")
                
                # 发布角度数据
                angle_msg = Float64MultiArray()
                angle_msg.data = [angle_yaw, angle_pitch]
                self.angle_publisher_.publish(angle_msg)

                # --- (可选) 绘制调试信息 ---
                x, y, box_w, box_h = bbox['bbox']
                cv2.rectangle(frame, (x, y), (x + box_w, y + box_h), (0, 255, 0), 2)
                cv2.circle(frame, (face_cx, face_cy), 5, (0, 255, 255), -1) # 人脸中心
                cv2.circle(frame, (center_x, center_y), 5, (255, 0, 0), -1) # 画面中心
                cv2.line(frame, (center_x, center_y), (face_cx, face_cy), (0, 255, 255), 2)

                # 更新屏幕上的文本，显示角度
                text = f"Yaw: {angle_yaw:.2f} deg, Pitch: {angle_pitch:.2f} deg"
                cv2.putText(frame, text, (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

            else:
                print("未检测到人脸")
                # 即使没检测到人脸，也画出画面中心点
                cv2.circle(frame, (center_x, center_y), 5, (255, 0, 0), -1)
            
            # 发布图像
            try:
                msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                self.image_publisher_.publish(msg)
            except Exception as e:
                self.get_logger().error(f"发布图像失败: {e}")

            # 显示画面
            cv2.imshow("Camera", frame)

            # 按 Esc 键退出
            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                break

        # 释放资源
        self.cap.release()
        cv2.destroyAllWindows()
        print("--- 程序已退出。 ---")

def main():
    # 初始化ROS2
    rclpy.init()
    # 创建节点
    node = CameraNode()
    # 关闭ROS2
    rclpy.shutdown()

if __name__ == "__main__":
    main()