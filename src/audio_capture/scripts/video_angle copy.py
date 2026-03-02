#!/usr/bin/env python3
import cv2
import numpy as np
from cvzone.FaceDetectionModule import FaceDetector
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge
from typing import Tuple, List
import time

CAMERA_FOV_H = 60
CAMERA_FOV_V = 45

class MotionDetector:
    """移动目标检测器 - 基于帧差和运动补偿"""
    
    def __init__(self):
        self.prev_gray = None
        self.sift = cv2.SIFT_create()
        FLANN_INDEX_KDTREE = 1
        index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
        search_params = dict(checks=50)
        self.flann = cv2.FlannBasedMatcher(index_params, search_params)
    
    def compute_motion_saliency(self, current_frame: np.ndarray) -> np.ndarray:
        """
        计算运动显著性图 - 包含运动补偿
        
        Args:
            current_frame: 当前帧
            
        Returns:
            运动显著性图 (归一化到 [0, 1])
        """
        current_gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
        h, w = current_gray.shape
        motion_saliency = np.zeros((h, w), dtype=np.float32)
        
        if self.prev_gray is not None:
            try:
                homography = self._estimate_homography(self.prev_gray, current_gray)
                
                if homography is not None:
                    compensated_prev = cv2.warpPerspective(self.prev_gray, homography, (w, h))
                    frame_diff = cv2.absdiff(current_gray, compensated_prev)
                    motion_saliency = cv2.blur(frame_diff.astype(np.float32), (5, 5))
                else:
                    frame_diff = cv2.absdiff(current_gray, self.prev_gray)
                    motion_saliency = cv2.blur(frame_diff.astype(np.float32), (5, 5))
                    
            except Exception as e:
                frame_diff = cv2.absdiff(current_gray, self.prev_gray)
                motion_saliency = cv2.blur(frame_diff.astype(np.float32), (5, 5))
        
        if motion_saliency.max() > 0:
            motion_saliency = cv2.normalize(motion_saliency, None, 0, 1, cv2.NORM_MINMAX, cv2.CV_32F)
        
        self.prev_gray = current_gray.copy()
        return motion_saliency
    
    def _estimate_homography(self, img1: np.ndarray, img2: np.ndarray) -> np.ndarray:
        """使用SIFT特征估计两帧之间的单应性矩阵"""
        kp1, des1 = self.sift.detectAndCompute(img1, None)
        kp2, des2 = self.sift.detectAndCompute(img2, None)
        
        if des1 is None or des2 is None or len(des1) < 4 or len(des2) < 4:
            return None
        
        matches = self.flann.knnMatch(des1, des2, k=2)
        
        good_matches = []
        for match_pair in matches:
            if len(match_pair) == 2:
                m, n = match_pair
                if m.distance < 0.7 * n.distance:
                    good_matches.append(m)
        
        if len(good_matches) < 4:
            return None
        
        src_pts = np.float32([kp1[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
        dst_pts = np.float32([kp2[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)
        
        homography, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
        
        return homography
    
    def get_motion_centroid(self, motion_saliency: np.ndarray, threshold: float = 0.3) -> Tuple[int, int]:
        """获取运动显著性图的质心位置"""
        _, binary = cv2.threshold(motion_saliency, threshold, 1.0, cv2.THRESH_BINARY)
        binary_8u = (binary * 255).astype(np.uint8)
        
        contours, _ = cv2.findContours(binary_8u, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return (-1, -1)
        
        largest_contour = max(contours, key=cv2.contourArea)
        
        M = cv2.moments(largest_contour)
        if M["m00"] == 0:
            return (-1, -1)
        
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        
        return (cx, cy)

def visualize_motion_detection(frame: np.ndarray, motion_saliency: np.ndarray,
                               face_detected: bool, face_center: Tuple[int, int],
                               motion_center: Tuple[int, int], motion_strength: float,
                               final_angle: Tuple[float, float]) -> np.ndarray:
    """
    可视化运动检测结果 - 仿照 eyes.py 的风格
    
    Args:
        frame: 原始图像
        motion_saliency: 运动显著性图
        face_detected: 是否检测到人脸
        face_center: 人脸中心坐标
        motion_center: 运动中心坐标
        motion_strength: 运动强度
        final_angle: 最终角度
        
    Returns:
        可视化结果图像
    """
    h, w = frame.shape[:2]
    
    result_height = h * 2
    result_width = w * 2
    result_image = np.zeros((result_height, result_width, 3), dtype=np.uint8)
    
    result_image[0:h, 0:w] = frame
    
    motion_colored = cv2.applyColorMap((motion_saliency * 255).astype(np.uint8), cv2.COLORMAP_JET)
    result_image[0:h, w:2*w] = motion_colored
    
    overlay = frame.copy()
    saliency_mask = (motion_saliency * 255).astype(np.uint8)
    saliency_colored = cv2.applyColorMap(saliency_mask, cv2.COLORMAP_JET)
    overlay = cv2.addWeighted(overlay, 0.6, saliency_colored, 0.4, 0)
    result_image[h:2*h, 0:w] = overlay
    
    info_panel = np.zeros((h, w, 3), dtype=np.uint8)
    info_panel[:] = (30, 30, 30)
    
    font = cv2.FONT_HERSHEY_SIMPLEX
    
    cv2.putText(info_panel, 'Detection Results', (10, 40), font, 1.0, (255, 255, 255), 2)
    
    cv2.putText(info_panel, f'Yaw: {final_angle[0]:.2f} deg', (10, 80), font, 0.8, (0, 255, 255), 2)
    cv2.putText(info_panel, f'Pitch: {final_angle[1]:.2f} deg', (10, 115), font, 0.8, (0, 255, 255), 2)
    
    cv2.putText(info_panel, f'Motion Strength: {motion_strength:.3f}', (10, 160), font, 0.7, (255, 200, 100), 2)
    
    face_status = "Detected" if face_detected else "Not Detected"
    face_color = (0, 255, 0) if face_detected else (0, 0, 255)
    cv2.putText(info_panel, f'Face: {face_status}', (10, 200), font, 0.7, face_color, 2)
    
    motion_detected = motion_center != (-1, -1)
    motion_status = "Detected" if motion_detected else "Not Detected"
    motion_color = (0, 255, 0) if motion_detected else (0, 0, 255)
    cv2.putText(info_panel, f'Motion: {motion_status}', (10, 240), font, 0.7, motion_color, 2)
    
    cv2.putText(info_panel, 'Legend:', (10, 290), font, 0.7, (255, 255, 255), 2)
    cv2.circle(info_panel, (30, 330), 8, (0, 255, 0), -1)
    cv2.putText(info_panel, 'Face Center', (50, 335), font, 0.6, (0, 255, 0), 2)
    
    cv2.circle(info_panel, (30, 370), 8, (0, 0, 255), -1)
    cv2.putText(info_panel, 'Motion Center', (50, 375), font, 0.6, (0, 0, 255), 2)
    
    cv2.circle(info_panel, (30, 410), 5, (255, 0, 0), -1)
    cv2.putText(info_panel, 'Frame Center', (50, 415), font, 0.6, (255, 0, 0), 2)
    
    result_image[h:2*h, w:2*w] = info_panel
    
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(result_image, 'Original + Detection', (10, 30), font, 0.8, (255, 255, 255), 2)
    cv2.putText(result_image, 'Motion Saliency', (w + 10, 30), font, 0.8, (255, 255, 255), 2)
    cv2.putText(result_image, 'Saliency Overlay', (10, h + 30), font, 0.8, (255, 255, 255), 2)
    cv2.putText(result_image, 'Info Panel', (w + 10, h + 30), font, 0.8, (255, 255, 255), 2)
    
    return result_image

class CameraNode(Node):
    def __init__(self):
        super().__init__("VideoAngle_node")
        self.image_publisher_ = self.create_publisher(Image, "camera/image", 10)
        self.angle_publisher_ = self.create_publisher(Float64MultiArray, "face/angle", 10)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("摄像头打开失败")
            return
        
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        self.detector = FaceDetector(minDetectionCon=0.7)
        self.motion_detector = MotionDetector()
        
        cv2.namedWindow("Motion Detection System", cv2.WINDOW_NORMAL)
        
        self.frame_count = 0
        self.fps_counter = time.time()
        
        self.main_loop()
    
    def compute_angle_from_point(self, point: Tuple[int, int], w: int, h: int) -> Tuple[float, float]:
        """从像素坐标计算角度"""
        center_x, center_y = w // 2, h // 2
        offset_x = point[0] - center_x
        offset_y = point[1] - center_y
        degrees_per_pixel_x = CAMERA_FOV_H / w
        degrees_per_pixel_y = CAMERA_FOV_V / h
        angle_yaw = offset_x * degrees_per_pixel_x
        angle_pitch = -offset_y * degrees_per_pixel_y
        return angle_yaw, angle_pitch
    
    def fuse_angles(self, face_angle: Tuple[float, float], face_detected: bool,
                   motion_angle: Tuple[float, float], motion_detected: bool,
                   motion_strength: float) -> Tuple[float, float]:
        """融合人脸检测和移动目标检测的角度"""
        if face_detected and motion_detected:
            weight_motion = 0.3 + motion_strength * 0.4
            weight_face = 1.0 - weight_motion
            
            final_yaw = weight_face * face_angle[0] + weight_motion * motion_angle[0]
            final_pitch = weight_face * face_angle[1] + weight_motion * motion_angle[1]
            return (final_yaw, final_pitch)
        elif face_detected:
            return face_angle
        elif motion_detected:
            return motion_angle
        else:
            return (0.0, 0.0)
    
    def main_loop(self):
        print("=" * 60)
        print("运动目标检测系统已启动")
        print("=" * 60)
        print("按 'Esc' 键退出程序")
        print("按 's' 键保存当前结果")
        print("=" * 60)
        
        while rclpy.ok():
            success, frame = self.cap.read()
            if not success:
                self.get_logger().error("读取视频帧失败")
                break

            frame = cv2.flip(frame, 1)
            h, w, _ = frame.shape
            center_x, center_y = w // 2, h // 2

            frame, bboxs = self.detector.findFaces(frame, draw=False)
            
            motion_saliency = self.motion_detector.compute_motion_saliency(frame)
            motion_centroid = self.motion_detector.get_motion_centroid(motion_saliency)
            motion_strength = np.mean(motion_saliency)

            face_detected = False
            face_angle = (0.0, 0.0)
            face_center = (-1, -1)
            motion_detected = False
            motion_angle = (0.0, 0.0)
            
            detection_frame = frame.copy()
            
            if bboxs:
                face_detected = True
                bbox = bboxs[0]
                face_cx, face_cy = bbox["center"]
                face_center = (face_cx, face_cy)
                face_angle = self.compute_angle_from_point((face_cx, face_cy), w, h)
                
                x, y, box_w, box_h = bbox['bbox']
                cv2.rectangle(detection_frame, (x, y), (x + box_w, y + box_h), (0, 255, 0), 2)
                cv2.circle(detection_frame, (face_cx, face_cy), 8, (0, 255, 0), -1)
                cv2.circle(detection_frame, (face_cx, face_cy), 12, (0, 255, 0), 2)
                cv2.line(detection_frame, (center_x, center_y), (face_cx, face_cy), (0, 255, 0), 2)
                cv2.putText(detection_frame, 'Face', (face_cx + 15, face_cy - 15), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            if motion_centroid != (-1, -1):
                motion_detected = True
                motion_angle = self.compute_angle_from_point(motion_centroid, w, h)
                
                cv2.drawMarker(detection_frame, motion_centroid, (0, 0, 255), 
                              cv2.MARKER_CROSS, 20, 3)
                cv2.circle(detection_frame, motion_centroid, 15, (0, 0, 255), 3)
                cv2.line(detection_frame, (center_x, center_y), motion_centroid, (0, 0, 255), 2)
                cv2.putText(detection_frame, 'Motion', (motion_centroid[0] + 20, motion_centroid[1] - 20), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            final_angle = self.fuse_angles(
                face_angle, face_detected,
                motion_angle, motion_detected,
                motion_strength
            )
            
            cv2.circle(detection_frame, (center_x, center_y), 5, (255, 0, 0), -1)
            
            result_image = visualize_motion_detection(
                detection_frame, motion_saliency,
                face_detected, face_center,
                motion_centroid, motion_strength,
                final_angle
            )
            
            self.frame_count += 1
            if self.frame_count % 10 == 0:
                current_time = time.time()
                fps = 10 / (current_time - self.fps_counter)
                self.fps_counter = current_time
                print(f"FPS: {fps:.2f} | Motion Strength: {motion_strength:.3f} | "
                      f"Yaw: {final_angle[0]:.2f} deg | Pitch: {final_angle[1]:.2f} deg")
            
            angle_msg = Float64MultiArray()
            angle_msg.data = [final_angle[0], final_angle[1]]
            self.angle_publisher_.publish(angle_msg)

            display_image = cv2.resize(result_image, None, fx=0.6, fy=0.6)
            cv2.imshow("Motion Detection System", display_image)

            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                break
            elif key == ord('s'):
                filename = f'motion_detection_{self.frame_count}.jpg'
                cv2.imwrite(filename, result_image)
                print(f"结果已保存到: {filename}")

        self.cap.release()
        cv2.destroyAllWindows()
        print("=" * 60)
        print("程序已退出")
        print("=" * 60)

def main():
    rclpy.init()
    node = CameraNode()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
