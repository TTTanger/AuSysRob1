import cv2
import numpy as np
import time
from typing import Tuple, Optional

class HandEyeCalibration:
    """
    手眼标定类
    用于校准摄像头像素坐标和机械臂基座物理坐标之间的转换关系
    """
    
    def __init__(self, camera_index: int = 0):
        """
        初始化手眼标定类
        
        Args:
            camera_index: 摄像头索引，默认为0
        """
        self.camera_index = camera_index
        self.cap = None
        self.compensation_vector = None
        self.scale_factor = None
        self.calibrated = False
        
    def open_camera(self) -> bool:
        """
        打开摄像头
        
        Returns:
            bool: 是否成功打开摄像头
        """
        try:
            self.cap = cv2.VideoCapture(self.camera_index)
            if not self.cap.isOpened():
                print(f"❌ 无法打开摄像头 {self.camera_index}")
                return False
            print(f"✅ 摄像头 {self.camera_index} 打开成功")
            return True
        except Exception as e:
            print(f"❌ 打开摄像头时发生错误: {e}")
            return False
    
    def close_camera(self):
        """关闭摄像头"""
        if self.cap and self.cap.isOpened():
            self.cap.release()
            print("📷 摄像头已关闭")
    
    def detect_red_lego(self, frame) -> Optional[Tuple[int, int]]:
        """
        检测红色乐高积木
        
        Args:
            frame: 输入图像帧
            
        Returns:
            Optional[Tuple[int, int]]: 检测到的红色积木中心像素坐标，如果未检测到则返回None
        """
        # 转换到HSV色彩空间
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # 定义红色范围（HSV）
        # 红色在HSV中跨越0度和180度，所以需要两个范围
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        
        # 创建红色掩码
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 + mask2
        
        # 形态学操作去除噪声
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        # 查找轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            # 找到最大的轮廓（假设是乐高积木）
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            
            # 面积阈值，过滤掉太小的区域
            if area > 1000:  # 可以根据实际情况调整
                # 计算轮廓的中心点
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    return (cx, cy)
        
        return None
    
    def capture_and_detect(self) -> Optional[Tuple[int, int]]:
        """
        拍摄图片并检测红色乐高积木
        
        Returns:
            Optional[Tuple[int, int]]: 检测到的像素坐标
        """
        if not self.cap or not self.cap.isOpened():
            print("Camera not opened")
            return None
        
        print("📷 正在拍摄图片...")
        ret, frame = self.cap.read()
        if not ret:
            print("Cannot read camera frame")
            return None
        
        # 检测红色乐高积木
        pixel_coord = self.detect_red_lego(frame)
        
        if pixel_coord:
            print(f"Detected red Lego, pixel coordinates: {pixel_coord}")
            
            # 在图像上标记检测到的位置
            cv2.circle(frame, pixel_coord, 10, (0, 255, 0), 2)
            cv2.putText(frame, f"({pixel_coord[0]}, {pixel_coord[1]})", 
                       (pixel_coord[0] + 15, pixel_coord[1] - 15),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # 显示图像
            cv2.imshow("检测结果", frame)
            cv2.waitKey(2000)  # 显示2秒
            cv2.destroyAllWindows()
            
            return pixel_coord
        else:
            print("❌ 未检测到红色乐高积木")
            return None
    
    def calibrate(self, physical_coord: Tuple[float, float]) -> bool:
        """
        执行手眼标定
        
        Args:
            physical_coord: 物理坐标 (x, y)，相对于机械臂基座中心点
            
        Returns:
            bool: 标定是否成功
        """
        print("🔧 开始手眼标定...")
        print("请将红色乐高积木放置在摄像头视野内，然后按任意键继续...")
        
        # 打开摄像头
        if not self.open_camera():
            return False
        
        try:
            # 实时显示摄像头画面
            while True:
                ret, frame = self.cap.read()
                if not ret:
                    print("❌ Cannot read camera frame")
                    break
                
                # 检测红色积木并实时显示
                pixel_coord = self.detect_red_lego(frame)
                if pixel_coord:
                    cv2.circle(frame, pixel_coord, 10, (0, 255, 0), 2)
                    cv2.putText(frame, f"Detected red Lego: {pixel_coord}", 
                               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                cv2.putText(frame, "Press 'c' to confirm, press 'q' to exit", 
                           (10, frame.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                
                cv2.imshow("Hand-Eye Calibration", frame)
                
                key = cv2.waitKey(1) & 0xFF
                if key == ord('c'):
                    break
                elif key == ord('q'):
                    print("❌ User cancelled calibration")
                    return False
            
            # 拍摄并检测
            pixel_coord = self.capture_and_detect()
            if not pixel_coord:
                return False
            
            # 计算补偿向量
            self.compensation_vector = (
                physical_coord[0] - pixel_coord[0],
                physical_coord[1] - pixel_coord[1]
            )
            
            # 计算缩放因子（可选，用于更精确的转换）
            # 这里假设像素和物理坐标的比例关系
            self.scale_factor = 1.0  # 可以根据实际情况调整
            
            self.calibrated = True
            print(f"✅ Calibration completed!")
            print(f"   Pixel coordinates: {pixel_coord}")
            print(f"   Physical coordinates: {physical_coord}")
            print(f"   Compensation vector: {self.compensation_vector}")
            
            return True
            
        finally:
            self.close_camera()
    
    def pixel_to_physical(self, pixel_coord: Tuple[int, int]) -> Tuple[float, float]:
        """
        将像素坐标转换为物理坐标
        
        Args:
            pixel_coord: 像素坐标 (x, y)
            
        Returns:
            Tuple[float, float]: 物理坐标 (x, y)
        """
        if not self.calibrated:
            raise ValueError("❌ Calibration not completed, please call calibrate() first")
        
        physical_x = pixel_coord[0] + self.compensation_vector[0]
        physical_y = pixel_coord[1] + self.compensation_vector[1]
        
        return (-physical_x, physical_y)
    
    def get_calibration_info(self) -> dict:
        """
        获取标定信息
        
        Returns:
            dict: 包含标定信息的字典
        """
        return {
            'calibrated': self.calibrated,
            'compensation_vector': self.compensation_vector,
            'scale_factor': self.scale_factor
        } 