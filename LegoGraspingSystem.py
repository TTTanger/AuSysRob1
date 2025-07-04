#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
乐高积木识别和抓取系统
整合手眼标定、逆运动学、正向控制和机械误差补偿
"""

import cv2
import numpy as np
import time
import os
from typing import Tuple, Optional, List
import sys

# 导入自定义模块
from HandEyeCalibration import HandEyeCalibration
from ForwardController import BraccioRobot
from InverseKinematic import inverse_kinematics
from RobotCompensation import (
    apply_joint_compensation, apply_position_compensation,
    GRIP_HEIGHT, GRIPPER_OPEN_ANGLE, GRIPPER_CLOSE_ANGLE, GRIPPER_GRAB_ANGLE,
    MOVE_TIME, CAMERA_INDEX, SAFE_HEIGHT, WORKTABLE_HEIGHT,
    DEBUG_MODE, SHOW_CAMERA_FEED, SAVE_DETECTION_IMAGES, IMAGE_SAVE_PATH,
    SERIAL_PORT, BAUD_RATE, TIMEOUT, print_compensation_status
)

class LegoGraspingSystem:
    """
    乐高积木识别和抓取系统主类
    """
    
    def __init__(self, camera_index: int = CAMERA_INDEX, 
                 serial_port: str = SERIAL_PORT,
                 baud_rate: int = BAUD_RATE,
                 timeout: int = TIMEOUT):
        """
        初始化系统
        
        Args:
            camera_index: 摄像头索引
            serial_port: 串口号
            baud_rate: 波特率
            timeout: 超时时间
        """
        self.camera_index = camera_index
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.timeout = timeout
        
        # 初始化组件
        self.calibration = HandEyeCalibration(camera_index)
        self.robot = BraccioRobot(serial_port, baud_rate, timeout)
        self.cap = None
        
        # 系统状态
        self.is_calibrated = False
        self.is_robot_connected = False
        self.is_camera_open = False
        
        # 创建图像保存目录
        if SAVE_DETECTION_IMAGES:
            os.makedirs(IMAGE_SAVE_PATH, exist_ok=True)
        
        print("🤖 乐高积木识别和抓取系统初始化完成")
        print_compensation_status()
    
    def initialize_system(self) -> bool:
        """
        初始化系统组件
        
        Returns:
            bool: 初始化是否成功
        """
        print("🔧 正在初始化系统组件...")
        
        # 检查机械臂连接
        if self.robot.s and self.robot.s.is_open:
            self.is_robot_connected = True
            print("✅ 机械臂连接正常")
        else:
            print("❌ 机械臂连接失败")
            return False
        
        # 打开摄像头
        if self.open_camera():
            self.is_camera_open = True
            print("✅ 摄像头打开成功")
        else:
            print("❌ 摄像头打开失败")
            return False
        
        return True
    
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
            return True
        except Exception as e:
            print(f"❌ 打开摄像头时发生错误: {e}")
            return False
    
    def close_camera(self):
        """关闭摄像头"""
        if self.cap and self.cap.isOpened():
            self.cap.release()
            self.is_camera_open = False
            print("📷 摄像头已关闭")
    
    def detect_red_lego(self, frame) -> Optional[Tuple[int, int]]:
        """
        检测红色乐高积木
        
        Args:
            frame: 输入图像帧
            
        Returns:
            Optional[Tuple[int, int]]: 检测到的红色积木中心像素坐标
        """
        # 转换到HSV色彩空间
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # 定义红色范围（HSV）
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
    
    def perform_hand_eye_calibration(self) -> bool:
        """
        执行手眼标定
        
        Returns:
            bool: 标定是否成功
        """
        print("\n" + "="*60)
        print("🔧 开始手眼标定流程")
        print("="*60)
        
        print("📋 标定步骤说明:")
        print("1. 请将红色乐高积木放置在摄像头视野内")
        print("2. 系统将拍摄图片并识别积木位置")
        print("3. 请测量积木相对于机械臂基座中心点的物理坐标")
        print("4. 输入物理坐标完成标定")
        print("-"*60)
        
        # 获取用户输入的物理坐标
        try:
            print("请输入红色乐高积木的物理坐标 (相对于机械臂基座中心点):")
            x = float(input("X坐标 (毫米): "))
            y = float(input("Y坐标 (毫米): "))
            physical_coord = (x, y)
            
            print(f"📏 输入的物理坐标: ({x}, {y}) 毫米")
            
        except ValueError:
            print("❌ 输入格式错误，请输入有效的数字")
            return False
        
        # 执行标定
        if self.calibration.calibrate(physical_coord):
            self.is_calibrated = True
            print("✅ 手眼标定完成！")
            return True
        else:
            print("❌ 手眼标定失败")
            return False
    
    def calculate_grasp_position(self, pixel_coord: Tuple[int, int]) -> Optional[Tuple[float, float, float]]:
        """
        计算抓取位置
        
        Args:
            pixel_coord: 像素坐标
            
        Returns:
            Optional[Tuple[float, float, float]]: 抓取位置 (x, y, z)
        """
        if not self.is_calibrated:
            print("❌ 尚未完成手眼标定")
            return None
        
        # 将像素坐标转换为物理坐标
        physical_coord_origin = self.calibration.pixel_to_physical(pixel_coord)
        x,y=physical_coord_origin

        if  x >=100:
            x = x-5
            y = y +38

        elif -80<= x <100:
            x= x-10
            y = y+28
        else :
            x= x-23
            y=y+14
        

        physical_coord = (x,y)


        # 应用位置补偿
        compensated_coord = apply_position_compensation(
            physical_coord[0], 
            physical_coord[1], 
            WORKTABLE_HEIGHT + GRIP_HEIGHT)
        
        # 应用位置补偿
        compensated_coord = apply_position_compensation(
            physical_coord[0], 
            physical_coord[1], 
            WORKTABLE_HEIGHT + GRIP_HEIGHT
        )
        
        # print(f"📍 像素坐标: {pixel_coord}")
        # print(f"📍 物理坐标: {physical_coord}")
        # print(f"📍 补偿后坐标: {compensated_coord}")
        
        return compensated_coord
    
    def check_reachability(self, position: Tuple[float, float, float]) -> bool:
        """
        检查位置是否可达
        
        Args:
            position: 目标位置 (x, y, z)
            
        Returns:
            bool: 是否可达
        """
        # 尝试进行逆运动学计算
        try:
            joint_angles = inverse_kinematics(position)
            if joint_angles is None:
                return False
            
            # 检查关节角度是否在合理范围内
            for i, angle in enumerate(joint_angles):
                if not (0 <= angle <= 180):
                    print(f"⚠️ 关节 {i+1} 角度 {angle:.2f}° 超出范围")
                    return False
            
            return True
            
        except Exception as e:
            print(f"❌ 逆运动学计算失败: {e}")
            return False
    
    def execute_grasp_sequence(self, position: Tuple[float, float, float]) -> bool:
        """
        执行抓取序列
        
        Args:
            position: 目标位置 (x, y, z)
            
        Returns:
            bool: 抓取是否成功
        """
        print(f"🤖 开始执行抓取序列，目标位置: {position}")
        
        try:
            # 1. 计算逆运动学
            joint_angles = inverse_kinematics(position)
            if joint_angles is None:
                print("❌ 逆运动学计算失败，位置不可达")
                return False
            
            # 2. 应用关节补偿
            compensated_angles = apply_joint_compensation(*joint_angles)
            
            print(f"🔧 原始关节角度: {joint_angles}")
            print(f"🔧 补偿后关节角度: {compensated_angles}")
            
            # 3. 移动到安全高度
            safe_position = (position[0], position[1], position[2] + SAFE_HEIGHT)
            safe_angles = inverse_kinematics(safe_position)
            if safe_angles:
                safe_compensated = apply_joint_compensation(*safe_angles)
                self.robot.move_to_angles(*safe_compensated, GRIPPER_OPEN_ANGLE, MOVE_TIME)
                time.sleep(2)
            
            # 4. 移动到抓取位置
            self.robot.move_to_angles(*compensated_angles, GRIPPER_OPEN_ANGLE, MOVE_TIME)
            time.sleep(2)
            
            # 5. 闭合夹持器
            self.robot.move_to_angles(*compensated_angles, GRIPPER_CLOSE_ANGLE, MOVE_TIME)
            time.sleep(1)
            
            # 6. 移动到安全高度
            if safe_angles:
                self.robot.move_to_angles(*safe_compensated, GRIPPER_CLOSE_ANGLE, MOVE_TIME)
                time.sleep(2)

            # 7. 回到初始位置
            self.robot.move_to_angles(0, 90, 180, 90, 0, GRIPPER_CLOSE_ANGLE, MOVE_TIME)
            time.sleep(2)

            # 8. 释放夹持器
            self.robot.move_to_angles(0, 90, 180, 90, 0, GRIPPER_OPEN_ANGLE, MOVE_TIME)
            time.sleep(2)
            
            print("✅ 抓取序列执行完成")
            return True
            
        except Exception as e:
            print(f"❌ 抓取序列执行失败: {e}")
            return False
    
    def run_detection_loop(self):
        """
        运行检测和抓取循环
        """
        print("\n" + "="*60)
        print("🔄 开始检测和抓取循环")
        print("="*60)
        print("按 'q' 退出循环，按 'r' 重新标定")
        
        if not self.is_calibrated:
            print("❌ 尚未完成手眼标定，请先进行标定")
            return
        
        while True:
            # 读取摄像头画面
            ret, frame = self.cap.read()
            if not ret:
                print("❌ 无法读取摄像头画面")
                break
            
            # 检测红色乐高积木
            pixel_coord = self.detect_red_lego(frame)
            
            # 显示检测结果
            if SHOW_CAMERA_FEED:
                display_frame = frame.copy()
                
                if pixel_coord:
                    # 标记检测到的积木
                    cv2.circle(display_frame, pixel_coord, 15, (0, 255, 0), 3)
                    cv2.putText(display_frame, f"Lego: {pixel_coord}", 
                               (pixel_coord[0] + 20, pixel_coord[1] - 20),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    
                    # 计算物理坐标
                    grasp_position = self.calculate_grasp_position(pixel_coord)
                    if grasp_position:
                        cv2.putText(display_frame, f"Pos: ({grasp_position[0]:.1f}, {grasp_position[1]:.1f})", 
                                   (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                
                # 添加控制提示
                cv2.putText(display_frame, "Press 'g' to grasp, 'q' to quit, 'r' to recalibrate", 
                           (10, display_frame.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                
                cv2.imshow("乐高积木检测", display_frame)
            
            # 处理键盘输入
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                print("👋 用户退出检测循环")
                break
            elif key == ord('r'):
                print("🔄 重新标定")
                if self.perform_hand_eye_calibration():
                    continue
                else:
                    break
            elif key == ord('g') and pixel_coord:
                print("🤖 开始抓取...")
                
                # 计算抓取位置
                grasp_position = self.calculate_grasp_position(pixel_coord)
                if not grasp_position:
                    print("❌ 无法计算抓取位置")
                    continue
                
                # 检查可达性
                if not self.check_reachability(grasp_position):
                    print("⚠️ 目标位置不可达，跳过此次抓取")
                    continue
                
                # 执行抓取
                if self.execute_grasp_sequence(grasp_position):
                    print("✅ 抓取成功！")
                    # 等待一段时间再进行下一次检测
                    time.sleep(3)
                else:
                    print("❌ 抓取失败")
        
        cv2.destroyAllWindows()
    
    def run(self):
        """
        运行主程序
        """
        print("🚀 启动乐高积木识别和抓取系统")
        
        # 初始化系统
        if not self.initialize_system():
            print("❌ 系统初始化失败")
            return
        
        try:
            # 执行手眼标定
            if not self.perform_hand_eye_calibration():
                print("❌ 手眼标定失败，程序退出")
                return
            
            # 运行检测和抓取循环
            self.run_detection_loop()
            
        except KeyboardInterrupt:
            print("\n⚠️ 用户中断程序")
        except Exception as e:
            print(f"❌ 程序运行出错: {e}")
        finally:
            # 清理资源
            self.cleanup()
    
    def cleanup(self):
        """清理资源"""
        print("🧹 正在清理资源...")
        
        # 关闭摄像头
        self.close_camera()
        
        # 关闭机械臂连接
        if self.robot:
            self.robot.close_serial()
        
        # 关闭所有OpenCV窗口
        cv2.destroyAllWindows()
        
        print("✅ 资源清理完成")

def main():
    """主函数"""
    print("="*60)
    print("🤖 乐高积木识别和抓取系统")
    print("="*60)
    
    # 创建系统实例
    system = LegoGraspingSystem()
    
    # 运行系统
    system.run()

if __name__ == "__main__":
    main() 