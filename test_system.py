#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
系统测试脚本
用于验证各个组件是否正常工作
"""

import cv2
import numpy as np
import time
import sys

def test_camera():
    """测试摄像头功能"""
    print("📷 测试摄像头...")
    
    cap = cv2.VideoCapture(1)
    if not cap.isOpened():
        print("❌ 摄像头测试失败")
        return False
    
    print("✅ 摄像头打开成功")
    
    # 拍摄一张测试图片
    ret, frame = cap.read()
    if ret:
        print("✅ 摄像头拍摄成功")
        cv2.imshow("测试图片", frame)
        cv2.waitKey(2000)
        cv2.destroyAllWindows()
    else:
        print("❌ 摄像头拍摄失败")
        return False
    
    cap.release()
    return True

def test_red_detection():
    """测试红色检测功能"""
    print("🔍 测试红色检测...")
    
    # 创建一个测试图像（红色矩形）
    test_image = np.zeros((400, 600, 3), dtype=np.uint8)
    cv2.rectangle(test_image, (200, 150), (400, 250), (0, 0, 255), -1)
    
    # 转换到HSV并检测红色
    hsv = cv2.cvtColor(test_image, cv2.COLOR_BGR2HSV)
    
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([180, 255, 255])
    
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = mask1 + mask2
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest_contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            print(f"✅ 红色检测成功，检测到位置: ({cx}, {cy})")
            return True
    
    print("❌ 红色检测失败")
    return False

def test_inverse_kinematics():
    """测试逆运动学"""
    print("🧮 测试逆运动学...")
    
    try:
        from InverseKinematic import inverse_kinematics
        
        # 测试一个可达的位置
        test_position = [100, 100, 50]
        result = inverse_kinematics(test_position)
        
        if result is not None:
            print(f"✅ 逆运动学测试成功，关节角度: {result}")
            return True
        else:
            print("❌ 逆运动学计算失败")
            return False
            
    except ImportError:
        print("❌ 无法导入逆运动学模块")
        return False
    except Exception as e:
        print(f"❌ 逆运动学测试出错: {e}")
        return False

def test_forward_controller():
    """测试正向控制器"""
    print("🤖 测试正向控制器...")
    
    try:
        from ForwardController import BraccioRobot
        
        # 创建机器人实例（不连接串口）
        robot = BraccioRobot(com_port='COM99')  # 使用不存在的串口进行测试
        
        # 测试DH参数
        if hasattr(robot, 'dh_parameters') and len(robot.dh_parameters) == 5:
            print("✅ 正向控制器DH参数正确")
            return True
        else:
            print("❌ 正向控制器DH参数错误")
            return False
            
    except ImportError:
        print("❌ 无法导入正向控制器模块")
        return False
    except Exception as e:
        print(f"❌ 正向控制器测试出错: {e}")
        return False

def test_compensation():
    """测试补偿功能"""
    print("🔧 测试补偿功能...")
    
    try:
        from RobotCompensation import apply_joint_compensation, apply_position_compensation
        
        # 测试关节补偿
        original_angles = [0, 90, 180, 90, 0]
        compensated_angles = apply_joint_compensation(*original_angles)
        print(f"✅ 关节补偿测试成功: {compensated_angles}")
        
        # 测试位置补偿
        original_position = [100, 100, 50]
        compensated_position = apply_position_compensation(*original_position)
        print(f"✅ 位置补偿测试成功: {compensated_position}")
        
        return True
        
    except ImportError:
        print("❌ 无法导入补偿模块")
        return False
    except Exception as e:
        print(f"❌ 补偿功能测试出错: {e}")
        return False

def test_hand_eye_calibration():
    """测试手眼标定类"""
    print("📐 测试手眼标定类...")
    
    try:
        from HandEyeCalibration import HandEyeCalibration
        
        # 创建标定实例
        calibration = HandEyeCalibration()
        
        # 测试类的基本功能
        if hasattr(calibration, 'calibrate') and hasattr(calibration, 'pixel_to_physical'):
            print("✅ 手眼标定类创建成功")
            return True
        else:
            print("❌ 手眼标定类缺少必要方法")
            return False
            
    except ImportError:
        print("❌ 无法导入手眼标定模块")
        return False
    except Exception as e:
        print(f"❌ 手眼标定测试出错: {e}")
        return False

def main():
    """主测试函数"""
    print("="*60)
    print("🧪 系统组件测试")
    print("="*60)
    
    tests = [
        ("摄像头功能", test_camera),
        ("红色检测", test_red_detection),
        ("逆运动学", test_inverse_kinematics),
        ("正向控制器", test_forward_controller),
        ("补偿功能", test_compensation),
        ("手眼标定", test_hand_eye_calibration)
    ]
    
    passed = 0
    total = len(tests)
    
    for test_name, test_func in tests:
        print(f"\n🔍 测试: {test_name}")
        print("-" * 40)
        
        try:
            if test_func():
                passed += 1
                print(f"✅ {test_name} 测试通过")
            else:
                print(f"❌ {test_name} 测试失败")
        except Exception as e:
            print(f"❌ {test_name} 测试异常: {e}")
    
    print("\n" + "="*60)
    print(f"📊 测试结果: {passed}/{total} 通过")
    print("="*60)
    
    if passed == total:
        print("🎉 所有测试通过！系统可以正常运行。")
        return True
    else:
        print("⚠️ 部分测试失败，请检查相关组件。")
        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1) 