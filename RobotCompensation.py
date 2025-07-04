"""
机械臂误差补偿配置
用于手动调整机械臂的位置控制，补偿机械误差
"""

# ==================== 机械臂位置补偿参数 ====================
# 这些参数用于补偿机械臂的机械误差
# 正值表示向正方向补偿，负值表示向负方向补偿

# 基座关节补偿 (度)
BASE_OFFSET = 0.0

# 肩部关节补偿 (度)
SHOULDER_OFFSET = 0.0

# 肘部关节补偿 (度)
ELBOW_OFFSET = 0.0

# 腕部关节补偿 (度)
WRIST_OFFSET = 0.0

# 扭转关节补偿 (度)
TWIST_OFFSET = 0.0

# ==================== 位置补偿参数 ====================
# 用于补偿末端执行器到达位置的误差

# X方向位置补偿 (毫米)
X_POSITION_OFFSET = 0.0

# Y方向位置补偿 (毫米)
Y_POSITION_OFFSET = 0.0

# Z方向位置补偿 (毫米)
Z_POSITION_OFFSET = 0.0

# ==================== 抓取参数配置 ====================
# 抓取相关的参数设置

# 抓取高度 (毫米) - 机械臂末端到目标物体的距离
GRIP_HEIGHT = 50.0

# 夹持器开合角度
GRIPPER_OPEN_ANGLE = 0  # 完全打开
GRIPPER_CLOSE_ANGLE = 90   # 完全闭合
GRIPPER_GRAB_ANGLE = 50   # 抓取时的角度

# 移动时间 (毫秒)
MOVE_TIME = 50

# ==================== 摄像头参数配置 ====================
# 摄像头相关的参数设置

# 摄像头索引
CAMERA_INDEX = 1

# 红色检测阈值调整
RED_DETECTION_THRESHOLD = {
    'lower_red1': [0, 100, 100],
    'upper_red1': [10, 255, 255],
    'lower_red2': [160, 100, 100],
    'upper_red2': [180, 255, 255],
    'min_area': 1000  # 最小检测面积
}

# ==================== 工作区域限制 ====================
# 机械臂工作区域的限制参数

# 工作台高度 (毫米)
WORKTABLE_HEIGHT = 0.0

# 安全高度 (毫米) - 机械臂移动时的安全高度
SAFE_HEIGHT = 30.0

# ==================== 调试参数 ====================
# 调试和测试相关的参数

# 是否启用调试模式
DEBUG_MODE = True

# 是否显示摄像头画面
SHOW_CAMERA_FEED = True

# 是否保存检测图像
SAVE_DETECTION_IMAGES = False

# 图像保存路径
IMAGE_SAVE_PATH = "./detection_images/"

# ==================== 串口通信参数 ====================
# 机械臂串口通信参数

# 串口号
SERIAL_PORT = 'COM5'

# 波特率
BAUD_RATE = 115200

# 超时时间 (秒)
TIMEOUT = 5

# ==================== 应用补偿的函数 ====================

def apply_joint_compensation(base, shoulder, elbow, wrist, twist):
    """
    应用关节角度补偿
    
    Args:
        base: 基座角度
        shoulder: 肩部角度
        elbow: 肘部角度
        wrist: 腕部角度
        twist: 扭转角度
        
    Returns:
        tuple: 补偿后的关节角度
    """
    compensated_base = base + BASE_OFFSET
    compensated_shoulder = shoulder + SHOULDER_OFFSET
    compensated_elbow = elbow + ELBOW_OFFSET
    compensated_wrist = wrist + WRIST_OFFSET
    compensated_twist = twist + TWIST_OFFSET
    
    return (compensated_base, compensated_shoulder, compensated_elbow, 
            compensated_wrist, compensated_twist)

def apply_position_compensation(x, y, z):
    """
    应用位置补偿
    
    Args:
        x: X坐标
        y: Y坐标
        z: Z坐标
        
    Returns:
        tuple: 补偿后的位置坐标
    """
    compensated_x = x + X_POSITION_OFFSET
    compensated_y = y + Y_POSITION_OFFSET
    compensated_z = z + Z_POSITION_OFFSET
    
    return (compensated_x, compensated_y, compensated_z)

# ==================== 参数验证函数 ====================

def validate_compensation_parameters():
    """
    验证补偿参数的有效性
    
    Returns:
        bool: 参数是否有效
    """
    # 检查关节角度补偿是否在合理范围内
    joint_offsets = [BASE_OFFSET, SHOULDER_OFFSET, ELBOW_OFFSET, WRIST_OFFSET, TWIST_OFFSET]
    for offset in joint_offsets:
        if abs(offset) > 30:  # 假设最大补偿角度为30度
            print(f"⚠️ 警告: 关节补偿角度 {offset} 可能过大")
    
    # 检查位置补偿是否在合理范围内
    position_offsets = [X_POSITION_OFFSET, Y_POSITION_OFFSET, Z_POSITION_OFFSET]
    for offset in position_offsets:
        if abs(offset) > 50:  # 假设最大位置补偿为50mm
            print(f"⚠️ 警告: 位置补偿 {offset} 可能过大")
    
    return True

def print_compensation_status():
    """
    打印当前补偿参数状态
    """
    print("=" * 50)
    print("🔧 机械臂补偿参数状态:")
    print("=" * 50)
    print(f"基座补偿: {BASE_OFFSET}°")
    print(f"肩部补偿: {SHOULDER_OFFSET}°")
    print(f"肘部补偿: {ELBOW_OFFSET}°")
    print(f"腕部补偿: {WRIST_OFFSET}°")
    print(f"扭转补偿: {TWIST_OFFSET}°")
    print("-" * 30)
    print(f"X位置补偿: {X_POSITION_OFFSET}mm")
    print(f"Y位置补偿: {Y_POSITION_OFFSET}mm")
    print(f"Z位置补偿: {Z_POSITION_OFFSET}mm")
    print("-" * 30)
    print(f"抓取高度: {GRIP_HEIGHT}mm")
    print(f"安全高度: {SAFE_HEIGHT}mm")
    print("=" * 50)

# 在模块加载时验证参数
if __name__ == "__main__":
    validate_compensation_parameters()
    print_compensation_status() 