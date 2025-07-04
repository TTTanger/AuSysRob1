#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
元组修改示例
展示如何解包、修改、再打包元组数据
"""

def modify_tuple_method1():
    """方法1：解包后重新打包"""
    print("="*50)
    print("方法1：解包后重新打包")
    print("="*50)
    
    # 原始元组
    original_tuple = (100, 200, 300)
    print(f"原始元组: {original_tuple}")
    
    # 解包
    x, y, z = original_tuple
    print(f"解包后: x={x}, y={y}, z={z}")
    
    # 修改值
    x = x - 5
    y = y + 20
    print(f"修改后: x={x}, y={y}, z={z}")
    
    # 重新打包为元组
    modified_tuple = (x, y, z)
    print(f"修改后的元组: {modified_tuple}")
    
    return modified_tuple

def modify_tuple_method2():
    """方法2：使用列表转换"""
    print("\n" + "="*50)
    print("方法2：使用列表转换")
    print("="*50)
    
    # 原始元组
    original_tuple = (100, 200, 300)
    print(f"原始元组: {original_tuple}")
    
    # 转换为列表
    temp_list = list(original_tuple)
    print(f"转换为列表: {temp_list}")
    
    # 修改列表
    if temp_list[0] >= 100:
        temp_list[0] = temp_list[0] - 5
        temp_list[1] = temp_list[1] + 20
    elif 0 <= temp_list[0] < 100:
        temp_list[1] = temp_list[1] + 20
    
    print(f"修改后的列表: {temp_list}")
    
    # 转换回元组
    modified_tuple = tuple(temp_list)
    print(f"修改后的元组: {modified_tuple}")
    
    return modified_tuple

def modify_tuple_method3():
    """方法3：使用命名元组（更优雅的方式）"""
    print("\n" + "="*50)
    print("方法3：使用命名元组")
    print("="*50)
    
    from collections import namedtuple
    
    # 定义命名元组
    Point = namedtuple('Point', ['x', 'y', 'z'])
    
    # 创建原始元组
    original_point = Point(100, 200, 300)
    print(f"原始命名元组: {original_point}")
    
    # 修改（创建新的命名元组）
    if original_point.x >= 100:
        modified_point = Point(original_point.x - 5, original_point.y + 20, original_point.z)
    elif 0 <= original_point.x < 100:
        modified_point = Point(original_point.x, original_point.y + 20, original_point.z)
    else:
        modified_point = original_point
    
    print(f"修改后的命名元组: {modified_point}")
    print(f"访问属性: x={modified_point.x}, y={modified_point.y}, z={modified_point.z}")
    
    return modified_point

def modify_tuple_method4():
    """方法4：使用字典解包（Python 3.5+）"""
    print("\n" + "="*50)
    print("方法4：使用字典解包")
    print("="*50)
    
    # 原始元组
    original_tuple = (100, 200, 300)
    print(f"原始元组: {original_tuple}")
    
    # 解包为字典
    x, y, z = original_tuple
    
    # 创建修改后的字典
    modified_dict = {'x': x, 'y': y, 'z': z}
    
    # 修改值
    if modified_dict['x'] >= 100:
        modified_dict['x'] = modified_dict['x'] - 5
        modified_dict['y'] = modified_dict['y'] + 20
    elif 0 <= modified_dict['x'] < 100:
        modified_dict['y'] = modified_dict['y'] + 20
    
    print(f"修改后的字典: {modified_dict}")
    
    # 重新打包为元组
    modified_tuple = (modified_dict['x'], modified_dict['y'], modified_dict['z'])
    print(f"修改后的元组: {modified_tuple}")
    
    return modified_tuple

def practical_example():
    """实际应用示例：模拟您的代码场景"""
    print("\n" + "="*50)
    print("实际应用示例")
    print("="*50)
    
    # 模拟从手眼标定得到的物理坐标
    physical_coord = (150, 80, 50)  # (x, y, z)
    print(f"原始物理坐标: {physical_coord}")
    
    # 解包
    x, y, z = physical_coord
    
    # 应用您的修改逻辑
    if x >= 100:
        x = x - 5
        y = y + 20
    elif 0 <= x < 100:
        y = y + 20
    
    # 重新打包
    modified_coord = (x, y, z)
    print(f"修改后的物理坐标: {modified_coord}")
    
    # 如果需要应用位置补偿
    from RobotCompensation import apply_position_compensation
    
    try:
        compensated_coord = apply_position_compensation(x, y, z)
        print(f"补偿后的坐标: {compensated_coord}")
    except ImportError:
        print("无法导入补偿函数，跳过补偿步骤")
    
    return modified_coord

def tuple_slicing_example():
    """方法5：使用切片（适用于部分修改）"""
    print("\n" + "="*50)
    print("方法5：使用切片")
    print("="*50)
    
    # 原始元组
    original_tuple = (100, 200, 300, 400, 500)
    print(f"原始元组: {original_tuple}")
    
    # 只修改前两个元素
    x, y = original_tuple[0], original_tuple[1]
    
    # 修改
    if x >= 100:
        x = x - 5
        y = y + 20
    
    # 使用切片重新组合
    modified_tuple = (x, y) + original_tuple[2:]
    print(f"修改后的元组: {modified_tuple}")
    
    return modified_tuple

def main():
    """主函数：演示所有方法"""
    print("元组修改方法演示")
    print("="*60)
    
    # 演示各种方法
    method1_result = modify_tuple_method1()
    method2_result = modify_tuple_method2()
    method3_result = modify_tuple_method3()
    method4_result = modify_tuple_method4()
    method5_result = tuple_slicing_example()
    
    # 实际应用示例
    practical_result = practical_example()
    
    print("\n" + "="*60)
    print("总结")
    print("="*60)
    print("推荐使用方法1（解包后重新打包）或方法2（列表转换）")
    print("对于复杂数据结构，推荐使用方法3（命名元组）")
    print("对于您的代码场景，建议使用方法1")

if __name__ == "__main__":
    main() 