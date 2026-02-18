#!/usr/bin/env python3
"""
异常处理模块
功能：关节越界检查、动作超时监控
"""
import time

def check_joint_angles(angles):
    """
    检查关节角度是否在合法范围 [0, 270] 内
    :param angles: list of floats
    :return: bool True=合法 False=越界
    """
    for i, ang in enumerate(angles):
        if ang < 0 or ang > 270:
            print(f"错误: joint{i+1}角度{ang}超出0-270°，动作停止。")
            return False
    return True

def execute_with_timeout(action_func, args, expected_duration, timeout=3.0):
    """
    执行一个动作并监控超时
    :param action_func: 可调用对象（发送指令的函数）
    :param args: 动作函数的参数元组
    :param expected_duration: 预期动作完成所需时间（秒）
    :param timeout: 超时阈值（秒）
    :return: bool True=在超时内完成，False=超时
    """
    # 执行动作（发送指令）
    action_func(*args)
    
    # 等待预期时间，同时检测超时
    start = time.time()
    while time.time() - start < expected_duration:
        if time.time() - start >= timeout:
            print(f"超时: 动作执行超过{timeout}秒，跳过该动作。")
            return False
        time.sleep(0.1)  # 短间隔检查
    return True