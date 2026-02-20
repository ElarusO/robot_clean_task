#!/usr/bin/env python3
"""
卫浴赛项核心动作逻辑节点
功能：实现马桶刷抓取-清洁-放置的完整动作序列
复用项目1/2的机械臂/爪部控制逻辑，集成角度越界保护
支持单独调用任意子动作（通过修改常量或调用函数）
"""
import os
import sys
import rospy

# 确保当前目录在 sys.path 中，以便导入自定义模块
sys.path.insert(0, os.path.dirname(__file__))

import arm_control
import gripper_ctrl as gripper_control
import error_handle

# ========== 预设角度（自定义，符合0-270°范围）==========
# 初始/归位角度
HOME_ANGLES = [0.0, 0.0, 0.0, 0.0, 0.0]

# 抓取位置角度（机械臂到位）
GRASP_ANGLES = [30.0, 60.0, 30.0, 0.0, 0.0]   # joint1~5

# 爪部夹紧角度（贴合马桶刷手柄）
GRIPPER_CLOSE_ANGLE = 45.0   # 0~90°

# 抬升角度（将刷子提起）
LIFT_ANGLES = [30.0, 30.0, 60.0, 0.0, 0.0]

# 清洁角度（模拟马桶内部清洁，增加关节4旋转）
CLEAN_ANGLES = [30.0, 30.0, 60.0, 90.0, 0.0]

# 放置位置角度（此处复用抓取角度）
PLACE_ANGLES = GRASP_ANGLES.copy()

# ========== 工具函数 ==========
def reset_arm_and_gripper():
    """复位所有关节和爪部到初始状态"""
    print(">> 执行安全复位：所有关节归零，爪部松开")
    arm_control.set_arm_angles(HOME_ANGLES)
    gripper_control.set_gripper_angle(0.0)
    rospy.sleep(2.0)   # 等待动作完成

def check_and_execute(angles, action_name, duration):
    """
    检查角度合法性，并执行机械臂移动
    若角度非法，则触发复位并终止程序
    """
    if not error_handle.check_joint_angles(angles):
        reset_arm_and_gripper()
        rospy.signal_shutdown("角度越界，任务终止")
        sys.exit(1)
    print(f"  执行 {action_name}，耗时 {duration}s")
    arm_control.set_arm_angles(angles)
    rospy.sleep(duration)

# ========== 子动作函数（可单独调用）==========
def move_to_grasp():
    """子动作1：机械臂移动到抓取位置"""
    check_and_execute(GRASP_ANGLES, "机械臂到位（抓取位置）", 2.0)

def close_gripper():
    """子动作2：爪部夹紧"""
    print(f"  执行 爪部夹紧 (角度 {GRIPPER_CLOSE_ANGLE}°)，耗时 1s")
    if not error_handle.execute_with_timeout(
            gripper_control.set_gripper_angle, (GRIPPER_CLOSE_ANGLE,), 1.0):
        print("  爪部夹紧超时，但继续执行")
    rospy.sleep(1.0)   # 等待实际夹紧

def lift():
    """子动作3：抬升机械臂"""
    check_and_execute(LIFT_ANGLES, "抬升", 2.0)

def move_to_clean():
    """子动作4：移动到清洁角度"""
    check_and_execute(CLEAN_ANGLES, "移动到清洁角度", 2.0)

def hold_clean():
    """子动作5：保持清洁姿势5秒（模拟清洁）"""
    print("  执行 保持清洁姿势，耗时 5s")
    rospy.sleep(5.0)

def reset_to_grasp():
    """子动作6：复位到抓取位置"""
    check_and_execute(PLACE_ANGLES, "复位到抓取位置", 2.0)

def open_gripper():
    """子动作7：爪部松开（放置）"""
    print("  执行 爪部松开（放置），耗时 1s")
    if not error_handle.execute_with_timeout(
            gripper_control.set_gripper_angle, (0.0,), 1.0):
        print("  爪部松开超时，但继续执行")
    rospy.sleep(1.0)

def move_to_home():
    """子动作8：机械臂归位到初始角度"""
    check_and_execute(HOME_ANGLES, "机械臂归位", 2.0)

# ========== 完整动作序列 ==========
def run_sequence():
    """执行完整抓取-清洁-放置流程，总耗时 ≤20s"""
    print("\n========== 卫浴清洁任务开始 ==========")
    move_to_grasp()        # 2s
    close_gripper()        # 1s
    lift()                 # 2s
    move_to_clean()        # 2s
    hold_clean()           # 5s
    reset_to_grasp()       # 2s
    open_gripper()         # 1s
    move_to_home()         # 2s
    print("========== 卫浴清洁任务完成 ==========\n")

# ========== 主节点入口 ==========
def main():
    rospy.init_node('bathroom_arm_action', anonymous=True)
    
    # 可在此通过常量控制是否执行完整序列
    # 若只想单独测试某个子动作，可注释掉 run_sequence()，并直接调用对应函数
    run_sequence()

if __name__ == "__main__":
    main()