#!/usr/bin/env python3
"""
餐厅赛项核心任务时序编排
功能：调用机械臂/爪部控制模块，按顺序执行餐具清理及收纳、餐余垃圾清扫、最终复位
支持通过常量控制阶段执行
异常处理：每次设置关节前进行越界检查，每个动作进行超时监控
"""
import os
import sys
# 将当前脚本所在目录添加到 sys.path 的最前面
sys.path.insert(0, os.path.dirname(__file__))

import rospy
import arm_control
# import gripper_control
import gripper_ctrl as gripper_control
import error_handle




# ========== 功能控制常量 ==========
RUN_STAGE1 = True   # 餐具清理及收纳（循环2次）
RUN_STAGE2 = True   # 餐余垃圾清扫（循环2次）
# 若需单独关闭某一阶段，只需修改对应常量为False，无需改动核心逻辑

# ========== 预设角度（示例值，可根据实际调整）==========
# 餐具清理步骤（每个餐具执行一次）
CLEAN_GRIPPER_CLOSE = 70.0      # 爪部夹紧角度
CLEAN_LIFT_ANGLES = [0, 45, 30, 0, 0]      # 抬升后关节角度
CLEAN_TILT_ANGLES = [0, 45, 30, 90, 0]     # 倾倒角度
CLEAN_RELEASE_ANGLES = [0, 0, 0, 0, 0]     # 收纳复位

# 餐余垃圾清扫步骤（根据题目示例）
WASTE_PICK_ANGLES = [30, 45, 30, 0, 0]     # 抓取角度
WASTE_BIN_ANGLES = [30, 45, 30, 120, 0]    # 垃圾桶上方角度
WASTE_RESET_ANGLES = [0, 0, 0, 0, 0]       # 复位角度

# ========== 工具函数 ==========
def reset_arm_and_gripper():
    """复位所有关节和爪部到初始状态"""
    print("执行复位: 所有关节归零，爪部松开")
    arm_control.set_arm_angles([0.0]*5)
    gripper_control.set_gripper_angle(0.0)
    rospy.sleep(2.0)  # 等待动作完成

# ========== 阶段1：餐具清理及收纳（循环2次）==========
def run_stage1():
    for i in range(2):
        print(f"\n=== 餐具清理 第{i+1}处 ===")
        # 步骤1：爪部夹紧
        if not error_handle.execute_with_timeout(
                gripper_control.set_gripper_angle, (CLEAN_GRIPPER_CLOSE,), 1.0):
            print("步骤1超时，继续下一动作")

        # 步骤2：机械臂抬升
        if not error_handle.check_joint_angles(CLEAN_LIFT_ANGLES):
            reset_arm_and_gripper()
            return False
        if not error_handle.execute_with_timeout(
                arm_control.set_arm_angles, (CLEAN_LIFT_ANGLES,), 2.0):
            print("步骤2超时，继续下一动作")

        # 步骤3：倾倒角度调整
        if not error_handle.check_joint_angles(CLEAN_TILT_ANGLES):
            reset_arm_and_gripper()
            return False
        if not error_handle.execute_with_timeout(
                arm_control.set_arm_angles, (CLEAN_TILT_ANGLES,), 2.0):
            print("步骤3超时，继续下一动作")

        # 步骤4：倒出销钉（模拟等待）
        print("倒出销钉...")
        rospy.sleep(1.0)

        # 步骤5：爪部松开
        if not error_handle.execute_with_timeout(
                gripper_control.set_gripper_angle, (0.0,), 1.0):
            print("步骤5超时，继续下一动作")

        # 步骤6：收纳复位
        if not error_handle.check_joint_angles(CLEAN_RELEASE_ANGLES):
            reset_arm_and_gripper()
            return False
        if not error_handle.execute_with_timeout(
                arm_control.set_arm_angles, (CLEAN_RELEASE_ANGLES,), 2.0):
            print("步骤6超时，继续下一动作")
    return True

# ========== 阶段2：餐余垃圾清扫（循环2次）==========
def run_stage2():
    for i in range(2):
        print(f"\n=== 餐余垃圾清扫 第{i+1}处 ===")
        # 步骤1：调整至抓取角度
        if not error_handle.check_joint_angles(WASTE_PICK_ANGLES):
            reset_arm_and_gripper()
            return False
        if not error_handle.execute_with_timeout(
                arm_control.set_arm_angles, (WASTE_PICK_ANGLES,), 2.0):
            print("步骤1超时，继续下一动作")

        # 步骤2：爪部闭合至70°
        if not error_handle.execute_with_timeout(
                gripper_control.set_gripper_angle, (70.0,), 1.0):
            print("步骤2超时，继续下一动作")

        # 步骤3：抬升并调整至垃圾桶角度
        if not error_handle.check_joint_angles(WASTE_BIN_ANGLES):
            reset_arm_and_gripper()
            return False
        if not error_handle.execute_with_timeout(
                arm_control.set_arm_angles, (WASTE_BIN_ANGLES,), 2.0):
            print("步骤3超时，继续下一动作")

        # 步骤4：爪部松开至0°
        if not error_handle.execute_with_timeout(
                gripper_control.set_gripper_angle, (0.0,), 0.5):
            print("步骤4超时，继续下一动作")

        # 步骤5：机械臂复位
        if not error_handle.check_joint_angles(WASTE_RESET_ANGLES):
            reset_arm_and_gripper()
            return False
        if not error_handle.execute_with_timeout(
                arm_control.set_arm_angles, (WASTE_RESET_ANGLES,), 2.0):
            print("步骤5超时，继续下一动作")
    return True

# ========== 主流程 ==========
def main():
    #tiaoshi
    print("gripper_control 模块路径:", gripper_control.__file__)
    print("模块中的属性:", dir(gripper_control))

    rospy.init_node('task_sequence_node', anonymous=True)
    print("===== 餐厅3D清洁任务开始 =====")

    # 阶段1
    if RUN_STAGE1:
        if not run_stage1():
            print("阶段1执行失败，任务终止")
            return
    else:
        print("跳过餐具清理阶段")

    # 阶段2
    if RUN_STAGE2:
        if not run_stage2():
            print("阶段2执行失败，任务终止")
            return
    else:
        print("跳过餐余垃圾清扫阶段")

    # 阶段3：任务完成复位
    print("\n===== 任务完成，执行最终复位 =====")
    reset_arm_and_gripper()
    print("餐厅3D清洁任务完成，所有动作执行结束")

if __name__ == "__main__":
    main()