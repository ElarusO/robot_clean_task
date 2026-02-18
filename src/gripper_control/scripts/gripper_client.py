#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from gripper_control.srv import GripperControl

def gripper_client():
    rospy.wait_for_service("/gripper_control")
    try:
        proxy = rospy.ServiceProxy("/gripper_control", GripperControl)
        rospy.loginfo("客户端已连接至 /gripper_control，请输入目标角度（0~90），输入 q 退出")

        while not rospy.is_shutdown():
            user_input = input(">>> 目标角度: ").strip()
            if user_input.lower() in ["q", "quit", "exit"]:
                break

            try:
                angle = float(user_input)
            except ValueError:
                print("输入错误：请输入数字或 q")
                continue

            # 发送请求
            resp = proxy(angle)
            # 打印完整响应
            print("\n=== 响应信息 ===")
            print(f"执行结果: {'成功' if resp.result else '失败'}")
            print(f"当前实际角度: {resp.current_angle:.2f}°")
            print(f"说明: {resp.info}\n")

    except rospy.ServiceException as e:
        rospy.logerr(f"服务调用失败: {e}")

if __name__ == "__main__":
    rospy.init_node("gripper_client")
    gripper_client()