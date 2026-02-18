#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from gripper_control.srv import GripperControl, GripperControlResponse

class GripperController:
    """机械爪控制逻辑封装：角度校验、防抖、状态保持"""
    def __init__(self, initial_angle=0.0):
        self.current_angle = initial_angle  # 当前实际角度，初始为0°（闭合）
        self.min_angle = 0.0
        self.max_angle = 90.0
        self.dead_zone = 5.0               # 防抖阈值（差值＜5°拒绝）

    def handle_request(self, req):
        """服务回调函数：处理角度请求，返回响应"""
        target = req.gripper_angle
        result = False
        info = ""

        # 1. 角度范围校验
        if target < self.min_angle or target > self.max_angle:
            info = f"角度非法：{target}° 不在 [{self.min_angle}°~{self.max_angle}°] 范围内"
            return GripperControlResponse(result, self.current_angle, info)

        # 2. 防抖校验：与当前实际角度比较
        if abs(target - self.current_angle) < self.dead_zone:
            info = f"角度差值过小：{abs(target - self.current_angle):.2f}° < {self.dead_zone}°，无需动作"
            return GripperControlResponse(result, self.current_angle, info)

        # 3. 合法执行
        self.current_angle = target
        result = True
        info = f"执行成功，机械爪已转动至 {self.current_angle}°"
        return GripperControlResponse(result, self.current_angle, info)


if __name__ == "__main__":
    rospy.init_node("gripper_server")
    controller = GripperController()

    # 创建服务 /gripper_control，类型为GripperControl
    srv = rospy.Service("/gripper_control", GripperControl, controller.handle_request)
    rospy.loginfo("机械爪控制服务已启动，等待请求...")
    rospy.spin()