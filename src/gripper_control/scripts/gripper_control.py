#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from gripper_control.srv import GripperControl, GripperControlResponse

# ==================== 服务器内部逻辑（与原始代码一致） ====================
class GripperController:
    """机械爪控制逻辑封装：角度校验、防抖、状态保持"""
    def __init__(self, initial_angle=0.0):
        self.current_angle = initial_angle
        self.min_angle = 0.0
        self.max_angle = 90.0
        self.dead_zone = 5.0

    def handle_request(self, req):
        target = req.gripper_angle
        result = False
        info = ""

        if target < self.min_angle or target > self.max_angle:
            info = f"角度非法：{target}° 不在 [{self.min_angle}°~{self.max_angle}°] 范围内"
            return GripperControlResponse(result, self.current_angle, info)

        if abs(target - self.current_angle) < self.dead_zone:
            info = f"角度差值过小：{abs(target - self.current_angle):.2f}° < {self.dead_zone}°，无需动作"
            return GripperControlResponse(result, self.current_angle, info)

        self.current_angle = target
        result = True
        info = f"执行成功，机械爪已转动至 {self.current_angle}°"
        return GripperControlResponse(result, self.current_angle, info)


# ==================== 对外提供的函数接口 ====================

def start_server(node_name="gripper_server"):
    """
    启动机械爪控制服务器（阻塞）。
    
    参数:
        node_name: ROS节点名称（可选，默认为 "gripper_server"）
    
    说明:
        该函数会初始化节点，创建服务 /gripper_control，并进入 spin 循环。
        通常应在独立的 Python 脚本中调用，或使用多线程/多进程运行。
    """
    rospy.init_node(node_name, anonymous=True)
    controller = GripperController()
    service = rospy.Service("/gripper_control", GripperControl, controller.handle_request)
    rospy.loginfo("机械爪控制服务已启动，等待请求...")
    rospy.spin()


def send_command(angle, timeout=None):
    """
    向机械爪服务器发送角度命令，并返回响应对象。
    
    参数:
        angle: 目标角度（float），应在 0~90 范围内
        timeout: 等待服务出现的超时时间（秒），None 表示无限等待
    
    返回:
        若调用成功，返回 GripperControlResponse 对象，包含字段：
            result (bool)      - 执行结果（成功/失败）
            current_angle (float) - 命令执行后的实际角度
            info (str)         - 描述信息
        若服务调用失败（如超时、服务异常），返回 None
    
    注意:
        该函数会自动初始化一个客户端节点（如果尚未初始化），
        因此可以直接在任意 Python 脚本中使用，无需提前运行 rospy.init_node。
    """
    # 如果 ROS 节点尚未初始化，则初始化一个匿名节点
    if not rospy.core.is_initialized():
        rospy.init_node("gripper_client", anonymous=True)

    try:
        rospy.wait_for_service("/gripper_control", timeout=timeout)
        proxy = rospy.ServiceProxy("/gripper_control", GripperControl)
        response = proxy(angle)
        return response
    except rospy.ServiceException as e:
        rospy.logerr(f"服务调用失败: {e}")
        return None