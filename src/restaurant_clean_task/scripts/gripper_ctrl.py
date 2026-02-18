#!/usr/bin/env python3
"""
柔性爪部控制模块
功能：通过gripper_tools发送角度指令到爪部服务端
依赖：gripper_tools库，服务端需提前运行
"""
import gripper_tools

def set_gripper_angle(angle):
    """
    设置爪部角度
    :param angle: float 0~90°
    :return: bool 是否成功发送指令（服务响应结果）
    """
    resp = gripper_tools.send_command(angle, timeout=2.0)
    if resp is not None:
        return resp.result
    else:
        print("错误: 无法连接到gripper服务，请确保服务端已启动")
        return False