#!/usr/bin/env python3
"""
机械臂关节控制模块
功能：发布5轴机械臂关节角度到话题 /arm_joint_angles
依赖：ROS节点需提前初始化（由主程序完成）
"""
import rospy
from std_msgs.msg import Float64MultiArray

_pub = None

def _get_publisher():
    """获取或创建发布者（确保节点已初始化）"""
    global _pub
    if _pub is None:
        try:
            _pub = rospy.Publisher('/arm_joint_angles', Float64MultiArray, queue_size=10)
        except rospy.ROSInitException:
            raise RuntimeError("ROS节点未初始化，请先初始化节点")
    return _pub

def set_arm_angles(angles):
    """
    设置机械臂关节角度（仅发布指令，不等待）
    :param angles: list of 5 floats，每个关节角度 0~270°
    :return: None
    """
    pub = _get_publisher()
    msg = Float64MultiArray()
    msg.data = angles
    pub.publish(msg)
    rospy.logdebug(f"发布关节角度: {angles}")