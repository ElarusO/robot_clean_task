#!/usr/bin/env python3
"""
机械臂关节角度发布与订阅模块
赛事适配说明：适配5轴机械臂，每关节旋转范围0-270°
项目：ROS节点通信与5轴机械臂关节基础控制
作者：[你的名字]

本模块提供两个主要函数：
- start_publisher(node_name='arm_joint_publisher') 启动发布节点
- start_subscriber(node_name='arm_state_subscriber') 启动订阅节点
"""

import rospy
import sys
import select          # 非阻塞键盘输入
from sensor_msgs.msg import JointState
import numpy as np

# ---------- 发布节点类 ----------
class ArmJointPublisher:
    def __init__(self, node_name='arm_joint_publisher'):
        """
        初始化发布节点
        :param node_name: ROS节点名称，默认为arm_joint_publisher
        """
        rospy.init_node(node_name, anonymous=True)
        self.pub = rospy.Publisher('/arm_joint_angles', JointState, queue_size=10)
        self.joint_state = JointState()
        self.joint_state.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
        self.joint_state.position = [
            0.0,
            self.deg_to_rad(30),
            self.deg_to_rad(60),
            self.deg_to_rad(90),
            self.deg_to_rad(120)
        ]
        self.previous_angles = self.joint_state.position.copy()
        self.rate = rospy.Rate(1)   # 1Hz
        rospy.loginfo("机械臂关节发布节点已启动，按Ctrl+C退出")
        rospy.loginfo("关节角度限制：0-270度")

    def deg_to_rad(self, degrees):
        return degrees * 3.141592653589793 / 180.0

    def rad_to_deg(self, radians):
        return radians * 180.0 / 3.141592653589793

    def validate_angle(self, angle_deg):
        if 0 <= angle_deg <= 270:
            return True
        rospy.logwarn(f"角度 {angle_deg}° 超出范围！有效范围：0-270度")
        return False

    def update_joint_angles(self):
        rospy.loginfo("\n=== 手动角度修改模式 ===")
        rospy.loginfo("当前关节角度：")
        for i, name in enumerate(self.joint_state.name):
            angle_deg = self.rad_to_deg(self.joint_state.position[i])
            rospy.loginfo(f"{name}: {angle_deg:.1f}°")
        try:
            new_angles_deg = []
            for i, name in enumerate(self.joint_state.name):
                user_input = input(f"输入 {name} 的新角度(0-270°) [当前: {self.rad_to_deg(self.joint_state.position[i]):.1f}°]: ")
                if user_input.strip():
                    new_angle = float(user_input)
                    if self.validate_angle(new_angle):
                        new_angles_deg.append(new_angle)
                    else:
                        rospy.logwarn(f"角度无效，保持原值")
                        new_angles_deg.append(self.rad_to_deg(self.joint_state.position[i]))
                else:
                    new_angles_deg.append(self.rad_to_deg(self.joint_state.position[i]))
            for i in range(len(self.joint_state.position)):
                self.previous_angles[i] = self.joint_state.position[i]
                self.joint_state.position[i] = self.deg_to_rad(new_angles_deg[i])
            rospy.loginfo("关节角度已更新！")
        except ValueError:
            rospy.logerr("输入错误！请输入有效的数字")
        except KeyboardInterrupt:
            rospy.loginfo("角度修改取消")

    def run(self):
        loop_count = 0
        while not rospy.is_shutdown():
            self.joint_state.header.stamp = rospy.Time.now()
            self.pub.publish(self.joint_state)
            loop_count += 1
            if loop_count % 10 == 0:
                try:
                    response = input("\n是否修改关节角度？(y/n, 默认n): ")
                    if response.lower() == 'y':
                        self.update_joint_angles()
                except:
                    pass
            self.rate.sleep()

# ---------- 订阅节点类 ----------
class ArmStateSubscriber:
    def __init__(self, node_name='arm_state_subscriber'):
        """
        初始化订阅节点
        :param node_name: ROS节点名称，默认为arm_state_subscriber
        """
        rospy.init_node(node_name, anonymous=True)
        self.previous_positions = None
        rospy.Subscriber('/arm_joint_angles', JointState, self.joint_state_callback)
        rospy.loginfo("机械臂状态订阅节点已启动，等待数据...")
        rospy.loginfo("监控话题：/arm_joint_angles")
        self.msg_count = 0
        self.last_seq = -1

    def rad_to_deg(self, radians):
        return radians * 180.0 / 3.141592653589793

    def joint_state_callback(self, msg):
        self.msg_count += 1
        if hasattr(msg.header, 'seq') and self.last_seq != -1:
            if msg.header.seq != self.last_seq + 1:
                rospy.logwarn(f"可能丢包！当前seq={msg.header.seq}, 期望={self.last_seq + 1}")
        if hasattr(msg.header, 'seq'):
            self.last_seq = msg.header.seq

        print("\033[2J\033[H")  # 清屏
        print("=" * 60)
        print(f"机械臂关节状态监控 (消息#{self.msg_count})")
        print(f"时间戳: {msg.header.stamp.to_sec():.3f}s")
        print("=" * 60)
        print(f"{'关节名称':<10} {'当前角度(°)':<15} {'角度差值(°)':<15} {'状态':<10}")
        print("-" * 60)

        if len(msg.position) != 5:
            rospy.logerr(f"错误：期望5个关节，收到{len(msg.position)}个")
            return

        if self.previous_positions is None:
            self.previous_positions = list(msg.position)

        for i in range(len(msg.name)):
            joint_name = msg.name[i] if i < len(msg.name) else f"joint{i+1}"
            current_rad = msg.position[i]
            current_deg = self.rad_to_deg(current_rad)
            prev_rad = self.previous_positions[i]
            prev_deg = self.rad_to_deg(prev_rad)
            diff_deg = current_deg - prev_deg

            if abs(diff_deg) < 0.1:
                status = "静止"
            elif diff_deg > 0:
                status = "正向运动"
            else:
                status = "反向运动"

            print(f"{joint_name:<10} {current_deg:>7.2f}°       {diff_deg:>+7.2f}°       {status:<10}")

        print("=" * 60)
        print("提示：角度差值=当前角度-上一帧角度")
        print("=" * 60)

        self.previous_positions = list(msg.position)

    def run(self):
        rospy.spin()

# ---------- 对外接口函数 ----------
def start_publisher(node_name='arm_joint_publisher'):
    """
    启动关节角度发布节点（阻塞运行，直到节点关闭）
    :param node_name: 可选，指定ROS节点名称
    """
    publisher = ArmJointPublisher(node_name)
    try:
        publisher.run()
    except rospy.ROSInterruptException:
        pass

def start_subscriber(node_name='arm_state_subscriber'):
    """
    启动关节状态订阅节点（阻塞运行，直到节点关闭）
    :param node_name: 可选，指定ROS节点名称
    """
    subscriber = ArmStateSubscriber(node_name)
    try:
        subscriber.run()
    except rospy.ROSInterruptException:
        pass