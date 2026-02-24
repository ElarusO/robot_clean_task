#!/usr/bin/env python3
"""
机械臂关节状态订阅节点
赛事适配说明：实时监控5轴机械臂关节状态，计算角度差值
"""

import rospy
from sensor_msgs.msg import JointState
import numpy as np

class ArmStateSubscriber:
    def __init__(self):
        """初始化订阅节点"""
        # 初始化ROS节点
        rospy.init_node('arm_state_subscriber', anonymous=True)
        
        # 存储上一帧关节角度
        self.previous_positions = None
        
        # 创建订阅者
        rospy.Subscriber('/arm_joint_angles', JointState, self.joint_state_callback)
        
        rospy.loginfo("机械臂状态订阅节点已启动，等待数据...")
        rospy.loginfo("监控话题：/arm_joint_angles")
        
        # 消息计数器，用于检测丢包
        self.msg_count = 0
        self.last_seq = -1
        
    def rad_to_deg(self, radians):
        """弧度转角度"""
        return radians * 180.0 / 3.141592653589793
    
    def joint_state_callback(self, msg):
        """
        关节状态回调函数

        """
        self.msg_count += 1
        
        # 检查消息顺序（丢包检测）
        if hasattr(msg.header, 'seq') and self.last_seq != -1:
            if msg.header.seq != self.last_seq + 1:
                rospy.logwarn(f"可能丢包！当前seq={msg.header.seq}, 期望={self.last_seq + 1}")
        if hasattr(msg.header, 'seq'):
            self.last_seq = msg.header.seq
        
        # 清屏
        print("\033[2J\033[H")
        
        print("=" * 60)
        print(f"机械臂关节状态监控 (消息#{self.msg_count})")
        print(f"时间戳: {msg.header.stamp.to_sec():.3f}s")
        print("=" * 60)
        print(f"{'关节名称':<10} {'当前角度(°)':<15} {'角度差值(°)':<15} {'状态':<10}")
        print("-" * 60)
        
        # 检查数据完整性
        if len(msg.position) != 5:
            rospy.logerr(f"错误：期望5个关节，收到{len(msg.position)}个")
            return
        
        # 如果是第一帧数据，初始化previous_positions
        if self.previous_positions is None:
            self.previous_positions = list(msg.position)
        
        # 计算并打印每个关节的信息
        for i in range(len(msg.name)):
            # 获取关节名称
            joint_name = msg.name[i] if i < len(msg.name) else f"joint{i+1}"
            
            # 当前角度（弧度转角度）
            current_rad = msg.position[i]
            current_deg = self.rad_to_deg(current_rad)
            
            # 计算角度差值
            prev_rad = self.previous_positions[i]
            prev_deg = self.rad_to_deg(prev_rad)
            diff_deg = current_deg - prev_deg
            
            # 判断状态
            if abs(diff_deg) < 0.1:  # 变化小于0.1度视为静止
                status = "静止"
            elif diff_deg > 0:
                status = "正向运动"
            else:
                status = "反向运动"
            
            # 格式化输出（赛事要求格式）
            print(f"{joint_name:<10} {current_deg:>7.2f}°       {diff_deg:>+7.2f}°       {status:<10}")
        
        print("=" * 60)
        print("提示：角度差值=当前角度-上一帧角度")
        print("=" * 60)
        
        # 更新上一帧角度
        self.previous_positions = list(msg.position)
        
    def run(self):
        """运行节点"""
        rospy.spin()

if __name__ == '__main__':
    try:
        subscriber = ArmStateSubscriber()
        subscriber.run()
    except rospy.ROSInterruptException:
        pass
