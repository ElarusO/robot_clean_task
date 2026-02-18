#!/usr/bin/env python3
"""
机械臂关节角度发布节点
赛事适配说明：适配5轴机械臂，每关节旋转范围0-270°
项目：ROS节点通信与5轴机械臂关节基础控制
作者：[你的名字]
"""

import rospy
import sys
import select          # 非阻塞键盘输入
from sensor_msgs.msg import JointState

class ArmJointPublisher:
    def __init__(self):
        """
        初始化发布节点
        赛事参数说明：关节名称按赛事要求固定为joint1-joint5
        """
        # 初始化ROS节点
        rospy.init_node('arm_joint_publisher', anonymous=True)
        
        # 创建话题发布者
        self.pub = rospy.Publisher('/arm_joint_angles', JointState, queue_size=10)
        
        # 初始化关节状态
        self.joint_state = JointState()
        
        # 设置关节名称（赛事固定参数）
        self.joint_state.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
        
        # 设置初始角度值（单位：弧度）
        # 赛事初始角度：0°, 30°, 60°, 90°, 120°
        self.joint_state.position = [
            0.0,                      # joint1: 0度
            self.deg_to_rad(30),     # joint2: 30度
            self.deg_to_rad(60),     # joint3: 60度
            self.deg_to_rad(90),     # joint4: 90度
            self.deg_to_rad(120)     # joint5: 120度
        ]
        
        # 存储上一帧角度，用于手动修改时的比较
        self.previous_angles = self.joint_state.position.copy()
        
        # 设置发布频率：1Hz（赛事要求）
        self.rate = rospy.Rate(1)
        
        rospy.loginfo("机械臂关节发布节点已启动，按Ctrl+C退出")
        rospy.loginfo("关节角度限制：0-270度")
        
    def deg_to_rad(self, degrees):
        """角度转弧度"""
        return degrees * 3.141592653589793 / 180.0
    
    def rad_to_deg(self, radians):
        """弧度转角度"""
        return radians * 180.0 / 3.141592653589793
    
    def validate_angle(self, angle_deg):
        """
        验证角度是否在有效范围内
        赛事要求：每个关节旋转角度≥270°，这里限制为0-270度
        """
        if 0 <= angle_deg <= 270:
            return True
        else:
            rospy.logwarn(f"角度 {angle_deg}° 超出范围！有效范围：0-270度")
            return False
    
    def update_joint_angles(self):
        """
        手动更新关节角度
        提供终端输入方式，支持0-270°范围校验
        """
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
            
            # 更新关节角度
            for i in range(len(self.joint_state.position)):
                self.previous_angles[i] = self.joint_state.position[i]
                self.joint_state.position[i] = self.deg_to_rad(new_angles_deg[i])
            
            rospy.loginfo("关节角度已更新！")
            
        except ValueError:
            rospy.logerr("输入错误！请输入有效的数字")
        except KeyboardInterrupt:
            rospy.loginfo("角度修改取消")
    
    def publish_joint_states(self):
        loop_count = 0
        while not rospy.is_shutdown():
            self.joint_state.header.stamp = rospy.Time.now()
            self.pub.publish(self.joint_state)
        
        # 每10次循环询问一次（发布频率1Hz，即10秒一次）
            loop_count += 1
            if loop_count % 10 == 0:
                try:
                    response = input("\n是否修改关节角度？(y/n, 默认n): ")
                    if response.lower() == 'y':
                        self.update_joint_angles()
                except:
                    pass
        
        self.rate.sleep()

if __name__ == '__main__':
    try:
        publisher = ArmJointPublisher()
        publisher.publish_joint_states()
    except rospy.ROSInterruptException:
        pass