#!/usr/bin/env python3
"""
机械臂关节角度发布节点
核心特性：
1. 1Hz持续发布5个关节完整信息
2. 初始角度：0°、30°、60°、90°、120°
3. 独立线程处理用户输入，不阻塞发布流程
"""

import rospy
import threading
import sys
from sensor_msgs.msg import JointState

class ArmJointPublisher:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('arm_joint_publisher', anonymous=True)
        
        # 创建话题发布者
        self.pub = rospy.Publisher('/arm_joint_angles', JointState, queue_size=50)
        
        # 初始化关节状态消息
        self.joint_state = JointState()
        self.joint_state.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']  # 固定关节名称
        
        # 初始角度（转弧度）
        self.joint_state.position = [
            self.deg_to_rad(0),
            self.deg_to_rad(30),
            self.deg_to_rad(60),
            self.deg_to_rad(90),
            self.deg_to_rad(120)
        ]
        
        # 线程锁(防止读写冲突）
        self.angle_lock = threading.Lock()
        
        # 发布频率
        self.rate = rospy.Rate(1)
        
        # 标记节点是否运行
        self.is_running = True
        
        rospy.loginfo("=== 机械臂关节发布节点启动 ===")
        rospy.loginfo(f"发布频率：1Hz | 初始角度：0°, 30°, 60°, 90°, 120°")
        rospy.loginfo("输入 'update' 可修改关节角度，输入 'exit' 退出节点")
        rospy.loginfo("话题名称：/arm_joint_angles")

    def deg_to_rad(self, degrees):
        """角度转弧度"""
        return degrees * 3.1415926535 / 180.0

    def rad_to_deg(self, radians):
        """弧度转角度"""
        return radians * 180.0 / 3.1415926535

    def validate_angle(self, angle_deg):
        """验证角度是否在0-270°范围内"""
        if 0 <= angle_deg <= 270:
            return True
        rospy.logwarn(f"角度 {angle_deg}° 无效！有效范围：0-270°")
        return False

    def update_angles_interactive(self):
        """独立线程：处理用户输入修改角度"""
        while self.is_running and not rospy.is_shutdown():
            try:
                # 等待用户输入（这里的阻塞仅影响该线程，不影响发布主线程）
                user_input = input("\n请输入指令（update/exit）：").strip().lower()
                
                if user_input == 'exit':
                    self.is_running = False
                    rospy.loginfo("收到退出指令，节点即将停止")
                    break
                
                elif user_input == 'update':
                    rospy.loginfo("\n--- 开始修改关节角度 ---")
                    new_angles_rad = []
                    
                    # 逐个关节输入新角度
                    for i, joint_name in enumerate(self.joint_state.name):
                        current_deg = self.rad_to_deg(self.joint_state.position[i])
                        while True:
                            try:
                                new_deg = float(input(f"{joint_name}（当前{current_deg:.1f}°）："))
                                if self.validate_angle(new_deg):
                                    new_angles_rad.append(self.deg_to_rad(new_deg))
                                    break
                            except ValueError:
                                rospy.logwarn("输入错误！请输入数字（如：45）")
                    
                    # 加锁更新角度（防止和发布线程冲突）
                    with self.angle_lock:
                        self.joint_state.position = new_angles_rad
                    rospy.loginfo("角度修改完成！已更新发布内容")
                
                else:
                    rospy.logwarn("无效指令！仅支持：update（修改角度）、exit（退出）")
            
            except EOFError:
                # 处理终端输入异常（如Ctrl+D）
                continue
            except KeyboardInterrupt:
                self.is_running = False
                break

    def publish_loop(self):
        """主线程1Hz持续发布关节状态"""
        # 启动用户输入处理线程
        input_thread = threading.Thread(target=self.update_angles_interactive)
        input_thread.daemon = True  # 守护线程：主程序退出时自动结束
        input_thread.start()
        
        # 核心发布循环
        while self.is_running and not rospy.is_shutdown():
            try:
                # 加锁读取角度（防止读写冲突）
                with self.angle_lock:
                    self.joint_state.header.stamp = rospy.Time.now()  # 更新时间戳
                    self.pub.publish(self.joint_state)  # 发布消息
                
                # 打印当前发布状态（可选，方便调试）
                current_degs = [f"{self.rad_to_deg(rad):.1f}°" for rad in self.joint_state.position]
                rospy.logdebug(f"发布关节角度：{current_degs}")  # debug级别，不刷屏
                
                # 严格1Hz睡眠（核心：保证发布频率）
                self.rate.sleep()
            
            except rospy.ROSInterruptException:
                break
            except Exception as e:
                rospy.logerr(f"发布异常：{e}")
                continue
        
        # 节点退出清理
        rospy.loginfo("发布节点已停止")

if __name__ == '__main__':
    try:
        publisher = ArmJointPublisher()
        publisher.publish_loop()
    except rospy.ROSInterruptException:
        pass