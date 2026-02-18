#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ROS1 Noetic 海龟闭环Leader+实时坐标误差+回位验证
import rospy
import math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class TurtleClosedLoop:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('turtle_leader_node', anonymous=True)
        # 发布/订阅话题
        self.cmd_vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.pose_sub = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        # 初始化位姿/速度指令
        self.current_pose = Pose()
        self.twist = Twist()
        # 闭环比例控制参数
        self.Kp_linear = 0.5    
        self.Kp_angular = 2.0   
        # 多点巡航目标点
        self.targets = [(2,2), (2,8), (8,8), (8,2), (2,2)]
        self.current_target_idx = 0
        # 核心参数：到达阈值+固定起点（用于最终回位验证）
        self.arrive_threshold = 0.05
        self.start_x, self.start_y = self.targets[0]  # 起点(2,2)
        # 循环频率
        self.rate = rospy.Rate(10)
        rospy.loginfo("Leader初始化完成，起点：(%.2f, %.2f)", self.start_x, self.start_y)

    def pose_callback(self, msg):
        self.current_pose = msg

    def calculate_error(self, target_x, target_y):
        """计算误差：返回距离/角度误差，同时计算x/y坐标误差（供打印）"""
        dx = target_x - self.current_pose.x  # x方向坐标误差
        dy = target_y - self.current_pose.y  # y方向坐标误差
        distance_error = math.hypot(dx, dy)  # 直线距离
        # 角度误差归一化
        target_angle = math.atan2(dy, dx)
        angle_error = target_angle - self.current_pose.theta
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
        return distance_error, angle_error, dx, dy

    def print_real_time_error(self, target_x, target_y, dx, dy, distance_error):
        rospy.loginfo(
            "【实时误差】目标点(%.2f,%.2f) | X误差：%.4fm | Y误差：%.4fm | 距离误差：%.4fm",
            target_x, target_y, dx, dy, distance_error
        )

    def p_control(self, distance, angle_error):
        self.twist.linear.x = self.Kp_linear * distance if distance > self.arrive_threshold else 0.0
        self.twist.angular.z = self.Kp_angular * angle_error
        self.cmd_vel_pub.publish(self.twist)

    def check_back_to_start(self):
        # 计算当前位姿与起点的最终误差
        final_dx = self.start_x - self.current_pose.x
        final_dy = self.start_y - self.current_pose.y
        final_distance_error = math.hypot(final_dx, final_dy)
        # 醒目打印起点回位验证日志
        rospy.loginfo("="*50)
        rospy.loginfo("起点回位验证结果】")
        rospy.loginfo("起点坐标：(%.2f, %.2f)", self.start_x, self.start_y)
        rospy.loginfo("当前最终坐标：(%.4f, %.4f)", self.current_pose.x, self.current_pose.y)
        rospy.loginfo("最终X方向误差：%.4f m", final_dx)
        rospy.loginfo("最终Y方向误差：%.4f m", final_dy)
        rospy.loginfo("最终直线距离误差：%.4f m（到达阈值：%.2f m）", final_distance_error, self.arrive_threshold)
        # 阈值判断，直接标注是否成功回到起点
        if final_distance_error < self.arrive_threshold:
            rospy.loginfo("验证通过：海龟成功回到起点")
        else:
            rospy.loginfo("验证失败：海龟未回到起点")
        rospy.loginfo("="*50)

    def multi_point_navigation(self):
        rospy.loginfo("Leader开始巡航任务...")
        for circle in range(3):
            self.current_target_idx = 0
            rospy.loginfo("Leader开始第%d圈巡航", circle+1)
            while self.current_target_idx < len(self.targets) and not rospy.is_shutdown():
                target_x, target_y = self.targets[self.current_target_idx]
                rospy.loginfo("\n当前目标点: (%.2f, %.2f)", target_x, target_y)

                while not rospy.is_shutdown():
                    # 获取dx/dy坐标误差
                    distance_error, angle_error, dx, dy = self.calculate_error(target_x, target_y)
                    self.print_real_time_error(target_x, target_y, dx, dy, distance_error)

                    # 角度/距离控制
                    if abs(angle_error) > 0.02:
                        self.p_control(0, angle_error)
                    else:
                        self.p_control(distance_error, angle_error)

                    # 到达目标点判断
                    if distance_error < self.arrive_threshold:
                        rospy.loginfo("到达目标点: (%.2f, %.2f)\n", target_x, target_y)
                        self.current_target_idx += 1
                        break
                    self.rate.sleep()

        # 停止逻辑
        rospy.loginfo("巡航任务全部完成！")
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)
        # 验证是否回到起点并打印最终误差
        self.check_back_to_start()

if __name__ == '__main__':
    try:
        leader = TurtleClosedLoop()
        rospy.sleep(1.0)  # 等待位姿初始化
        leader.multi_point_navigation()
        # 保活节点，持续发布位姿供跟随者
        rospy.loginfo("节点保活中，持续发布位姿...")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logwarn("节点被手动中断！")
    except Exception as e:
        rospy.logerr("运行异常: %s", str(e))
