#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ROS1 Noetic 海龟绘制N边形 - 动态速度+参数化
import rospy
import math
from geometry_msgs.msg import Twist

class TurtlePolygon:
    def __init__(self):
        # 1. 初始化ROS节点
        rospy.init_node('turtle_polygon_node', anonymous=True)
        # 2. 创建话题发布者，发布控制指令到/cmd_vel，队列大小10
        self.cmd_vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        # 3. 从launch/参数服务器读取参数（设置默认值，防止参数未传）
        self.N = rospy.get_param('~polygon_edges', 5)  # 多边形边数，默认5
        self.v_base = rospy.get_param('~base_vel', 0.2)# 基础线速度，默认0.2m/s
        # 4. 初始化控制指令消息
        self.twist = Twist()
        # 5. 设置循环频率（10Hz，减少开环控制的时间误差）
        self.rate = rospy.Rate(10)

    # 函数：直线运动 - 动态速度，指定距离
    def move_straight(self, distance, speed):
        # 重置速度指令
        self.twist.linear.x = speed
        self.twist.angular.z = 0.0
        # 计算需要运动的时间（距离/速度）
        start_time = rospy.Time.now()
        duration = rospy.Duration(distance / speed)
        # 持续发布速度指令，直到达到指定时间
        while rospy.Time.now() - start_time < duration and not rospy.is_shutdown():
            self.cmd_vel_pub.publish(self.twist)
            self.rate.sleep()
        # 运动结束后停止海龟（防止惯性偏移）
        self.twist.linear.x = 0.0
        self.cmd_vel_pub.publish(self.twist)
        rospy.sleep(0.1)  # 短暂停顿，避免转向与直线运动衔接误差

    # 函数：原地转向 - 固定角速度1rad/s，指定角度（弧度）
    def turn(self, angle):
        # 重置速度指令，固定角速度1rad/s（方便计算转向时间）
        self.twist.linear.x = 0.0
        self.twist.angular.z = 1.0  # 正方向为逆时针
        # 计算转向时间（角度/角速度）
        start_time = rospy.Time.now()
        duration = rospy.Duration(angle / 1.0)
        # 持续发布转向指令
        while rospy.Time.now() - start_time < duration and not rospy.is_shutdown():
            self.cmd_vel_pub.publish(self.twist)
            self.rate.sleep()
        # 转向结束后停止海龟
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)
        rospy.sleep(0.1)

    # 核心函数：绘制N边形
    def draw_polygon(self):
        rospy.loginfo("开始绘制%d边形，基础线速度：%.2fm/s", self.N, self.v_base)
        # 计算N边形外角（海龟转向角度，弧度）：360°/N = 2π/N
        turn_angle = 2 * math.pi / self.N
        # 设定多边形边长（固定0.5m，可改为参数传入）
        edge_length = 0.5

        # 循环N次：走边 + 转向（动态速度：第i条边速度=v_base×i）
        for i in range(1, self.N+1):
            current_speed = self.v_base * i  # 动态速度
            rospy.loginfo("绘制第%d条边，当前速度：%.2fm/s", i, current_speed)
            # 走直线：边长0.5m，当前动态速度
            self.move_straight(edge_length, current_speed)
            # 原地转向：N边形外角
            self.turn(turn_angle)

        rospy.loginfo("%d边形绘制完成！", self.N)

if __name__ == '__main__':
    try:
        # 创建实例并执行绘制
        turtle = TurtlePolygon()
        turtle.draw_polygon()
    except rospy.ROSInterruptException:
        # 捕获节点中断异常，优雅退出
        pass
    except ZeroDivisionError:
        rospy.logerr("边数N不能为0！")
