#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist, Pose, Point
from turtlesim.msg import Pose as TurtlePose
from turtlesim.srv import Spawn, Kill
import random
import numpy as np

class TurtleNavigatorWithObstacleAvoidance:
    def __init__(self):
        rospy.init_node('turtle_navigator_obstacle_avoidance', anonymous=True)
        
        # 导航参数
        self.target_points = [
            Point(2.0, 2.0, 0.0),
            Point(8.0, 2.0, 0.0),
            Point(8.0, 8.0, 0.0),
            Point(2.0, 8.0, 0.0)
        ]
        
        # 避障参数
        self.obstacle_threshold = 2.0  # 障碍物检测阈值（米）
        self.avoidance_force_gain = 2.0  # 避障力增益
        self.attractive_force_gain = 1.0  # 吸引力增益
        
        # 控制参数
        self.kp_angle = 4.0  # 角度比例系数
        self.kp_distance = 1.5  # 距离比例系数
        self.max_linear_speed = 2.0
        self.max_angular_speed = 2.0
        self.distance_tolerance = 0.1  # 距离容差
        self.angle_tolerance = 0.02  # 角度容差（rad）
        
        # 状态变量
        self.current_pose = None
        self.obstacle_pose = None
        self.current_target_index = 0
        self.lap_count = 0
        self.max_laps = 3
        self.is_avoiding = False
        self.avoidance_start_time = 0
        
        # 初始化障碍物
        self.obstacle_name = "obstacle_turtle"
        self.setup_obstacle()
        
        # 发布者和订阅者
        self.cmd_vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/turtle1/pose', TurtlePose, self.pose_callback)
        rospy.Subscriber('/'+self.obstacle_name+'/pose', TurtlePose, self.obstacle_pose_callback)
        
        # 定时器
        rospy.Timer(rospy.Duration(0.1), self.control_loop)
        
        # 随机移动障碍物的定时器
        rospy.Timer(rospy.Duration(3.0), self.move_obstacle_randomly)
        
        rospy.loginfo("Turtle navigator with obstacle avoidance initialized")
        rospy.loginfo("Target points: (2,2) -> (8,2) -> (8,8) -> (2,8)")
        rospy.loginfo("Running for %d laps", self.max_laps)
    
    def setup_obstacle(self):
        """创建障碍物海龟"""
        try:
            # 等待spawn服务
            rospy.wait_for_service('spawn')
            spawn_turtle = rospy.ServiceProxy('spawn', Spawn)
            
            # 在随机位置生成障碍物海龟
            x = random.uniform(3.0, 7.0)
            y = random.uniform(3.0, 7.0)
            theta = random.uniform(0, 2*math.pi)
            
            response = spawn_turtle(x, y, theta, self.obstacle_name)
            rospy.loginfo("Obstacle turtle spawned at (%.2f, %.2f)", x, y)
            
        except rospy.ServiceException as e:
            rospy.logerr("Failed to spawn obstacle turtle: %s", str(e))
    
    def move_obstacle_randomly(self, event):
        """随机移动障碍物"""
        if self.obstacle_pose is None:
            return
            
        try:
            # 随机生成新的目标位置
            target_x = random.uniform(2.0, 9.0)
            target_y = random.uniform(2.0, 9.0)
            
            # 计算朝向目标的角度
            dx = target_x - self.obstacle_pose.x
            dy = target_y - self.obstacle_pose.y
            target_angle = math.atan2(dy, dx)
            
            # 设置障碍物速度
            cmd_vel = Twist()
            distance = math.sqrt(dx**2 + dy**2)
            
            if distance > 0.5:
                cmd_vel.linear.x = min(0.5, distance * 0.5)
                angle_diff = self.normalize_angle(target_angle - self.obstacle_pose.theta)
                cmd_vel.angular.z = angle_diff * 2.0
            else:
                # 到达目标，随机转向
                cmd_vel.angular.z = random.uniform(-1.0, 1.0)
            
            obstacle_cmd_pub = rospy.Publisher('/'+self.obstacle_name+'/cmd_vel', Twist, queue_size=10)
            obstacle_cmd_pub.publish(cmd_vel)
            
        except Exception as e:
            rospy.logwarn("Error moving obstacle: %s", str(e))
    
    def pose_callback(self, data):
        """主海龟位姿回调"""
        self.current_pose = data
    
    def obstacle_pose_callback(self, data):
        """障碍物海龟位姿回调"""
        self.obstacle_pose = data
    
    def normalize_angle(self, angle):
        """将角度归一化到[-π, π]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def calculate_distance_to_obstacle(self):
        """计算到障碍物的距离"""
        if self.current_pose is None or self.obstacle_pose is None:
            return float('inf')
        
        dx = self.obstacle_pose.x - self.current_pose.x
        dy = self.obstacle_pose.y - self.current_pose.y
        return math.sqrt(dx**2 + dy**2)
    
    def artificial_potential_field(self, target_point):
        """人工势场法计算合速度"""
        if self.current_pose is None or self.obstacle_pose is None:
            return self.calculate_basic_control(target_point)
        
        # 计算吸引力（朝向目标）
        dx_att = target_point.x - self.current_pose.x
        dy_att = target_point.y - self.current_pose.y
        distance_to_target = math.sqrt(dx_att**2 + dy_att**2)
        
        # 归一化吸引力向量
        if distance_to_target > 0:
            attractive_force_x = (dx_att / distance_to_target) * self.attractive_force_gain
            attractive_force_y = (dy_att / distance_to_target) * self.attractive_force_gain
        else:
            attractive_force_x = 0
            attractive_force_y = 0
        
        # 计算斥力（远离障碍物）
        dx_rep = self.current_pose.x - self.obstacle_pose.x
        dy_rep = self.current_pose.y - self.obstacle_pose.y
        distance_to_obstacle = math.sqrt(dx_rep**2 + dy_rep**2)
        
        # 如果距离超过阈值，不产生斥力
        if distance_to_obstacle > self.obstacle_threshold:
            repulsive_force_x = 0
            repulsive_force_y = 0
            self.is_avoiding = False
        else:
            self.is_avoiding = True
            # 斥力大小与距离成反比
            if distance_to_obstacle > 0.1:  # 防止除以零
                repulsive_magnitude = min(2.0, self.avoidance_force_gain / (distance_to_obstacle**2))
                repulsive_force_x = (dx_rep / distance_to_obstacle) * repulsive_magnitude
                repulsive_force_y = (dy_rep / distance_to_obstacle) * repulsive_magnitude
            else:
                repulsive_force_x = 5.0 * dx_rep
                repulsive_force_y = 5.0 * dy_rep
        
        # 计算合力
        total_force_x = attractive_force_x + repulsive_force_x
        total_force_y = attractive_force_y + repulsive_force_y
        
        # 计算目标角度
        target_angle = math.atan2(total_force_y, total_force_x)
        
        return target_angle, distance_to_target
    
    def calculate_basic_control(self, target_point):
        """基础控制计算（无避障）"""
        if self.current_pose is None:
            return 0, float('inf')
        
        dx = target_point.x - self.current_pose.x
        dy = target_point.y - self.current_pose.y
        
        distance = math.sqrt(dx**2 + dy**2)
        target_angle = math.atan2(dy, dx)
        
        return target_angle, distance
    
    def turn_to_target(self, target_angle):
        """转向目标角度（P控制）"""
        if self.current_pose is None:
            return Twist()
        
        angle_error = self.normalize_angle(target_angle - self.current_pose.theta)
        
        cmd_vel = Twist()
        
        # 如果角度误差小，可以同时前进
        if abs(angle_error) < 0.1:
            cmd_vel.linear.x = min(self.max_linear_speed, 0.5)
        
        # P控制角速度
        cmd_vel.angular.z = max(-self.max_angular_speed, 
                               min(self.max_angular_speed, self.kp_angle * angle_error))
        
        return cmd_vel
    
    def move_to_target(self, target_angle, distance):
        """移动到目标点"""
        cmd_vel = Twist()
        
        if self.current_pose is None:
            return cmd_vel
        
        angle_error = self.normalize_angle(target_angle - self.current_pose.theta)
        
        # 角度误差大时，先转向
        if abs(angle_error) > self.angle_tolerance:
            cmd_vel.angular.z = max(-self.max_angular_speed, 
                                   min(self.max_angular_speed, self.kp_angle * angle_error))
        else:
            # 角度正确，前进
            linear_speed = min(self.max_linear_speed, self.kp_distance * distance)
            cmd_vel.linear.x = linear_speed
            
            # 微小角度修正
            cmd_vel.angular.z = self.kp_angle * angle_error
        
        return cmd_vel
    
    def control_loop(self, event):
        """主控制循环"""
        if self.current_pose is None:
            return
        
        # 检查是否完成所有圈数
        if self.lap_count >= self.max_laps and self.current_target_index == 0:
            # 停止运动
            self.cmd_vel_pub.publish(Twist())
            rospy.loginfo("Mission completed! %d laps finished.", self.max_laps)
            rospy.signal_shutdown("Mission completed")
            return
        
        # 获取当前目标点
        current_target = self.target_points[self.current_target_index]
        
        # 检查到障碍物的距离
        obstacle_distance = self.calculate_distance_to_obstacle()
        
        # 选择控制策略
        if obstacle_distance < self.obstacle_threshold:
            # 使用人工势场法（避障模式）
            target_angle, distance = self.artificial_potential_field(current_target)
            if self.is_avoiding:
                rospy.loginfo_throttle(1, "Avoiding obstacle! Distance: %.2f", obstacle_distance)
        else:
            # 正常导航模式
            target_angle, distance = self.calculate_basic_control(current_target)
            self.is_avoiding = False
        
        # 执行控制
        if distance > self.distance_tolerance:
            cmd_vel = self.move_to_target(target_angle, distance)
        else:
            # 到达目标点，转向下一个点
            cmd_vel = self.turn_to_target(target_angle)
            
            # 检查是否精确到达
            if distance < self.distance_tolerance:
                self.switch_to_next_target()
        
        # 发布速度命令
        self.cmd_vel_pub.publish(cmd_vel)
    
    def switch_to_next_target(self):
        """切换到下一个目标点"""
        self.current_target_index += 1
        
        # 检查是否完成一圈
        if self.current_target_index >= len(self.target_points):
            self.current_target_index = 0
            self.lap_count += 1
            rospy.loginfo("Lap %d completed!", self.lap_count)
        
        # 记录新目标点
        new_target = self.target_points[self.current_target_index]
        rospy.loginfo("Moving to target %d: (%.1f, %.1f)", 
                     self.current_target_index, new_target.x, new_target.y)
    
    def cleanup(self):
        """清理资源"""
        try:
            # 清除障碍物海龟
            rospy.wait_for_service('kill')
            kill_turtle = rospy.ServiceProxy('kill', Kill)
            kill_turtle(self.obstacle_name)
            rospy.loginfo("Obstacle turtle removed")
        except:
            pass

def main():
    try:
        navigator = TurtleNavigatorWithObstacleAvoidance()
        rospy.on_shutdown(navigator.cleanup)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("Error in main: %s", str(e))

if __name__ == '__main__':
    main()
