#!/usr/bin/env python3
"""
机械臂关节控制整合模块
赛事适配说明：适配5轴机械臂，每关节旋转范围0-270°
项目：ROS节点通信与5轴机械臂关节基础控制
功能包含：
1. 关节角度发布节点（支持手动修改角度）
2. 关节状态订阅节点（监控角度变化）
3. 简易角度发布接口（Float64MultiArray格式）
"""

import rospy
import sys
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np

# -------------------------- 全局发布者（简易接口用） --------------------------
_pub = None

def _get_publisher():
    """获取或创建Float64MultiArray类型的发布者（确保节点已初始化）"""
    global _pub
    if _pub is None:
        try:
            _pub = rospy.Publisher('/arm_joint_angles', Float64MultiArray, queue_size=10)
        except rospy.ROSInitException:
            raise RuntimeError("ROS节点未初始化，请先初始化节点")
    return _pub

def set_arm_angles(angles):
    """
    简易接口：设置机械臂关节角度（仅发布指令，不等待）
    :param angles: list of 5 floats，每个关节角度 0~270°
    :return: None
    """
    # 角度范围校验
    for i, angle in enumerate(angles):
        if not (0 <= angle <= 270):
            rospy.logwarn(f"关节{i+1}角度{angle}°超出范围(0-270°)，已自动修正为合法值")
            angles[i] = max(0, min(270, angle))
    
    pub = _get_publisher()
    msg = Float64MultiArray()
    msg.data = angles
    pub.publish(msg)
    rospy.logdebug(f"发布关节角度(Float64MultiArray): {angles}")

# -------------------------- JointState发布节点类 --------------------------
class ArmJointPublisher:
    def __init__(self, node_name='arm_joint_publisher'):
        """
        初始化JointState类型的发布节点
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

    @staticmethod
    def deg_to_rad(degrees):
        """角度转弧度"""
        return degrees * 3.141592653589793 / 180.0

    @staticmethod
    def rad_to_deg(radians):
        """弧度转角度"""
        return radians * 180.0 / 3.141592653589793

    def validate_angle(self, angle_deg):
        """校验角度是否在合法范围"""
        if 0 <= angle_deg <= 270:
            return True
        rospy.logwarn(f"角度 {angle_deg}° 超出范围！有效范围：0-270度")
        return False

    def update_joint_angles(self):
        """手动更新关节角度"""
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
            
            # 更新角度（弧度）
            for i in range(len(self.joint_state.position)):
                self.previous_angles[i] = self.joint_state.position[i]
                self.joint_state.position[i] = self.deg_to_rad(new_angles_deg[i])
            rospy.loginfo("关节角度已更新！")
        
        except ValueError:
            rospy.logerr("输入错误！请输入有效的数字")
        except KeyboardInterrupt:
            rospy.loginfo("角度修改取消")

    def run(self):
        """运行发布节点主循环"""
        loop_count = 0
        while not rospy.is_shutdown():
            self.joint_state.header.stamp = rospy.Time.now()
            self.pub.publish(self.joint_state)
            loop_count += 1
            
            # 每10次循环询问是否修改角度
            if loop_count % 10 == 0:
                try:
                    response = input("\n是否修改关节角度？(y/n, 默认n): ")
                    if response.lower() == 'y':
                        self.update_joint_angles()
                except:
                    pass
            self.rate.sleep()

# -------------------------- 关节状态订阅节点类 --------------------------
class ArmStateSubscriber:
    def __init__(self, node_name='arm_state_subscriber'):
        """
        初始化关节状态订阅节点
        :param node_name: ROS节点名称，默认为arm_state_subscriber
        """
        rospy.init_node(node_name, anonymous=True)
        self.previous_positions = None
        rospy.Subscriber('/arm_joint_angles', JointState, self.joint_state_callback)
        rospy.loginfo("机械臂状态订阅节点已启动，等待数据...")
        rospy.loginfo("监控话题：/arm_joint_angles")
        self.msg_count = 0
        self.last_seq = -1

    @staticmethod
    def rad_to_deg(radians):
        """弧度转角度"""
        return radians * 180.0 / 3.141592653589793

    def joint_state_callback(self, msg):
        """关节状态回调函数"""
        self.msg_count += 1
        
        # 检查消息序列号是否连续（检测丢包）
        if hasattr(msg.header, 'seq') and self.last_seq != -1:
            if msg.header.seq != self.last_seq + 1:
                rospy.logwarn(f"可能丢包！当前seq={msg.header.seq}, 期望={self.last_seq + 1}")
        if hasattr(msg.header, 'seq'):
            self.last_seq = msg.header.seq

        # 清屏并打印状态
        print("\033[2J\033[H")  # 清屏（兼容Linux/macOS）
        print("=" * 60)
        print(f"机械臂关节状态监控 (消息#{self.msg_count})")
        print(f"时间戳: {msg.header.stamp.to_sec():.3f}s")
        print("=" * 60)
        print(f"{'关节名称':<10} {'当前角度(°)':<15} {'角度差值(°)':<15} {'状态':<10}")
        print("-" * 60)

        # 校验关节数量
        if len(msg.position) != 5:
            rospy.logerr(f"错误：期望5个关节，收到{len(msg.position)}个")
            return

        # 初始化上一帧数据
        if self.previous_positions is None:
            self.previous_positions = list(msg.position)

        # 打印每个关节状态
        for i in range(len(msg.name)):
            joint_name = msg.name[i] if i < len(msg.name) else f"joint{i+1}"
            current_rad = msg.position[i]
            current_deg = self.rad_to_deg(current_rad)
            prev_rad = self.previous_positions[i]
            prev_deg = self.rad_to_deg(prev_rad)
            diff_deg = current_deg - prev_deg

            # 判断运动状态
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

        # 更新上一帧数据
        self.previous_positions = list(msg.position)

    def run(self):
        """运行订阅节点（阻塞）"""
        rospy.spin()

# -------------------------- 对外接口函数 --------------------------
def start_publisher(node_name='arm_joint_publisher'):
    """
    启动JointState类型的关节角度发布节点（阻塞运行，直到节点关闭）
    :param node_name: 可选，指定ROS节点名称
    """
    publisher = ArmJointPublisher(node_name)
    try:
        publisher.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("发布节点已退出")

def start_subscriber(node_name='arm_state_subscriber'):
    """
    启动关节状态订阅节点（阻塞运行，直到节点关闭）
    :param node_name: 可选，指定ROS节点名称
    """
    subscriber = ArmStateSubscriber(node_name)
    try:
        subscriber.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("订阅节点已退出")

# -------------------------- 主函数（测试用） --------------------------
if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='机械臂关节控制工具')
    parser.add_argument('--mode', type=str, choices=['pub', 'sub', 'set'], 
                        default='pub', help='运行模式: pub(发布节点)/sub(订阅节点)/set(设置角度)')
    parser.add_argument('--angles', type=float, nargs=5, 
                        help='set模式下指定5个关节角度（0-270°）')
    
    args = parser.parse_args()
    
    if args.mode == 'pub':
        start_publisher()
    elif args.mode == 'sub':
        start_subscriber()
    elif args.mode == 'set':
        if not args.angles:
            print("错误：set模式需要指定5个关节角度，例如：--angles 0 30 60 90 120")
            sys.exit(1)
        rospy.init_node('arm_angle_setter', anonymous=True)
        set_arm_angles(args.angles)
        rospy.loginfo(f"已设置关节角度：{args.angles}")