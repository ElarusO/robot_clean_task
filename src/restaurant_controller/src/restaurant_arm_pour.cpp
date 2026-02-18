#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <vector>
#include <string>
#include <iostream>
#include <iomanip>

// 赛事参数宏定义（机械臂关节数、角度范围等）
#define ARM_JOINT_NUM    5
#define ARM_ANGLE_MIN    0.0
#define ARM_ANGLE_MAX    270.0
#define GRIPPER_ANGLE_MIN 0.0
#define GRIPPER_ANGLE_MAX 90.0

class RestaurantArmPour
{
public:
    RestaurantArmPour(ros::NodeHandle& nh)
        : nh_(nh)
    {
        // ---------- 复用项目1/2的通信逻辑 ----------
        // 假设已有话题：
        //   /arm_controller/command  (std_msgs/Float64MultiArray) 发布关节角度
        //   /gripper_controller/command (std_msgs/Float64) 发布爪部角度
        joint_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/arm_controller/command", 1);
        gripper_pub_ = nh_.advertise<std_msgs::Float64>("/gripper_controller/command", 1);

        // 若项目1/2提供的是函数库（例如RobotInterface类），则在此处实例化并保存指针
        // robot_interface_ = new RobotInterface();   // 示例，需替换为实际构造函数

        // 初始化关节状态：全部为0°
        joint_angles_.resize(ARM_JOINT_NUM, 0.0);
        gripper_angle_ = 0.0;

        ROS_INFO("RestaurantArmPour node initialized.");
    }

    ~RestaurantArmPour()
    {
        // 若使用了动态分配的接口对象，在此释放
        // delete robot_interface_;
    }

    // ---------- 核心动作序列（第2点） ----------
    void executePourSequence()
    {
        ROS_INFO("===== Start Pour & Store Sequence =====");

        // 步骤1：爪部闭合至90°（耗时1s）
        if (!setGripperAngle(90.0, 1.0, 1)) return;  // 1 为步骤编号，用于打印

        // 步骤2：机械臂抬升（耗时2s）
        std::vector<double> arm_cmd2 = {0.0, 60.0, 90.0, 0.0, 0.0};  // joint1~5
        if (!setJointAngles(arm_cmd2, 2.0, 2)) return;

        // 步骤3：机械臂旋转至倾倒角度（耗时1s）
        // 仅改变joint4=150°，其余保持当前值（从上一步继承）
        std::vector<double> arm_cmd3 = joint_angles_;   // 拷贝当前目标角度
        arm_cmd3[3] = 150.0;   // joint4索引为3（0-based）
        if (!setJointAngles(arm_cmd3, 1.0, 3)) return;

        // 步骤4：爪部松开至0°（耗时0.5s）
        if (!setGripperAngle(0.0, 0.5, 4)) return;

        // 步骤5：机械臂复位（耗时2s）
        std::vector<double> arm_cmd5(ARM_JOINT_NUM, 0.0);  // 全0
        if (!setJointAngles(arm_cmd5, 2.0, 5)) return;

        // 步骤6：爪部保持0°（无耗时要求，仅设置状态并打印）
        if (!setGripperAngle(0.0, 0.0, 6)) return;   // 持续时间为0，表示不等待

        ROS_INFO("===== Sequence Completed =====");
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher joint_pub_;
    ros::Publisher gripper_pub_;

    // 若使用函数库接口，可注释掉上面的publisher，改用下面的指针
    // RobotInterface* robot_interface_;

    std::vector<double> joint_angles_;   // 当前关节目标角度缓存
    double gripper_angle_;              // 当前爪部目标角度缓存

    // ---------- 关节角度范围校验（第4点） ----------
    bool checkJointAngle(int joint_id, double angle)
    {
        if (angle < ARM_ANGLE_MIN || angle > ARM_ANGLE_MAX)
        {
            ROS_ERROR_STREAM("Joint" << joint_id+1 << " angle " << angle 
                            << " out of range [" << ARM_ANGLE_MIN << ", " << ARM_ANGLE_MAX << "]");
            return false;
        }
        return true;
    }

    bool checkGripperAngle(double angle)
    {
        if (angle < GRIPPER_ANGLE_MIN || angle > GRIPPER_ANGLE_MAX)
        {
            ROS_ERROR_STREAM("Gripper angle " << angle 
                            << " out of range [" << GRIPPER_ANGLE_MIN << ", " << GRIPPER_ANGLE_MAX << "]");
            return false;
        }
        return true;
    }

    // ---------- 复位逻辑（第4点） ----------
    void resetRobot()
    {
        ROS_WARN("Resetting robot to initial position...");
        // 机械臂复位：所有关节0°
        std::vector<double> zero_angles(ARM_JOINT_NUM, 0.0);
        if (!publishJointCommand(zero_angles))
            ROS_ERROR("Failed to reset arm!");
        // 爪部复位：0°
        if (!publishGripperCommand(0.0))
            ROS_ERROR("Failed to reset gripper!");

        // 更新缓存状态
        joint_angles_ = zero_angles;
        gripper_angle_ = 0.0;

        ros::Duration(1.0).sleep(); // 给复位动作预留时间
    }

    // ---------- 实际发送命令的底层函数（复用通信逻辑） ----------
    bool publishJointCommand(const std::vector<double>& angles)
    {
        // 方式A：话题发布（本示例采用）
        std_msgs::Float64MultiArray msg;
        msg.data = angles;
        joint_pub_.publish(msg);

        // 方式B：若项目1/2提供函数库接口，则替换为：
        // robot_interface_->setJointAngles(angles);
        return true;
    }

    bool publishGripperCommand(double angle)
    {
        // 方式A：话题发布
        std_msgs::Float64 msg;
        msg.data = angle;
        gripper_pub_.publish(msg);

        // 方式B：函数库接口
        // robot_interface_->setGripperAngle(angle);
        return true;
    }

    // ---------- 带校验、状态更新、耗时等待及终端打印的关节设置函数 ----------
    bool setJointAngles(const std::vector<double>& target_angles, double duration, int step_id)
    {
        // 1. 校验每个关节角度
        for (size_t i = 0; i < target_angles.size(); ++i)
        {
            if (!checkJointAngle(i, target_angles[i]))
            {
                errorHandler("Joint angle out of range", step_id);
                return false;
            }
        }

        // 2. 发送命令
        publishJointCommand(target_angles);

        // 3. 更新缓存
        joint_angles_ = target_angles;

        // 4. 终端打印：根据步骤号定制输出内容（第3点）
        printStepStatus(step_id, target_angles);

        // 5. 耗时等待（若duration>0）
        if (duration > 0)
            ros::Duration(duration).sleep();

        return true;
    }

    // 爪部专用设置函数
    bool setGripperAngle(double target_angle, double duration, int step_id)
    {
        if (!checkGripperAngle(target_angle))
        {
            errorHandler("Gripper angle out of range", step_id);
            return false;
        }

        publishGripperCommand(target_angle);
        gripper_angle_ = target_angle;

        printStepStatus(step_id, target_angle);

        if (duration > 0)
            ros::Duration(duration).sleep();

        return true;
    }

    // ---------- 终端状态打印（第3点） ----------
    void printStepStatus(int step_id, const std::vector<double>& angles)
    {
        std::cout << "步骤" << step_id << "：";
        switch (step_id)
        {
        case 1:
            std::cout << "爪部闭合中，爪部角度" << gripper_angle_ << "°";
            break;
        case 2:
            std::cout << "机械臂抬升中，joint2当前角度" << angles[1] << "°，joint3当前角度" << angles[2] << "°";
            break;
        case 3:
            std::cout << "机械臂旋转至倾倒角度，joint4当前角度" << angles[3] << "°";
            break;
        case 4:
            std::cout << "爪部松开中，爪部角度" << gripper_angle_ << "°";
            break;
        case 5:
            std::cout << "机械臂复位中，所有关节回归初始值";
            break;
        case 6:
            std::cout << "爪部保持0°，收纳准备状态";
            break;
        default:
            std::cout << "执行自定义动作";
        }
        std::cout << std::endl;
    }

    // 重载：爪部专用打印
    void printStepStatus(int step_id, double gripper_angle)
    {
        std::cout << "步骤" << step_id << "：";
        if (step_id == 1 || step_id == 4 || step_id == 6)
        {
            std::cout << "爪部角度" << gripper_angle << "°";
        }
        std::cout << std::endl;
    }

    // ---------- 错误处理：打印错误、复位、终止序列（第4点） ----------
    void errorHandler(const std::string& err_msg, int step_id)
    {
        ROS_ERROR_STREAM("Step " << step_id << " error: " << err_msg);
        resetRobot();
        ROS_ERROR("Sequence aborted due to error.");
        // 根据赛事要求，可以选择退出节点或抛出异常
        ros::shutdown();
        exit(1);
    }
};

// ---------- 主函数 ----------
int main(int argc, char** argv)
{
    ros::init(argc, argv, "restaurant_arm_pour");
    ros::NodeHandle nh;

    RestaurantArmPour controller(nh);

    // 执行核心动作序列
    controller.executePourSequence();

    // 保持节点运行，等待可能的后续指令（若需要）
    ros::spin();

    return 0;
}