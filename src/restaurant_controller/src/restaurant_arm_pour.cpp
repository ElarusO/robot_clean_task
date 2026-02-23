#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <vector>
#include <string>
#include <iostream>
#include <iomanip>
#include <std_msgs/String.h>
#include <ctime>
#include <sstream>

// 赛事参数宏定义（机械臂关节数、角度范围等）
#define ARM_JOINT_NUM    5
#define ARM_ANGLE_MIN    0.0
#define ARM_ANGLE_MAX    270.0
#define GRIPPER_ANGLE_MIN 0.0
#define GRIPPER_ANGLE_MAX 90.0

class RestaurantArmPour
{
public:

    void publishStatus()
    {
        std_msgs::String status_msg;
        status_msg.data = current_status_;
        status_pub_.publish(status_msg);
    } 
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
        status_pub_ = nh_.advertise<std_msgs::String>("/arm_pour_status", 1);
        current_status_ = "initialized";  // 初始状态
        publishStatus();  // 发布初始状态
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
        current_status_ = "step_1_gripper_close";  // 更新状态
        publishStatus();   
        if (!setGripperAngle(90.0, 1.0, 1)) return;  // 1 为步骤编号，用于打印

        // 步骤2：机械臂抬升（耗时2s）
        current_status_ = "step_2_arm_lift";
        publishStatus();   
        std::vector<double> arm_cmd2 = {0.0, 60.0, 90.0, 0.0, 0.0};  // joint1~5
        if (!setJointAngles(arm_cmd2, 2.0, 2)) return;

        // 步骤3：机械臂旋转至倾倒角度（耗时1s）
        // 仅改变joint4=150°，其余保持当前值（从上一步继承）
        current_status_ = "step_3_arm_rotate";  
        publishStatus();       
        std::vector<double> arm_cmd3 = joint_angles_;   // 拷贝当前目标角度
        arm_cmd3[3] = 150.0;   // joint4索引为3（0-based）
        if (!setJointAngles(arm_cmd3, 1.0, 3)) return;

        // 步骤4：爪部松开至0°（耗时0.5s）
        current_status_ = "step_4_gripper_open";
        publishStatus();   
        if (!setGripperAngle(0.0, 0.5, 4)) return;

        // 步骤5：机械臂复位（耗时2s）
        current_status_ = "step_5_arm_reset";
        publishStatus();   
        std::vector<double> arm_cmd5(ARM_JOINT_NUM, 0.0);  // 全0
        if (!setJointAngles(arm_cmd5, 2.0, 5)) return;

        // 步骤6：爪部保持0°（无耗时要求，仅设置状态并打印）
        current_status_ = "step_6_gripper_idle";
        publishStatus();  
        if (!setGripperAngle(0.0, 0.0, 6)) return;   // 持续时间为0，表示不等待

        current_status_ = "sequence_completed";
        publishStatus();

        ROS_INFO("===== Sequence Completed =====");
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher joint_pub_;
    ros::Publisher gripper_pub_;
    ros::Publisher status_pub_;     
    std::string current_status_;    

    // 若使用函数库接口，可注释掉上面的publisher，改用下面的指针
    // RobotInterface* robot_interface_;

    std::vector<double> joint_angles_;   // 当前关节目标角度缓存
    double gripper_angle_;              // 当前爪部目标角度缓存

    // 获取格式化时间戳（YYYY-MM-DD HH:MM:SS.ms）
    std::string getTimestamp()
    {
        std::time_t now = std::time(nullptr);
        std::tm* tm_now = std::localtime(&now);
        std::ostringstream oss;
        
        // 秒级时间
        oss << std::put_time(tm_now, "%Y-%m-%d %H:%M:%S");
        // 毫秒级补充
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()
        ).count() % 1000;
        oss << "." << std::setfill('0') << std::setw(3) << ms;
        
        return oss.str();
    }

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

        // 4. 终端打印：统一格式[时间戳 | 步骤X | 状态] 动作说明+耗时+角度
        printStepStatus(step_id, target_angles, duration);

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

        // 终端打印：统一格式
        printStepStatus(step_id, target_angle, duration);

        if (duration > 0)
            ros::Duration(duration).sleep();

        return true;
    }

    // ---------- 终端状态打印（统一格式） ----------
    void printStepStatus(int step_id, const std::vector<double>& angles, double duration)
    {
        std::string timestamp = getTimestamp();
        std::string step_str = std::to_string(step_id);
        std::string status = "执行中";
        std::string action_desc, time_cost, angle_info;

        // 构造动作说明、耗时、角度信息
        switch (step_id)
        {
        case 2:
            action_desc = "机械臂抬升";
            time_cost = std::to_string(duration) + "s";
            angle_info = "joint2=" + std::to_string(angles[1]) + "°,joint3=" + std::to_string(angles[2]) + "°";
            break;
        case 3:
            action_desc = "机械臂旋转至倾倒角度";
            time_cost = std::to_string(duration) + "s";
            angle_info = "joint4=" + std::to_string(angles[3]) + "°";
            break;
        case 5:
            action_desc = "机械臂复位";
            time_cost = std::to_string(duration) + "s";
            angle_info = "所有关节=0°";
            break;
        default:
            action_desc = "执行自定义关节动作";
            time_cost = std::to_string(duration) + "s";
            angle_info = "关节角度=[";
            for (size_t i=0; i<angles.size(); ++i) {
                angle_info += std::to_string(angles[i]) + "°";
                if (i != angles.size()-1) angle_info += ",";
            }
            angle_info += "]";
            break;
        }

        // 拼接统一格式并输出
        std::cout << "[" << timestamp << " | 步骤" << step_str << " | " << status << "] " 
                  << action_desc << "+" << time_cost << "+" << angle_info << std::endl;
    }

    // 重载：爪部专用打印
    void printStepStatus(int step_id, double gripper_angle, double duration)
    {
        std::string timestamp = getTimestamp();
        std::string step_str = std::to_string(step_id);
        std::string status = "执行中";
        std::string action_desc, time_cost, angle_info;

        switch (step_id)
        {
        case 1:
            action_desc = "爪部闭合";
            time_cost = std::to_string(duration) + "s";
            angle_info = "爪部角度=" + std::to_string(gripper_angle) + "°";
            break;
        case 4:
            action_desc = "爪部松开";
            time_cost = std::to_string(duration) + "s";
            angle_info = "爪部角度=" + std::to_string(gripper_angle) + "°";
            break;
        case 6:
            action_desc = "爪部保持空闲状态";
            time_cost = (duration > 0) ? (std::to_string(duration) + "s") : "0s";
            angle_info = "爪部角度=" + std::to_string(gripper_angle) + "°";
            break;
        default:
            action_desc = "执行自定义爪部动作";
            time_cost = (duration > 0) ? (std::to_string(duration) + "s") : "0s";
            angle_info = "爪部角度=" + std::to_string(gripper_angle) + "°";
            break;
        }

        // 拼接统一格式并输出
        std::cout << "[" << timestamp << " | 步骤" << step_str << " | " << status << "] " 
                  << action_desc << "+" << time_cost << "+" << angle_info << std::endl;
    }

    // ---------- 错误处理：打印错误、复位、终止序列（第4点） ----------
    void errorHandler(const std::string& err_msg, int step_id)
    {
        std::string timestamp = getTimestamp();
        // 错误状态打印也遵循统一格式
        std::cout << "[" << timestamp << " | 步骤" << step_id << " | 失败] " 
                  << "执行出错：" << err_msg << "+0s+0°" << std::endl;
        
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