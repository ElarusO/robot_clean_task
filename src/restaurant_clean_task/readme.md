# 餐厅赛项ROS功能包 README.md
## 1. 运行步骤
### 1.1 环境准备
确保已安装ROS（推荐Melodic/Noetic版本），并完成功能包依赖项安装，依赖项可通过`rosdep install --from-paths src --ignore-src -r -y`命令自动安装。

### 1.2 工作空间编译
将该功能包放入ROS工作空间的`src`目录下，回到工作空间根目录执行编译：
```bash
catkin_make
# 若使用catkin工具链，可执行
# catkin build
```

### 1.3 环境变量加载
编译完成后，加载工作空间环境变量：
```bash
source devel/setup.bash
```

### 1.4 核心节点运行

#### launch一键启动:
```bash
roslaunch restaurant_clean_task clean_task.launch
```

#### 附加项目:
```bash
roslaunch restaurant_clean_task bathroom_task.launch 
```

#### 餐厅赛项主任务启动
启动餐厅赛项完整任务逻辑（包含所有核心节点）：
```bash
roslaunch restaurant_task restaurant_task.launch
```

#### 附加项目（卫浴赛项）运行
若需测试附加项目`bathroom_arm_action`节点，执行：
```bash
rosrun bathroom_arm_action bathroom_arm_action_node
# 或通过launch文件启动（若存在）
# roslaunch bathroom_arm_action bathroom_arm_action.launch
```

### 1.5 任务触发
餐厅赛项主任务可通过ROS话题/服务触发，默认触发指令：
```bash
rosservice call /restaurant_task/start_task "{}"
```

## 2. 功能说明
### 2.1 核心功能（餐厅赛项）
该ROS功能包实现餐厅赛项全流程任务逻辑，涵盖以下核心能力：
- 环境感知：通过传感器（摄像头、激光雷达等）感知餐厅场景内餐桌、餐具、菜品、人员位置等信息；
- 动作规划：针对取餐、送餐、摆台、清理餐桌等餐厅赛项典型任务，完成机械臂/移动底盘的动作序列规划；
- 任务调度：按照赛项规则，自动调度各子任务（如先摆台→取餐→送餐→清理）的执行顺序，处理任务优先级与异常中断；
- 状态反馈：实时发布任务执行状态（如"执行中/完成/失败"）、设备位姿、环境感知结果等ROS话题，支持可视化监控。

### 2.2 附加功能（卫浴赛项）
独立节点`bathroom_arm_action`为卫浴赛项核心动作逻辑封装，与餐厅赛项主任务解耦，具备以下能力：
- 封装卫浴场景下机械臂核心动作（如开关水龙头、取放洗漱用品、清洁台面等）；
- 提供ROS Action接口，支持动作目标下发、执行状态反馈、结果回调；
- 可独立运行，也可通过话题/服务与其他赛项模块联动（非餐厅赛项必需）。

### 2.3 接口说明
| 类型       | 名称                          | 功能描述                     |
|------------|-------------------------------|------------------------------|
| Service    | /restaurant_task/start_task   | 触发餐厅赛项主任务开始执行   |
| Service    | /restaurant_task/stop_task    | 终止当前运行的餐厅赛项任务   |
| Topic      | /restaurant_task/status       | 发布任务实时状态（字符串）   |
| Action     | /bathroom_arm_action/goal     | 下发卫浴赛项机械臂动作目标   |
| Action     | /bathroom_arm_action/result   | 反馈卫浴赛项机械臂动作结果   |

## 3. 模块注释
### 3.1 餐厅赛项核心模块
#### `restaurant_task_node.cpp`（主任务节点）
```cpp
/**
 * @brief 餐厅赛项主任务节点初始化函数
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 * @return int 初始化结果（0为成功）
 * @details 初始化ROS节点、订阅/发布话题、服务、动作客户端，加载任务配置参数
 */
int initRestaurantTask(int argc, char** argv);

/**
 * @brief 餐厅赛项任务调度核心函数
 * @details 按照赛项规则依次执行摆台、取餐、送餐、清理等子任务，处理任务中断与异常恢复
 *          实时更新任务状态并发布到/status话题
 */
void taskScheduler();

/**
 * @brief 摆台子任务执行函数
 * @return bool 子任务执行结果（true为成功）
 * @details 控制移动底盘到指定餐桌位置，机械臂完成餐具摆放动作，包含动作校验逻辑
 */
bool tableSettingTask();

/**
 * @brief 取餐子任务执行函数
 * @return bool 子任务执行结果（true为成功）
 * @details 接收取餐目标坐标，控制底盘移动到出餐口，机械臂抓取菜品并完成夹取校验
 */
bool mealFetchingTask();

/**
 * @brief 送餐子任务执行函数
 * @return bool 子任务执行结果（true为成功）
 * @details 规划送餐路径，控制底盘移动到目标餐桌，机械臂放置菜品，支持路径避障
 */
bool mealDeliveryTask();

/**
 * @brief 清理餐桌子任务执行函数
 * @return bool 子任务执行结果（true为成功）
 * @details 机械臂拾取餐桌残留餐具/垃圾，控制底盘移动到回收区完成投放
 */
bool tableCleaningTask();

/**
 * @brief 任务异常处理函数
 * @param error_code 错误码（1-摆台失败，2-取餐失败，3-送餐失败，4-清理失败）
 * @details 根据错误码执行重试/终止/降级策略，发布异常信息到/status话题
 */
void errorHandler(int error_code);
```

#### `restaurant_perception.cpp`（环境感知模块）
```cpp
/**
 * @brief 餐桌/菜品/人员位置感知函数
 * @param frame_id 感知坐标系ID
 * @return geometry_msgs::PoseArray 感知到的目标位姿数组
 * @details 订阅摄像头/激光雷达数据，通过视觉识别+点云处理完成目标定位，输出世界坐标系下的位姿
 */
geometry_msgs::PoseArray perceiveTargets(std::string frame_id);

/**
 * @brief 障碍物检测函数
 * @return bool 是否检测到障碍物（true为检测到）
 * @details 解析激光雷达数据，判断移动路径上是否存在障碍物，为避障规划提供依据
 */
bool obstacleDetection();
```

### 3.2 附加模块（卫浴赛项）
#### `bathroom_arm_action_node.cpp`（卫浴赛项核心动作节点）
```cpp
/**
 * @brief 卫浴赛项机械臂动作服务端初始化函数
 * @param nh ROS节点句柄
 * @details 初始化Action服务端，注册动作回调函数，加载机械臂关节限位、速度参数
 */
void initBathroomArmActionServer(ros::NodeHandle& nh);

/**
 * @brief 卫浴机械臂动作执行回调函数
 * @param goal 动作目标（包含动作类型、目标位姿、执行时长）
 * @param as 动作服务端对象
 * @details 根据接收的动作目标（如开关水龙头、取放洗漱用品），解析并执行对应的机械臂关节运动序列
 *          实时反馈动作执行进度，执行完成后返回结果
 */
void executeBathroomArmAction(const bathroom_arm_action::ArmGoalConstPtr& goal, 
                              actionlib::SimpleActionServer<bathroom_arm_action::ArmAction>* as);

/**
 * @brief 水龙头开关动作执行函数
 * @param target_pose 水龙头目标位姿
 * @return bool 动作执行结果（true为成功）
 * @details 控制机械臂末端执行器到达水龙头位置，完成旋转开关动作，包含力反馈校验
 */
bool faucetSwitchAction(geometry_msgs::Pose target_pose);

/**
 * @brief 洗漱用品取放动作执行函数
 * @param item_pose 洗漱用品目标位姿
 * @param action_type 动作类型（0-拾取，1-放置）
 * @return bool 动作执行结果（true为成功）
 * @details 根据动作类型控制机械臂完成洗漱用品的拾取或放置，支持不同尺寸物品的抓取适配
 */
bool toiletriesHandleAction(geometry_msgs::Pose item_pose, int action_type);
```

### 3.3 配置文件模块
#### `restaurant_task_params.yaml`（参数配置文件）
```yaml
# 餐厅赛项基础参数
task_config:
  max_retry_times: 3        # 子任务最大重试次数
  task_timeout: 120.0       # 主任务总超时时间（秒）
  table_setting_timeout: 20 # 摆台子任务超时时间（秒）
  meal_fetching_timeout: 15 # 取餐子任务超时时间（秒）
  meal_delivery_timeout: 30 # 送餐子任务超时时间（秒）
  table_cleaning_timeout: 25 # 清理子任务超时时间（秒）

# 机械臂参数
arm_config:
  max_joint_speed: 0.5      # 关节最大运动速度（rad/s）
  end_effector_force: 10.0  # 末端执行器最大夹持力（N）

# 移动底盘参数
base_config:
  max_linear_speed: 0.3     # 底盘最大线速度（m/s）
  max_angular_speed: 0.5    # 底盘最大角速度（rad/s）
  obstacle_threshold: 0.5   # 障碍物检测阈值（m）

# 卫浴赛项附加参数（仅bathroom_arm_action节点使用）
bathroom_arm_config:
  faucet_rotate_angle: 1.57 # 水龙头旋转角度（rad）
  grip_force: 5.0           # 卫浴用品抓取力（N）
```


# ROS餐厅赛项功能包异常处理说明
## 一、文档概述
本文档针对ROS餐厅赛项完整任务逻辑功能包及附加的`bathroom_arm_action`节点（卫浴赛项核心动作逻辑封装），明确各环节异常类型、处理策略、错误码定义及日志规范，保障系统鲁棒性与可维护性。

## 二、核心设计原则
1. **分级处理**：轻微异常（如传感器临时抖动）自动恢复；严重异常（如硬件通信中断）触发告警并终止任务；
2. **日志可追溯**：所有异常需记录上下文（时间、节点、任务阶段、参数）；
3. **解耦性**：异常处理逻辑不侵入核心业务逻辑，通过ROS的`try-catch`、服务/动作回调返回码、话题状态监听实现；
4. **用户可感知**：关键异常通过ROS话题/可视化界面输出提示，便于现场调试。

## 三、通用异常分类与处理策略
### 3.1 基础通信异常
| 异常类型                | 场景示例                                  | 处理策略                                                                 |
|-------------------------|-------------------------------------------|--------------------------------------------------------------------------|
| ROS节点启动失败         | 依赖的库缺失、节点名冲突、权限不足        | 1. 启动脚本中检查依赖；2. 输出明确错误日志（缺失库名/冲突节点名）；3. 终止启动并返回错误码100。 |
| 话题/服务/动作连接超时  | 目标节点未启动、网络隔离、话题名拼写错误  | 1. 启动时检查依赖的话题/服务/动作列表；2. 超时（默认5s）后重试3次；3. 仍失败则记录日志并触发告警（错误码101）。 |
| 消息发布/订阅异常       | 消息类型不匹配、队列满、发布者无权限      | 1. 发布前校验消息类型；2. 队列满时清空缓存并重试；3. 权限异常输出系统权限日志（错误码102）。 |

### 3.2 硬件交互异常
| 异常类型                | 场景示例                                  | 处理策略                                                                 |
|-------------------------|-------------------------------------------|--------------------------------------------------------------------------|
| 传感器数据异常          | 激光雷达/摄像头无数据、数据超出合理范围   | 1. 校验数据阈值（如距离<0或>10m）；2. 临时异常则丢弃该帧数据，使用上一帧有效值；3. 连续10帧异常则标记传感器故障（错误码200）。 |
| 执行器（机械臂/底盘）无响应 | 电机卡死、串口通信中断、执行指令超时      | 1. 指令发送后等待ACK（默认3s）；2. 超时后发送停止指令并重试2次；3. 仍失败则触发硬件告警（错误码201），停止当前任务并复位执行器。 |
| 硬件状态反馈异常        | 执行器状态与指令不符（如指令前进但反馈静止） | 1. 对比指令与反馈状态；2. 单次异常则重新发送指令；3. 连续3次不符则判定硬件故障（错误码202）。 |

### 3.3 任务逻辑异常
| 异常类型                | 场景示例                                  | 处理策略                                                                 |
|-------------------------|-------------------------------------------|--------------------------------------------------------------------------|
| 任务阶段跳转失败        | 未完成上一阶段却触发下一阶段、阶段条件不满足 | 1. 增加阶段状态校验（如标志位/计数器）；2. 跳转失败时回滚到上一有效阶段；3. 记录阶段上下文（错误码300）。 |
| 目标识别/定位失败       | 餐桌/餐具识别不到、坐标计算错误           | 1. 调用备用识别模型；2. 定位失败时使用预设坐标；3. 连续5次失败则终止当前子任务（错误码301）。 |
| 资源抢占冲突            | 机械臂与底盘同时请求控制权限              | 1. 设计权限优先级（机械臂>底盘）；2. 冲突时阻塞低优先级任务，释放后重试；3. 记录资源占用日志（错误码302）。 |

## 四、`bathroom_arm_action`节点（卫浴赛项）专属异常处理
### 4.1 专属异常场景
| 异常类型                | 场景示例                                  | 处理策略                                                                 |
|-------------------------|-------------------------------------------|--------------------------------------------------------------------------|
| 卫浴配件抓取异常        | 毛巾/洗漱杯抓取偏移、夹爪力度不足/过大    | 1. 实时反馈夹爪压力值，动态调整力度；2. 抓取偏移时重新规划抓取位姿；3. 连续3次抓取失败则切换抓取方式（错误码400）。 |
| 防水/限位触发异常       | 机械臂进入防水禁区、关节限位报警          | 1. 预设防水区域坐标，运动前校验；2. 触发限位时立即停止运动并回退安全位置；3. 记录限位触发位置（错误码401）。 |
| 卫浴动作流程异常        | 如“放水→取杯→接水”流程乱序                | 1. 基于有限状态机（FSM）管控流程；2. 乱序时重置流程到初始状态；3. 输出流程乱序的触发条件（错误码402）。 |

### 4.2 与餐厅赛项的联动异常处理
若`bathroom_arm_action`作为附加节点与餐厅赛项节点共存：
1. 资源冲突：机械臂权限优先分配给卫浴赛项（附加项），餐厅赛项暂存任务，卫浴任务完成后恢复；
2. 通信隔离：卫浴节点异常时不影响餐厅赛项核心任务，仅标记附加项故障（错误码403）；
3. 日志隔离：卫浴节点异常日志单独输出到`/bathroom_arm_error.log`，避免混淆餐厅赛项日志。

## 五、错误码规范
| 错误码区间 | 归属模块               | 说明                     |
|------------|------------------------|--------------------------|
| 100-199    | ROS通信层              | 节点/话题/服务相关异常   |
| 200-299    | 硬件交互层             | 传感器/执行器相关异常    |
| 300-399    | 餐厅赛项任务逻辑层     | 任务流程/阶段相关异常    |
| 400-499    | bathroom_arm_action节点 | 卫浴赛项专属异常         |
| 900-999    | 系统级异常             | 如电源故障、系统崩溃     |

## 六、日志输出规范
1. **日志格式**：`[时间戳][节点名][错误码][任务阶段] 异常描述 + 上下文参数`  
   示例：`[2024-05-20 10:23:45][dining_arm][301][餐具抓取阶段] 餐桌识别失败，当前摄像头坐标：x=1.2,y=0.5,z=0.8`
2. **日志级别**：
   - DEBUG：调试信息（如正常阶段跳转）；
   - WARN：轻微异常（如单次传感器数据抖动）；
   - ERROR：严重异常（如硬件故障、任务终止）；
   - FATAL：系统级故障（如节点崩溃）。
3. **日志存储**：
   - 餐厅赛项核心日志：`~/.ros/log/dining_task.log`；
   - bathroom_arm_action日志：`~/.ros/log/bathroom_arm.log`；
   - 异常汇总日志：`~/.ros/log/error_summary.log`（每日归档）。

## 七、异常恢复机制
1. **自动恢复**：
   - 通信超时：重试3次，间隔1s；
   - 传感器单帧异常：丢弃数据，使用上一帧；
   - 执行器单次无响应：重新发送指令。
2. **手动恢复**：
   - 严重异常（如错误码201/401）触发后，系统输出恢复提示（如“机械臂无响应，请检查串口连接”）；
   - 提供ROS服务`/recovery_task`，支持手动重置指定任务阶段；
   - 提供`/reset_arm`服务，重置bathroom_arm_action节点到初始状态。

## 八、代码层实现示例（关键片段）
### 8.1 通用异常捕获（餐厅赛项节点）
```python
import rospy
from std_srvs.srv import Empty, EmptyResponse

class DiningTaskNode:
    def __init__(self):
        rospy.init_node("dining_task_node")
        self.task_phase = 0  # 0:初始化,1:导航,2:抓取,3:放置
        self.error_code = 0
        # 注册恢复服务
        self.recovery_srv = rospy.Service("/recovery_task", Empty, self.recovery_callback)

    def check_topic_connection(self, topic_name, timeout=5):
        """检查话题连接"""
        try:
            start_time = rospy.Time.now()
            while not rospy.is_shutdown():
                if rospy.Time.now() - start_time > rospy.Duration(timeout):
                    self.error_code = 101
                    rospy.logerr(f"[错误码{self.error_code}] 话题{topic_name}连接超时")
                    return False
                if rospy.get_published_topics().count(topic_name):
                    return True
                rospy.sleep(0.1)
        except Exception as e:
            self.error_code = 102
            rospy.logerr(f"[错误码{self.error_code}] 话题检查异常：{str(e)}")
            return False

    def execute_task_phase(self):
        """执行任务阶段，捕获逻辑异常"""
        try:
            if self.task_phase == 1:
                self.nav_to_table()
            elif self.task_phase == 2:
                self.grab_tableware()
            # 其他阶段...
        except Exception as e:
            self.error_code = 300
            rospy.logerr(f"[错误码{self.error_code}] 任务阶段{self.task_phase}执行失败：{str(e)}")
            self.rollback_phase()  # 回滚到上一阶段

    def recovery_callback(self, req):
        """恢复任务回调"""
        self.task_phase = 0
        self.error_code = 0
        rospy.loginfo("任务已重置到初始阶段")
        return EmptyResponse()

if __name__ == "__main__":
    node = DiningTaskNode()
    try:
        if not node.check_topic_connection("/table_detection"):
            rospy.signal_shutdown("关键话题连接失败")
        node.execute_task_phase()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logwarn("节点被中断")
    except Exception as e:
        rospy.logfatal(f"系统级异常：{str(e)}")
```

### 8.2 bathroom_arm_action节点异常处理（动作服务器）
```cpp
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "bathroom_arm/BathroomArmAction.h"

class BathroomArmActionServer {
private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<bathroom_arm::BathroomArmAction> as_;
    std::string action_name_;
    bathroom_arm::BathroomArmFeedback feedback_;
    bathroom_arm::BathroomArmResult result_;
    int error_code_;

    // 检查夹爪压力
    bool check_gripper_pressure(float pressure) {
        if (pressure < 5.0 || pressure > 30.0) { // 压力阈值
            error_code_ = 400;
            ROS_ERROR("[错误码%d] 夹爪压力异常：%.2f", error_code_, pressure);
            return false;
        }
        return true;
    }

    // 检查防水区域
    bool check_waterproof_area(float x, float y) {
        // 预设防水区域：x∈[0, 0.5], y∈[0, 0.5]
        if (x >= 0 && x <= 0.5 && y >= 0 && y <= 0.5) {
            error_code_ = 401;
            ROS_ERROR("[错误码%d] 机械臂进入防水禁区：(%.2f, %.2f)", error_code_, x, y);
            // 立即回退安全位置
            move_to_safe_position();
            return false;
        }
        return true;
    }

    void executeCB(const bathroom_arm::BathroomArmGoalConstPtr& goal) {
        ros::Rate r(10);
        bool success = true;
        error_code_ = 0;

        // 执行抓取动作
        for (int i = 0; i < goal->grab_steps; i++) {
            if (as_.isPreemptRequested() || !ros::ok()) {
                ROS_WARN("动作被抢占/节点异常");
                as_.setPreempted();
                success = false;
                break;
            }

            // 检查压力和防水区域
            float current_pressure = get_gripper_pressure(); // 读取硬件压力
            float x = get_arm_x(), y = get_arm_y(); // 读取机械臂坐标
            if (!check_gripper_pressure(current_pressure) || !check_waterproof_area(x, y)) {
                success = false;
                break;
            }

            // 反馈进度
            feedback_.progress = (i+1)*100.0 / goal->grab_steps;
            as_.publishFeedback(feedback_);
            r.sleep();
        }

        // 设置结果
        if (success) {
            result_.success = true;
            result_.error_code = 0;
            as_.setSucceeded(result_);
        } else {
            result_.success = false;
            result_.error_code = error_code_;
            as_.setAborted(result_);
        }
    }

public:
    BathroomArmActionServer(std::string name) : as_(nh_, name, boost::bind(&BathroomArmActionServer::executeCB, this, _1), false),
                                               action_name_(name) {
        as_.start();
        ROS_INFO("bathroom_arm_action 动作服务器启动");
    }

    ~BathroomArmActionServer() {}

    void move_to_safe_position() {
        // 发送机械臂回退安全位置指令
        ROS_INFO("机械臂回退到安全位置");
    }

    float get_gripper_pressure() {
        // 模拟读取硬件压力值
        return 20.0;
    }

    float get_arm_x() { return 0.6; }
    float get_arm_y() { return 0.6; }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "bathroom_arm_action_server");
    try {
        BathroomArmActionServer arm_server("bathroom_arm_action");
        ros::spin();
    } catch (std::exception& e) {
        ROS_FATAL("bathroom_arm_action 节点异常：%s", e.what());
        return 1;
    }
    return 0;
}
```

## 九、调试与维护建议
1. 部署阶段：开启ROS_DEBUG日志，验证所有异常场景的触发与处理逻辑；
2. 运行阶段：定期检查`error_summary.log`，统计高频异常（如传感器数据异常）并优化硬件/算法；
3. 故障定位：通过错误码+上下文日志快速定位问题，优先排查硬件连接（如串口、电源）；
4. 迭代优化：针对现场高频异常，补充异常处理逻辑（如新增特定场景的重试策略）。