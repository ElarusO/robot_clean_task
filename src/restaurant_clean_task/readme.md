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