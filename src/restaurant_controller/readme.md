# restaurant_controller
ROS功能包，实现餐厅赛项中机械臂餐具倾倒+收纳的核心动作序列控制。

## 1. 运行步骤
### 1.1 环境依赖
- ROS Kinetic/Melodic/Noetic（需适配对应系统）
- 依赖ROS功能包：`roscpp`、`sensor_msgs`、`std_msgs`
- 已配置好的ROS工作空间（如`catkin_ws`）

### 1.2 编译功能包
1. 将功能包放入ROS工作空间的`src`目录：
   ```bash
   cd ~/catkin_ws/src
   git clone <本仓库地址>  # 若为本地文件直接拷贝即可
   ```
2. 编译工作空间：
   ```bash
   cd ~/catkin_ws
   catkin_make
   ```
3. 刷新ROS环境变量：
   ```bash
   source devel/setup.bash
   ```

### 1.3 启动节点
#### launch一键启动:
```bash
roslaunch restaurant_controller clean_task.launch 
```
1. 启动ROS核心：
   ```bash
   roscore
   ```
2. 启动机械臂倾倒控制节点：
   ```bash
   rosrun restaurant_controller restaurant_arm_pour
   ```

### 1.4 节点状态检测
执行脚本检测节点运行状态：
```bash
cd ~/catkin_ws/src/restaurant_controller
chmod +x check_arm_node.sh
./check_arm_node.sh
```
脚本会检测：节点存活状态、关节指令话题输出、自定义状态话题输出。

## 2. 功能说明
### 2.1 核心功能
实现机械臂"餐具倾倒+收纳"的标准化动作序列，包含6个核心步骤，全程带状态发布、角度校验、错误处理及复位机制。

### 2.2 动作序列
| 步骤 | 动作描述                | 耗时  | 核心参数                          |
|------|-------------------------|-------|-----------------------------------|
| 1    | 爪部闭合至90°           | 1s    | 爪部角度：90°（GRIPPER_ANGLE_MAX） |
| 2    | 机械臂抬升              | 2s    | joint2=60°、joint3=90°            |
| 3    | 机械臂旋转至倾倒角度    | 1s    | joint4=150°                       |
| 4    | 爪部松开至0°            | 0.5s  | 爪部角度：0°（GRIPPER_ANGLE_MIN）  |
| 5    | 机械臂复位              | 2s    | 所有关节角度归0°                  |
| 6    | 爪部保持空闲状态        | 0s    | 爪部角度维持0°                    |

### 2.3 话题接口
| 话题名称                | 消息类型                  | 方向 | 说明                     |
|-------------------------|---------------------------|------|--------------------------|
| /arm_controller/command | std_msgs/Float64MultiArray | 发布 | 机械臂关节角度指令（5关节） |
| /gripper_controller/command | std_msgs/Float64      | 发布 | 爪部角度指令             |
| /arm_pour_status        | std_msgs/String           | 发布 | 节点当前运行状态         |

### 2.4 关键特性
1. **角度校验**：机械臂关节（0°~270°）、爪部（0°~90°）角度范围校验，超出范围触发错误处理；
2. **状态发布**：实时发布节点运行状态（如`step_1_gripper_close`、`sequence_completed`）；
3. **错误处理**：角度异常时自动复位机械臂至初始位置，打印错误日志并终止节点；
4. **时间戳打印**：终端输出带毫秒级时间戳的步骤执行日志，便于调试和过程追溯。

## 3. 模块注释
### 3.1 核心类：RestaurantArmPour
封装机械臂控制的所有逻辑，包含初始化、动作序列执行、指令发布、状态管理等核心方法。

| 方法名                | 功能说明                                                                 |
|-----------------------|--------------------------------------------------------------------------|
| RestaurantArmPour()   | 构造函数：初始化发布者、关节/爪部初始角度、发布初始状态（initialized）|
| ~RestaurantArmPour()  | 析构函数：释放动态分配资源（当前无额外资源）                             |
| executePourSequence() | 核心入口：按顺序执行6步动作序列，全程更新状态并发布                      |
| publishStatus()       | 发布当前节点状态到/arm_pour_status话题                                   |
| setJointAngles()      | 关节角度设置：校验→发布指令→更新缓存→打印日志→耗时等待                   |
| setGripperAngle()     | 爪部角度设置：校验→发布指令→更新缓存→打印日志→耗时等待                   |
| publishJointCommand() | 底层方法：封装关节角度话题发布逻辑                                       |
| publishGripperCommand() | 底层方法：封装爪部角度话题发布逻辑                                     |
| checkJointAngle()     | 单个关节角度范围校验                                                     |
| checkGripperAngle()   | 爪部角度范围校验                                                         |
| resetRobot()          | 复位逻辑：机械臂所有关节归0°、爪部归0°，并等待1s确保复位完成             |
| getTimestamp()        | 生成毫秒级时间戳（格式：YYYY-MM-DD HH:MM:SS.ms），用于日志打印           |
| printStepStatus()     | 重载方法：分别处理关节/爪部动作的终端日志打印，格式统一、信息清晰         |
| errorHandler()        | 错误处理：打印错误日志→复位机械臂→关闭节点→退出程序                     |

### 3.2 宏定义（赛事参数）
```cpp
#define ARM_JOINT_NUM    5       // 机械臂关节数量
#define ARM_ANGLE_MIN    0.0     // 机械臂单关节最小角度
#define ARM_ANGLE_MAX    270.0   // 机械臂单关节最大角度
#define GRIPPER_ANGLE_MIN 0.0    // 爪部最小角度
#define GRIPPER_ANGLE_MAX 90.0   // 爪部最大角度
```

### 3.3 脚本模块：check_arm_node.sh
| 脚本段                | 功能说明                                                                 |
|-----------------------|--------------------------------------------------------------------------|
| 节点存活检测          | 通过`rosnode list`检测`restaurant_arm_pour`节点是否运行                  |
| 关节话题检测          | 通过`rostopic echo`检测/arm_controller/command是否有消息输出             |
| 状态话题检测          | 读取/arm_pour_status话题内容，解析并打印当前节点状态                     |

### 3.4 构建配置：CMakeLists.txt
核心编译逻辑：
- 依赖`roscpp`、`sensor_msgs`、`std_msgs`；
- 编译`src/restaurant_arm_pour.cpp`生成可执行文件`restaurant_arm_pour`；
- 链接catkin库，确保ROS接口正常调用。

### 3.5 包配置：package.xml
声明功能包基础信息、维护者、许可证，以及编译/运行依赖（`catkin`、`roscpp`、`sensor_msgs`、`std_msgs`），符合ROS包规范。