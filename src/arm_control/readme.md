# arm_control：5轴机械臂关节基础控制ROS功能包
该功能包实现了5轴机械臂关节角度的发布与订阅功能，基于ROS框架提供稳定的关节状态通信能力，支持交互式修改关节角度、实时监控关节运动状态。

## 1. 运行步骤
### 1.1 环境依赖
- ROS Kinetic/Melodic/Noetic（Python3环境）
- 依赖ROS包：`rospy`、`sensor_msgs`、`std_msgs`、`catkin`

### 1.2 编译功能包
1. 将功能包放入ROS工作空间的`src`目录下（如`catkin_ws/src`）
2. 编译工作空间：
   ```bash
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

### 1.3 启动节点

#### launch一键启动:
```bash
roslaunch arm_control arm_all.launch
```

#### 方式1：分别启动发布节点和订阅节点（推荐）
- 终端1：启动关节角度发布节点
  ```bash
  rosrun arm_control arm_joint_publisher.py
  ```
- 终端2：启动关节状态订阅节点
  ```bash
  rosrun arm_control arm_state_subscriber.py
  ```

#### 方式2：使用ROS核心（可选，多节点协调）
先启动ROS Master：
```bash
roscore
```
再分别在不同终端执行上述`rosrun`命令。

### 1.4 交互操作（发布节点）
发布节点启动后支持以下指令：
- 输入`update`：进入交互式修改关节角度流程，按提示逐个输入5个关节的新角度（范围0-270°）
- 输入`exit`：停止发布节点并退出
- 其他输入：提示无效指令

## 2. 功能说明
### 2.1 核心功能
| 模块 | 文件名 | 核心功能 |
|------|--------|----------|
| 关节发布节点 | `arm_joint_publisher.py` | 1Hz持续发布5轴机械臂关节角度；支持交互式修改角度；独立线程处理用户输入，不阻塞发布流程；初始角度为0°、30°、60°、90°、120° |
| 关节订阅节点 | `arm_state_subscriber.py` | 订阅关节角度话题；实时计算并显示每个关节的角度差值；检测消息丢包；标注关节运动状态（静止/正向运动/反向运动） |

### 2.2 通信机制
- 话题名称：`/arm_joint_angles`
- 消息类型：`sensor_msgs/JointState`（包含关节名称、角度（弧度）、时间戳）
- 发布频率：严格1Hz，保证关节状态更新稳定性
- 线程安全：使用`threading.Lock`防止角度读写冲突

### 2.3 数据处理
- 角度转换：支持角度（°）与弧度（rad）双向转换，适配ROS消息格式
- 角度校验：限制关节角度范围为0-270°，输入无效角度时给出警告
- 状态判断：角度差值绝对值<0.1°视为静止，否则判断正向/反向运动

## 3. 模块注释
### 3.1 package.xml
ROS功能包配置文件，声明以下核心信息：
- 包名/版本/描述：`arm_control` v0.1.0，5轴机械臂关节基础控制包
- 许可证：BSD
- 构建依赖：`catkin`（构建工具）、`rospy`/`sensor_msgs`/`std_msgs`（编译依赖）
- 运行依赖：`rospy`/`sensor_msgs`/`std_msgs`（运行时依赖）
- 导出配置：预留第三方工具扩展接口

### 3.2 CMakeLists.txt
ROS编译配置文件，核心配置：
- 最低CMake版本：3.0.2
- 依赖包：`rospy`、`sensor_msgs`、`std_msgs`
- 安装配置：将Python脚本（`arm_joint_publisher.py`/`arm_state_subscriber.py`）安装到ROS二进制目录，支持`rosrun`调用
- 未启用功能：消息生成、动态参数配置、C++编译、测试（预留扩展接口）

### 3.3 arm_joint_publisher.py
| 类/函数 | 功能注释 |
|---------|----------|
| `ArmJointPublisher` | 关节角度发布核心类，封装节点初始化、发布、交互逻辑 |
| `__init__` | 初始化ROS节点/发布者/关节初始角度/线程锁/发布频率，打印启动信息 |
| `deg_to_rad` | 角度转弧度，适配ROS消息的弧度格式要求 |
| `rad_to_deg` | 弧度转角度，方便用户交互时的角度显示 |
| `validate_angle` | 校验角度是否在0-270°范围内，返回布尔值并打印警告 |
| `update_angles_interactive` | 独立线程函数，处理用户输入（update/exit），实现角度交互式修改 |
| `publish_loop` | 主线程函数，1Hz持续发布关节状态，更新时间戳，保证线程安全 |

### 3.4 arm_state_subscriber.py
| 类/函数 | 功能注释 |
|---------|----------|
| `ArmStateSubscriber` | 关节状态订阅核心类，封装节点初始化、回调、运行逻辑 |
| `__init__` | 初始化ROS节点/订阅者/历史角度缓存/消息计数器，打印启动信息 |
| `rad_to_deg` | 弧度转角度，将ROS消息中的弧度转换为易读的角度值 |
| `joint_state_callback` | 话题回调函数，处理关节状态消息：<br>1. 检测消息丢包（seq序列校验）<br>2. 清屏并格式化输出关节状态<br>3. 校验数据完整性（5个关节）<br>4. 计算角度差值并判断运动状态<br>5. 更新历史角度缓存 |
| `run` | 启动节点自旋，持续监听话题消息 |