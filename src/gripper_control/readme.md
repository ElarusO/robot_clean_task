# gripper_control 柔性机械爪ROS控制包
该功能包基于ROS实现柔性机械爪的角度控制，采用ROS Service（服务）通信模型，提供服务端与客户端交互接口，支持机械爪目标角度的校验、防抖控制与状态反馈。

## 1. 运行步骤
### 前提条件
- 已安装ROS（Kinetic/Melodic/Noetic等版本），并配置好工作空间（如`catkin_ws`）。
- 将`gripper_control`功能包放置到工作空间的`src`目录下。

### 编译功能包
```bash
# 进入工作空间根目录
cd ~/catkin_ws
# 编译功能包
catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release
# 刷新ROS环境
source devel/setup.bash
```
### launch一键启动:
```bash
roslaunch gripper_control gripper_all.launch 
```

### 启动服务端
打开终端，执行以下命令启动机械爪控制服务端：
```bash
rosrun gripper_control gripper_server.py
```
终端输出 `机械爪控制服务已启动，等待请求...` 表示服务端启动成功。

### 启动客户端
打开新终端，执行以下命令启动交互客户端：
```bash
rosrun gripper_control gripper_client.py
```
终端输出 `客户端已连接至 /gripper_control，请输入目标角度（0~90），输入 q 退出` 表示客户端启动成功，可按照提示输入目标角度进行控制。

### 交互示例
```bash
>>> 目标角度: 45

=== 响应信息 ===
执行结果: 成功
当前实际角度: 45.00°
说明: 执行成功，机械爪已转动至 45.0°

>>> 目标角度: 43

=== 响应信息 ===
执行结果: 失败
当前实际角度: 45.00°
说明: 角度差值过小：2.00° < 5.0°，无需动作

>>> 目标角度: q
```

## 2. 功能说明
### 核心功能
1. **角度范围校验**：限制机械爪目标角度在 `0°~90°` 范围内（0°为闭合，90°为完全张开），超出范围时返回失败并提示原因。
2. **防抖控制**：设置5°防抖阈值，当目标角度与当前实际角度差值小于5°时，拒绝执行动作，避免机械爪频繁小幅抖动。
3. **状态反馈**：客户端发送请求后，服务端返回执行结果（成功/失败）、当前实际角度、操作说明，便于用户确认控制状态。

### 通信接口
- 服务名称：`/gripper_control`
- 服务类型：`GripperControl`（自定义服务类型）
  - 请求字段：`gripper_angle`（float64）：目标角度。
  - 响应字段：
    - `result`（bool）：执行结果（true为成功，false为失败）。
    - `current_angle`（float64）：机械爪当前实际角度。
    - `info`（string）：操作说明（失败原因/成功提示）。

### 模块分工
- **服务端（gripper_server.py）**：封装机械爪控制逻辑，提供ROS服务接口，处理客户端的角度请求，完成校验、防抖、状态更新与响应返回。
- **客户端（gripper_client.py）**：提供交互式命令行界面，接收用户输入的目标角度，调用服务接口并打印响应结果。
- **CMakeLists.txt/package.xml**：配置功能包编译规则、依赖项（如rospy、std_msgs、message_generation等），生成自定义服务类型。

## 3. 模块注释
### 3.1 服务端（gripper_server.py）
```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from gripper_control.srv import GripperControl, GripperControlResponse

class GripperController:
    """机械爪控制逻辑封装：角度校验、防抖、状态保持"""
    def __init__(self, initial_angle=0.0):
        self.current_angle = initial_angle  # 当前实际角度，初始为0°（闭合）
        self.min_angle = 0.0                # 机械爪最小角度（闭合）
        self.max_angle = 90.0               # 机械爪最大角度（张开）
        self.dead_zone = 5.0                # 防抖阈值（差值＜5°拒绝执行）

    def handle_request(self, req):
        """服务回调函数：处理角度请求，返回响应
        Args:
            req: 服务请求对象，包含gripper_angle（目标角度）
        Returns:
            GripperControlResponse: 服务响应对象，包含result（执行结果）、current_angle（当前角度）、info（说明）
        """
        target = req.gripper_angle
        result = False
        info = ""

        # 1. 角度范围校验：超出0~90°范围则返回失败
        if target < self.min_angle or target > self.max_angle:
            info = f"角度非法：{target}° 不在 [{self.min_angle}°~{self.max_angle}°] 范围内"
            return GripperControlResponse(result, self.current_angle, info)

        # 2. 防抖校验：差值小于防抖阈值则返回失败
        if abs(target - self.current_angle) < self.dead_zone:
            info = f"角度差值过小：{abs(target - self.current_angle):.2f}° < {self.dead_zone}°，无需动作"
            return GripperControlResponse(result, self.current_angle, info)

        # 3. 合法执行：更新当前角度，返回成功
        self.current_angle = target
        result = True
        info = f"执行成功，机械爪已转动至 {self.current_angle}°"
        return GripperControlResponse(result, self.current_angle, info)

if __name__ == "__main__":
    rospy.init_node("gripper_server")          # 初始化ROS节点（名称：gripper_server）
    controller = GripperController()           # 实例化机械爪控制器
    # 创建ROS服务：名称/gripper_control，类型GripperControl，回调函数handle_request
    srv = rospy.Service("/gripper_control", GripperControl, controller.handle_request)
    rospy.loginfo("机械爪控制服务已启动，等待请求...")  # 打印启动日志
    rospy.spin()  # 保持节点运行，等待服务请求
```

### 3.2 客户端（gripper_client.py）
```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from gripper_control.srv import GripperControl

def gripper_client():
    """机械爪客户端逻辑：连接服务，接收用户输入，调用服务并打印响应"""
    rospy.wait_for_service("/gripper_control")  # 等待/gripper_control服务启动
    try:
        # 创建服务代理，绑定服务名称与服务类型
        proxy = rospy.ServiceProxy("/gripper_control", GripperControl)
        rospy.loginfo("客户端已连接至 /gripper_control，请输入目标角度（0~90），输入 q 退出")

        # 循环接收用户输入，直至节点关闭或输入退出指令
        while not rospy.is_shutdown():
            user_input = input(">>> 目标角度: ").strip()
            # 退出条件：输入q/quit/exit（不区分大小写）
            if user_input.lower() in ["q", "quit", "exit"]:
                break

            # 输入校验：确保为数字
            try:
                angle = float(user_input)
            except ValueError:
                print("输入错误：请输入数字或 q")
                continue

            # 调用服务，发送目标角度请求
            resp = proxy(angle)
            # 打印格式化的响应信息
            print("\n=== 响应信息 ===")
            print(f"执行结果: {'成功' if resp.result else '失败'}")
            print(f"当前实际角度: {resp.current_angle:.2f}°")
            print(f"说明: {resp.info}\n")

    except rospy.ServiceException as e:
        # 服务调用失败时打印错误日志
        rospy.logerr(f"服务调用失败: {e}")

if __name__ == "__main__":
    rospy.init_node("gripper_client")  # 初始化ROS节点（名称：gripper_client）
    gripper_client()                   # 执行客户端逻辑
```

### 3.3 编译配置（CMakeLists.txt 核心片段）
```cmake
cmake_minimum_required(VERSION 3.0.2)
project(gripper_control)

# 查找依赖包：rospy（Python ROS接口）、std_msgs（标准消息）、message_generation（消息生成）
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

# 添加自定义服务文件：GripperControl.srv
add_service_files(
  FILES
  GripperControl.srv
)

# 生成消息/服务，依赖std_msgs
generate_messages(
  DEPENDENCIES
  std_msgs
)

# 配置catkin包：依赖message_runtime（运行时消息支持）
catkin_package(
  CATKIN_DEPENDS message_runtime
)

# 包含目录：catkin依赖目录
include_directories(
  ${catkin_INCLUDE_DIRS}
)
```

### 3.4 包配置（package.xml 核心片段）
```xml
<!-- 构建工具依赖：catkin -->
<buildtool_depend>catkin</buildtool_depend>
<!-- 构建依赖：rospy、sensor_msgs、std_msgs、message_generation（消息生成） -->
<build_depend>rospy</build_depend>
<build_depend>sensor_msgs</build_depend>
<build_depend>std_msgs</build_depend>
<build_depend>message_generation</build_depend>
<!-- 构建导出依赖：供其他包编译时依赖 -->
<build_export_depend>rospy</build_export_depend>
<build_export_depend>sensor_msgs</build_export_depend>
<build_export_depend>std_msgs</build_export_depend>
<!-- 运行时依赖：rospy、sensor_msgs、message_runtime（运行时消息）、std_msgs -->
<exec_depend>rospy</exec_depend>
<exec_depend>sensor_msgs</exec_depend>
<exec_depend>message_runtime</exec_depend>
<exec_depend>std_msgs</exec_depend>
```