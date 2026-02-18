# 餐厅3D清洁赛项任务逻辑包

本ROS功能包实现餐厅赛项完整清洁任务流程，包含餐具清理及收纳（2处）和餐余垃圾清扫（2处），集成关节越界保护与动作超时保护，支持阶段单独控制。

## 文件结构
- `arm_control.py`：机械臂关节控制（发布角度到 `/arm_joint_angles`）
- `gripper_control.py`：爪部控制（调用 `gripper_tools.send_command`）
- `error_handle.py`：异常处理（越界检查、超时监控）
- `task_sequence.py`：主任务编排（调用上述模块）
- `README.md`：运行说明

## 依赖
- ROS（已测试于Noetic）
- 提供的 `gripper_tools` 库
- 提供的 `arm_joint_control` 库（可选，用于监控）

## 运行步骤
1. **启动爪部服务端**（单独终端）
   ```bash
   python -c "import gripper_tools; gripper_tools.start_server()"
   
   （可选）启动关节状态订阅，用于观察关节运动
   python -c "from arm_joint_control import start_subscriber; start_subscriber()"

   运行主任务节点

   python task_sequence.py