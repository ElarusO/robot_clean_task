#!/bin/bash
# check_arm_node.sh

# 1. 检测节点是否存活
if rosnode list | grep -q "restaurant_arm_pour"; then
    echo "[OK] 节点正在运行"
else
    echo "[ERROR] 节点未运行"
    exit 1
fi

# 2. 检测关节话题是否有输出
if rostopic echo /arm_controller/command -n 1 > /dev/null 2>&1; then
    echo "[OK] 关节指令话题正常"
else
    echo "[WARN] 关节指令话题无输出"
fi

# 3. 检测自定义状态话题（若添加了进阶方案）
if rostopic echo /arm_pour_status -n 1 > /dev/null 2>&1; then
    STATUS=$(rostopic echo /arm_pour_status -n 1 | grep "data:" | awk -F'"' '{print $2}')
    echo "[OK] 节点状态：$STATUS"
else
    echo "[WARN] 状态话题无输出"
fi
