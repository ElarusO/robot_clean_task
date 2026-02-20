#!/usr/bin/env python3
"""
gripper 服务端启动节点
功能：调用 gripper_tools.start_server() 启动爪部控制服务
"""
import os
import sys
# 将当前脚本所在目录加入 Python 路径，以便导入 gripper_tools
sys.path.insert(0, os.path.dirname(__file__))

import gripper_tools

if __name__ == "__main__":
    print("启动 gripper 服务端...")
    gripper_tools.start_server()   # 该函数会阻塞，直到节点关闭
