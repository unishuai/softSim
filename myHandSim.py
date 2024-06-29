# ============================================
# @File    : myHandSim.py
# @Date    : 2024-06-27 14:48
# @Author  : 帅宇昕
# ============================================
import numpy as np
import pybullet as p
import pybullet_data
import time

# 连接物理服务器
p.connect(p.GUI)
#添加路径
p.setAdditionalSearchPath(pybullet_data.getDataPath())
fps=240.
timeStep = 1./fps
# 设置仿真步长
p.setTimeStep(timeStep)
# 设置重力
p.setGravity(0, 0, -9.8)

# 加载地面
p.loadURDF("plane.urdf")
robot = p.loadURDF(r"freehand_description/urdf/freehand_description.urdf",
                   )



for i in range(100000):
    # panda.bullet_client.submitProfileTiming("full_step")

    p.stepSimulation()
    time.sleep(timeStep)