# ============================================
# @File    : myHandSim.py
# @Date    : 2024-06-27 14:48
# @Author  : 帅宇昕
# @email    : unishuai@gmail.com
# @des      : 机械臂测试代码
# ============================================
from collections import namedtuple

import numpy as np
import pybullet as p
import pybullet_data
import time


def moveArm(action, myMaxVelocity=-1):
    joint_poses = action
    if myMaxVelocity < 0:
        for i, joint_id in enumerate(arm_controllable_joints):
            p.setJointMotorControl2(robotId, joint_id, p.POSITION_CONTROL,
                                    joint_poses[i],
                                    force=joints[joint_id].maxForce,
                                    maxVelocity=joints[joint_id].maxVelocity)
    else:
        for i, joint_id in enumerate(arm_controllable_joints):
            p.setJointMotorControl2(robotId, joint_id, p.POSITION_CONTROL,
                                    joint_poses[i],
                                    force=joints[joint_id].maxForce,
                                    maxVelocity=myMaxVelocity)




# 连接物理服务器
p.connect(p.GUI)
# 添加路径
p.setAdditionalSearchPath(pybullet_data.getDataPath())
fps = 240.
timeStep = 1. / fps
# 设置仿真步长
p.setTimeStep(timeStep)
# 设置重力
p.setGravity(0, 0, -9.8)

# 机械臂的自由度
arm_num_dofs = 6
# 机械臂的初始位置
arm_rest_poses = [-1.5690622952052096, -1.5446774605904932, 1.343946009733127, -1.3708613585093699, -1.5707970583733368,
                  0.0009377758247187636]

# todo:读取urdf 文件
# 加载地面
p.loadURDF("plane.urdf")
robotId = p.loadURDF(r"../freehand_description/urdf/freehand_description.urdf", basePosition=[0, 0, 0], useFixedBase=1)
if robotId < 0:
    # 还是这种抛出异常这样子写的爽一些
    raise ValueError("文件读取异常,检查一下是不是路径或者其他的问题")

# todo:获取相关信息
# 映射，输出方便看
jointTypeMapping = {getattr(p, name): name for name in dir(p) if name.startswith('JOINT_')}
numJoints = p.getNumJoints(robotId)
# 简单的一个java 类
jointInfo = namedtuple('jointInfo',
                       ['id', 'name', 'type', 'damping', 'friction', 'lowerLimit', 'upperLimit', 'maxForce',
                        'maxVelocity', 'controllable'])
# 这个记录每个关节信息
joints = []
# 这个记录可以控制的关节信息
controllable_joints = []
for i in range(numJoints):
    info = p.getJointInfo(robotId, i)
    jointID = info[0]
    jointName = info[1].decode("utf-8")
    jointType = info[2]  # JOINT_REVOLUTE, JOINT_PRISMATIC, JOINT_SPHERICAL, JOINT_PLANAR, JOINT_FIXED
    jointDamping = info[6]
    jointFriction = info[7]
    jointLowerLimit = info[8]
    jointUpperLimit = info[9]
    jointMaxForce = info[10]
    jointMaxVelocity = info[11]
    controllable = (jointType != p.JOINT_FIXED)
    if controllable:
        # 打印输出
        # print("jointID:{0} ,jointName:{1} ,jointType:{2}".format(info[0],info[1].decode('utf-8'),jointTypeMapping[info[2]]))
        # 添加可以控制关节的信息
        controllable_joints.append(jointID)

        # # 让所有可以控制的关节速度都为0，不让其动
        #deprecated 之前那个代码真的是垃圾代码，不需要有这个
        # p.setJointMotorControl2(robotId, jointID, p.VELOCITY_CONTROL,
        #                         targetVelocity=0, force=0)
    # 添加进这个类似java的类里面
    info = jointInfo(jointID, jointName, jointType, jointDamping, jointFriction, jointLowerLimit, jointUpperLimit,
                     jointMaxForce, jointMaxVelocity, controllable)
    # 然后把所有信息存储起来
    joints.append(info)

assert len(controllable_joints) > arm_num_dofs, "自由度低于6，应该是没有读取到机械臂了,controllable_joints={0},arm_num_dofs{1}".format(len(controllable_joints),arm_num_dofs)

# 单独获取对手臂控制的关节
arm_controllable_joints = controllable_joints[:arm_num_dofs]

# 获取手臂的下限
arm_lower_limits = [info.lowerLimit for info in joints if info.controllable][:arm_num_dofs]
# 获取手臂的上限
arm_upper_limits = [info.upperLimit for info in joints if info.controllable][:arm_num_dofs]
# 获取手臂的移动范围
arm_joint_ranges = [info.upperLimit - info.lowerLimit for info in joints if info.controllable][:arm_num_dofs]

# todo:初始化位姿，后续就应该是编写相应的指抓函数了
moveArm(arm_rest_poses)


# for i in range(numJoints+1):
#     print(f"LinkState{i}",p.getLinkState(robotId,i))


for i in range(100000):
    # panda.bullet_client.submitProfileTiming("full_step")

    p.stepSimulation()
    time.sleep(timeStep)
