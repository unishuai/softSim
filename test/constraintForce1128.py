# ============================================
# @Project  : softSim
# @File     : constraintForce1114.py
# @Date     : 2024-11-14 21:11
# @Author   : unishuai
# @email    : unishuai@gmail.com
# @des      : 想直接使用指抓抓取的测试程序（单独导入二指抓碰不到）
# ============================================
import argparse

import pybullet as p
import pybullet_data
import time
# v1
import utils.twpgripper as mySim
from entities.contactPointInfo import ContactPointInfo
import logging



# 配置日志系统


if __name__ == "__main__":
    # 创建一个解析器对象，并添加描述
    parser = argparse.ArgumentParser(description='test demo')
    parser.add_argument('--object', choices=['lego', 'husky', "block"], type=str, default='block',help='choose the grasped object')
    args = parser.parse_args()

    physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version

    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
    # print(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    planeId = p.loadURDF("plane.urdf", basePosition=[0, 0, 0])
    objectType: str = args.object

    #region todo:添加缆线
    # todo: 调整相机视角
    distance = 1
    yaw_angle = 45
    pitch_angle = -45
    target_position = [0, 0, 0]
    p.resetDebugVisualizerCamera(cameraDistance=distance,
                                 cameraYaw=yaw_angle,
                                 cameraPitch=pitch_angle,
                                 cameraTargetPosition=target_position)

    # colBallId = p.createCollisionShapeArray([p.GEOM_BOX, p.GEOM_SPHERE],radii=[sphereRadius+0.03,sphereRadius+0.03], halfExtents=[[sphereRadius,sphereRadius,sphereRadius],[sphereRadius,sphereRadius,sphereRadius]])
    # 修改为一个球体

    linkIds = list()

    cableLen = 0.9

    # 原来的小圆柱高
    height = 0.1
    # height = 0.02
    # 原来的圆球个数，这个是固定的
    # ballNum = 45
    ballNum = int(cableLen / height)
    sphereRadius = 0.001
    preLinkId = -1
    baseAngle = [-90, 0, 0]
    basePose = [0.0, 0.0, 0.0]
    #region todo:设置日志功能
    logging.basicConfig(
        filename="constraintForce1128.log",
        filemode="w",  # 追加模式，如果改成 "w" 会覆盖文件
        level=logging.DEBUG,  # 设置最低日志级别
        # format="%(asctime)s - %(levelname)s - %(message)s",  # 日志格式
        format=f"totallen:{cableLen}-num:{ballNum}-%(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",  # 日期格式
    )
    #获取日志对象
    logger=logging.getLogger(__name__)
    #然后是处理器
    fileHander=logging.FileHandler("constraintForce1128.log", encoding="utf-8")
    fileHander.setLevel(logging.INFO)
    # 将处理器添加到日志
    logger.addHandler(fileHander)
    logger.info(time.asctime())

    #endregion

    for i in range(ballNum):
        colBallId = p.createCollisionShape(p.GEOM_CYLINDER, radius=sphereRadius, height=height)
        visualShapeId = p.createVisualShape(p.GEOM_CYLINDER, radius=sphereRadius, length=height, rgbaColor=[1, 0, 0, 1])

        # 缆线的质量
        mass = 0.01
        # 可视化的数据id，-1就是默认颜色，不自定义可视化
        # visualShapeId = -1
        basePosition = [0 + basePose[0], i * (height) * 1.5 + basePose[1], 0.1 * height + basePose[2]]
        baseOrientation = p.getQuaternionFromEuler(baseAngle)
        linkId = p.createMultiBody(mass,
                                   colBallId,
                                   visualShapeId,
                                   basePosition,
                                   baseOrientation
                                   )
        linkIds.append(linkId)
        if preLinkId != -1:
            constraint_id = p.createConstraint(
                parentBodyUniqueId=preLinkId,
                parentLinkIndex=-1,
                childBodyUniqueId=linkId,
                childLinkIndex=-1,
                jointType=p.JOINT_FIXED,
                jointAxis=[0, 0, 0],
                parentFramePosition=[0, 0, height / 2],
                childFramePosition=(0, 0, -height / 2),

            )

            # 设置约束的最大力

            p.changeConstraint(constraint_id, maxForce=10000000, erp=0.95)
        preLinkId = linkId

    # 固定第一段缆线到世界中的固定点
    fixed_constraint = p.createConstraint(
        parentBodyUniqueId=linkIds[0],
        parentLinkIndex=-1,
        childBodyUniqueId=-1,  # -1 表示世界
        childLinkIndex=-1,
        jointType=p.JOINT_POINT2POINT,
        jointAxis=[0, 0, 0],  # 固定关节不需要轴
        parentFramePosition=[0, 0, -height / 2],  # 在缆线局部坐标的固定点
        childFramePosition=[0,0,0.5]  # 世界坐标的固定点
    )

    p.setGravity(0, 0, -10)
    # p.setRealTimeSimulation(0)

    for i in linkIds:
        info = p.getDynamicsInfo(i, -1)

        p.changeDynamics(i, -1,
                         lateralFriction=0.7,
                         linearDamping=0.05,
                         angularDamping=0.05,
                         rollingFriction=0.01,
                         )

    #endregion


    ObjectId: int = linkIds[len(linkIds)//2]
    robot = mySim.GripperSimAuto(p, [0, 0, 0.5])
    # for i in [37,38,39,40]:
    #     dynamics_info = p.getDynamicsInfo(robot.robotId, i)
    #     print(f"id={i},dynamics_info:{dynamics_info}")

    # # fps = 240.
    # # timeStep = 1. / fps
    # # 设置仿真步长
    # p.setTimeStep(robot.control_dt)
    # # 设置重力
    # p.setGravity(0, 0, -9.8)
    #

    objectNumJoint = p.getNumJoints(ObjectId)
    mediaLinkState = p.getLinkState(ObjectId, 2)
    # print(f"objectNumJoint:{objectNumJoint},mediaLinkState--:{mediaLinkState}")

    # objectPos, objectOrn = mediaLinkState[4]
    # print(f"LEGO模型当前的位置: {objectPos}, {objectOrn}")
    objectPos, objectOrn = p.getBasePositionAndOrientation(ObjectId)

    contact_points = []
    for i in range(int(1e18)):

        time.sleep(0.007)
        count=0
        if count%19000==0:
            count=0
            posList =[]
            for i in linkIds:
                posList.append(p.getBasePositionAndOrientation(i)[0])

            logger.info(posList)

        count += 1

        p.stepSimulation()
