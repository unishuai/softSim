#============================================
# @File    : cableExportTest0219.py
# @Date    : 2025-02-19 下午3:39
# @Author  : 帅宇昕
#============================================

import argparse

import hydra
import numpy as np
import pybullet as p
import pybullet_data
import time

import tacto

# v1
import physicsWorld.ur10FreeHandSim as mySim
from entities.contactPointInfo import ContactPointInfo
import logging
import pathlib

from utils.TactoBodyDataClass import TactoBodyDataClass

# 配置日志系统


if __name__ == "__main__":
    parentPath=pathlib.Path(__file__).parent.resolve()
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

    cableLen = 1

    # 原来的小圆柱高
    height = 0.05
    # height = 0.02
    # 原来的圆球个数，这个是固定的
    # ballNum = 45
    ballNum = int(cableLen / height)
    sphereRadius = 0.001
    preLinkId = -1
    baseAngle = [-90, 0, 0]
    basePose = [0.0, 0.0, 0.0]

    # #region todo:设置日志功能
    # logging.basicConfig(
    #     filename="constraintForce1114.log",
    #     filemode="w",  # 追加模式，如果改成 "w" 会覆盖文件
    #     level=logging.DEBUG,  # 设置最低日志级别
    #     # format="%(asctime)s - %(levelname)s - %(message)s",  # 日志格式
    #     format=f"totallen:{cableLen}-num:{ballNum}-%(message)s",
    #     datefmt="%Y-%m-%d %H:%M:%S",  # 日期格式
    # )
    # #获取日志对象
    # logger=logging.getLogger(__name__)
    # #然后是处理器
    # fileHander=logging.FileHandler("constraintForce1114.log", encoding="utf-8")
    # fileHander.setLevel(logging.INFO)
    # # 将处理器添加到日志
    # logger.addHandler(fileHander)
    # logger.info(time.asctime())
    #
    # #endregion

    for i in range(ballNum):
        # todo:揽线方式
        # colBallId = p.createCollisionShape(p.GEOM_CYLINDER, radius=sphereRadius, height=height)
        # visualShapeId = p.createVisualShape(p.GEOM_CYLINDER, radius=sphereRadius, length=height, rgbaColor=[1, 0, 0, 1])
        #
        # # 缆线的质量
        # mass = 0.01
        # # 可视化的数据id，-1就是默认颜色，不自定义可视化
        # # visualShapeId = -1
        # basePosition = [0 + basePose[0], i * (height) * 1.5 + basePose[1], 0.1 * height + basePose[2]]
        # baseOrientation = p.getQuaternionFromEuler(baseAngle)
        # linkId = p.createMultiBody(mass,
        #                            colBallId,
        #                            visualShapeId,
        #                            basePosition,
        #                            baseOrientation
        #                            )

        # todo:模型方式
        mass = 0.003934
        # 可视化的数据id，-1就是默认颜色，不自定义可视化
        # visualShapeId = -1
        basePosition = [0 + basePose[0], i * (height) * 1.5 + basePose[1], 0.1 * height + basePose[2]]
        baseOrientation = p.getQuaternionFromEuler(baseAngle)
        linkId=p.loadURDF(str(parentPath.joinpath("urdfs").joinpath("test"+str(i)+".urdf")), basePosition, baseOrientation)



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
    robot = mySim.Ur10FreeHnadSimAuto(p, [-0.8, 0, 0])

    # region todo:添加触觉传感器sensor
    linkIdBodyDatas=[]

    # 2️⃣ 加载配置
    cfg = None
    with hydra.initialize(config_path="../conf", version_base=None):  # 指定配置文件所在目录
        cfg = hydra.compose(config_name="bulletWorld")
    # 测试后确定可以读取文件
    # print(cfg)
    # tianjia chaunganqi
    digits = tacto.Sensor(**cfg.tacto)
    # tianjia keshihua de xiangji
    digits.add_camera(robot.robotId, cfg.digit_link_id_allegro)
    # tianjia bei jiankong de wuti

    # for i in range(len(linkIds)):
    #     basePosition = [0 + basePose[0], i * (height) * 1.5 + basePose[1], 0.1 * height + basePose[2]]
    #     baseOrientation = p.getQuaternionFromEuler(baseAngle)
    #
    #     linkIdBodyData = TactoBodyDataClass(str(parentPath.joinpath("urdfs").joinpath("test"+str(i)+".urdf")), np.array([0 + basePose[0], i * (height) * 1.5 + basePose[1], 0.1 * height + basePose[2]]), 1, linkIds[i])
    #     digits.add_body(linkIdBodyData)
    #     linkIdBodyDatas.append(linkIdBodyData)

    cupId=p.loadURDF("/home/unishuai/PycharmProjects/softSim/otherModel/urdf/cupTest.urdf", np.array([0.3, -0.6, 1]))
    linkIdBodyData = TactoBodyDataClass(str(parentPath.parent.joinpath("otherModel").joinpath("urdf").joinpath("cupTest" +".urdf")),np.array([0.3, -0.6, 1]),1, cupId)
    digits.add_body(linkIdBodyData)
    # endregion

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

        # if robot.step(objectPos, [-2.0, 3.14, -3.14], 0) is True:
        #     for i in range(50):
        #         p.stepSimulation()
        #         time.sleep(0.007)
        #     # mediaLinkState = p.getLinkState(ObjectId, int(objectNumJoint / 2))
        #     # objectPos, objectOrn = mediaLinkState[4, 5]
        #     objectPos, objectOrn = p.getBasePositionAndOrientation(ObjectId)
        #
        # contact_points.clear()


        #region todo:控制输出碰撞信息
        # for ballId in linkIds:
        #     contactPoints = p.getContactPoints(robot.robotId, ballId)
        #     for contactPoint in contactPoints:
        #         # print("接触点信息")
        #         # for i in contactPoints:
        #         #     print(i, end=", ")
        #         # print()
        #
        #         linkStateInfoA = p.getLinkState(contactPoint[1], contactPoint[3])
        #         linkStateInfoB = p.getBasePositionAndOrientation(contactPoint[2])
        #         # print(f"linkJointA:{linkStateInfoA[0]},linkJointB:{linkStateInfoB[0]}")
        #         contact_points.append(ContactPointInfo.createContactInfo(contactPoint, linkStateInfoA, linkStateInfoB))
        #
        # if contact_points is not None and len(contact_points) > 1:
        #     print(f"contact_points的大小为{len(contact_points)}")
        #
        #     for i in contact_points:
        #         print(i)
        #endregion

        # count=0
        #
        # if count%19000==0:
        #     count=0
        #     posList =[]
        #     for i in linkIds:
        #         posList.append(p.getBasePositionAndOrientation(i)[0])

            # logger.info(posList)

        # count += 1


        if cfg is not None:
            color, depth =digits.render()
            digits.updateGUI(color, depth)

        p.stepSimulation()
