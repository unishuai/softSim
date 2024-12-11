# ============================================
# @Project  : softSim
# @File     : constraintForce1114.py
# @Date     : 2024-11-14 21:11
# @Author   : unishuai
# @email    : unishuai@gmail.com
# @des      : 测试缆线的位置是否符合要求的最终程序
# ============================================
import argparse

import pybullet as p
import pybullet_data
import time
# v1
import utils.ur10_sim_grasp_arm as mySim
import logging



# 配置日志系统
from entities.contactPointInfo import ContactPointInfo

if __name__ == "__main__":
    # 创建一个解析器对象，并添加描述
    parser = argparse.ArgumentParser(description='test demo')
    parser.add_argument('--object', choices=['lego', 'husky', "block"], type=str, default='block',help='choose the grasped object')
    args = parser.parse_args()

    physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version

    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
    # print(pybullet_data.getDataPath())

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
    sphereRadius = 0.0015
    preLinkId = -1
    baseAngle = [-90, 0, 0]
    basePose = [0.0, 0.0, 0.0]
    #region todo:设置日志功能
    logging.basicConfig(
        filename="constraintForce11280_2.log",
        filemode="w",  # 追加模式，如果改成 "w" 会覆盖文件
        level=logging.DEBUG,  # 设置最低日志级别
        # format="%(asctime)s - %(levelname)s - %(message)s",  # 日志格式
        format=f"totallen:{cableLen}-num:{ballNum}-%(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",  # 日期格式
    )
    #todo:位置的日志获取
    #获取日志对象
    positionlogger=logging.getLogger("positionLogger")
    #然后是处理器
    positionHandler=logging.FileHandler("constraintForce11280_2.log", encoding="utf-8")
    positionHandler.setLevel(logging.INFO)
    # 将处理器添加到日志
    positionlogger.addHandler(positionHandler)
    positionlogger.info(time.asctime())

    #todo:受力的日志获取
    # #获取日志对象
    # forcelogger = logging.getLogger("forceLogger")
    # # 然后是处理器
    #
    # forceHandler = logging.FileHandler("forceLogger.log", encoding="utf-8")
    # formatter = logging.Formatter('%(message)s')
    # forceHandler.setFormatter(formatter)
    # forceHandler.setLevel(logging.INFO)
    # # 将处理器添加到日志
    # forcelogger .addHandler(forceHandler)
    # forcelogger .info(time.asctime())
    #endregion

    for i in range(ballNum):
        colBallId = p.createCollisionShape(p.GEOM_CYLINDER, radius=sphereRadius, height=height)
        visualShapeId = p.createVisualShape(p.GEOM_CYLINDER, radius=sphereRadius, length=height, rgbaColor=[1, 0, 0, 1])

        # 缆线的质量
        mass = 0.003934
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
            # point= p.createConstraint(
            #     parentBodyUniqueId=preLinkId,
            #     parentLinkIndex=-1,
            #     childBodyUniqueId=linkId,
            #     childLinkIndex=-1,
            #     jointType=p.JOINT_POINT2POINT,
            #     jointAxis=[0, 0, 0],
            #     parentFramePosition=[0, 0, height / 2],
            #     childFramePosition=(0, 0, -height / 2),
            #
            # )

            # 设置约束的最大力

            p.changeConstraint(constraint_id, maxForce=1000,erp=0.9)
        preLinkId = linkId

    # ObjectId: int = preLinkId
    ObjectId: int = linkIds[len(linkIds)-2]

    # 固定第一段缆线到世界中的固定点
    fixed_constraint = p.createConstraint(
        parentBodyUniqueId=linkIds[0],
        parentLinkIndex=-1,
        childBodyUniqueId=-1,  # -1 表示世界
        childLinkIndex=-1,
        jointType=p.JOINT_POINT2POINT,
        jointAxis=[0, 0, 0],  # 固定关节不需要轴
        parentFramePosition=[0, 0, -height / 2],  # 在缆线局部坐标的固定点
        childFramePosition=[-0.13, 0.497 ,0.412]  # 世界坐标的固定点
    )
    tmp_constraint = p.createConstraint(
        parentBodyUniqueId=linkIds[len(linkIds)-2],
        parentLinkIndex=-1,
        childBodyUniqueId=-1,  # -1 表示世界
        childLinkIndex=-1,
        jointType=p.JOINT_POINT2POINT,
        jointAxis=[0, 0, 0],  # 固定关节不需要轴
        parentFramePosition=[0, 0, -height / 2],  # 在缆线局部坐标的固定点
        childFramePosition=[0.513, 0.023, 0.2]  # 世界坐标的固定点
    )


    p.setGravity(0, 0, -9.8)
    # p.setRealTimeSimulation(0)

    for i in linkIds:
        # info = p.getDynamicsInfo(i, -1)

        p.changeDynamics(i, -1,
                         lateralFriction=0.7,
                         linearDamping=0.30,
                         angularDamping=0.30,
                         rollingFriction=0.01,
                         )

    #endregion


    # ObjectId: int = linkIds[len(linkIds)//2]

    robot = mySim.PadnaSimAuto(p, [0.3, -0.4, 0])
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

    # objectNumJoint = p.getNumJoints(ObjectId)
    # mediaLinkState = p.getLinkState(ObjectId, 2)



    # print(f"LEGO模型当前的位置: {objectPos}, {objectOrn}")
    # objectPos, objectOrn = p.getBasePositionAndOrientation(ObjectId)



    objectPos, objectOrn = None, None

    contact_points = []
    firstenter=True
    count = 0
    # Increase the time step and solver iterations for better accuracy
    p.setTimeStep(1 / 240)  # Decrease time step for better resolution

    print(f"linkIds:{linkIds}")
    for i in range(int(1e18)):

        time.sleep(0.010)

        if firstenter:
            for i in range(600):
                time.sleep(0.010)
                p.stepSimulation()
            objectPos, objectOrn = p.getBasePositionAndOrientation(ObjectId)
            p.removeConstraint(tmp_constraint)
            firstenter=False

        if robot.step([0.513, 0.023, 0], 0.8, 0.04) is True:
            # for i in range(10):
            #     p.stepSimulation()
            #     time.sleep(0.007)
            # mediaLinkState = p.getLinkState(ObjectId, int(objectNumJoint / 2))
            # objectPos, objectOrn = mediaLinkState[4, 5]
            objectPos, objectOrn = p.getBasePositionAndOrientation(ObjectId)




        if count%50==0:
            count=0
            #todo:这里是输出每一个点的空间坐标的
            posList =[]

            posList.append(p.getBasePositionAndOrientation(linkIds[2])[0])
            posList.append(p.getBasePositionAndOrientation(linkIds[5])[0])
            posList.append(p.getBasePositionAndOrientation(linkIds[8])[0])
            posList.append(p.getBasePositionAndOrientation(linkIds[11])[0])
            posList.append(p.getBasePositionAndOrientation(linkIds[14])[0])

            posList.append(p.getBasePositionAndOrientation(linkIds[18])[0])
            posList.append(p.getBasePositionAndOrientation(linkIds[19])[0])
            positionlogger.info(posList)

            # region todo:控制输出碰撞信息
            contactPoints=[]
            contact_points=[]
            for ballId in range(len(linkIds)-1):
                contactPoints = p.getContactPoints(robot.id,linkIds[ballId] )
            for contactPoint in contactPoints:


                linkStateInfoA = p.getLinkState(contactPoint[1], contactPoint[3])
                linkStateInfoB = p.getBasePositionAndOrientation(contactPoint[2])
                # print(f"linkJointA:{linkStateInfoA[0]},linkJointB:{linkStateInfoB[0]}")
                contact_points.append(ContactPointInfo.createContactInfo(contactPoint, linkStateInfoA, linkStateInfoB))

            if contact_points is not None and len(contact_points) >= 1:
                # print(f"contact_points的大小为{len(contact_points)}")

                for i in contact_points:
                    # forcelogger.info(i)
                    # forcelogger.info("---------------\n")
                    # with open('forceLogger.txt', 'w') as file:
                    #     file.write(f"Object details: Name = {my_object.name}, Value = {my_object.value}\n")
                    # print(i)
                    # print()
                    positionlogger.info(i)

            # endregion

        count += 1
        # p.setTimeStep(1 / 240)
        # start_time = time.time()
        # for i in range(240):
        #
        # current_time = time.time()
        # print(current_time-start_time)
        p.stepSimulation()
