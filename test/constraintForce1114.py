# ============================================
# @Project  : softSim
# @File     : constraintForce1114.py
# @Date     : 2024-11-14 21:11
# @Author   : unishuai
# @email    : unishuai@gmail.com
# @des      : function description
# ============================================
import argparse

import pybullet as p
import pybullet_data
import time
# v1
import physicsWorld.ur10FreeHandSim as mySim
from entities.contactPointInfo import ContactPointInfo

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
    sphereRadius = 0.01
    preLinkId = -1
    baseAngle = [-90, 0, 0]
    basePose = [0.0, 0.0, 0.0]
    for i in range(ballNum):
        colBallId = p.createCollisionShape(p.GEOM_CYLINDER, radius=sphereRadius, height=height)
        visualShapeId = p.createVisualShape(p.GEOM_CYLINDER, radius=sphereRadius, length=height, rgbaColor=[1, 0, 0, 1])

        # 缆线的质量
        mass = 0.01
        # 可视化的数据id，-1就是默认颜色，不自定义可视化
        # visualShapeId = -1
        basePosition = [0 + basePose[0], i * (height) * 1.2 + basePose[1], 0.1 * height + basePose[2]]
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
    robot = mySim.Ur10FreeHnadSimAuto(p, [0, 0, 0])
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
    print(f"objectNumJoint:{objectNumJoint},mediaLinkState--:{mediaLinkState}")

    # objectPos, objectOrn = mediaLinkState[4]
    # print(f"LEGO模型当前的位置: {objectPos}, {objectOrn}")
    objectPos, objectOrn = p.getBasePositionAndOrientation(ObjectId)

    contact_points = []
    for i in range(int(1e18)):

        time.sleep(0.007)

        if robot.step(objectPos, [-2.0, 3.14, -3.14], 0) is True:
            for i in range(50):
                p.stepSimulation()
                time.sleep(0.007)
            # mediaLinkState = p.getLinkState(ObjectId, int(objectNumJoint / 2))
            # objectPos, objectOrn = mediaLinkState[4, 5]
            objectPos, objectOrn = p.getBasePositionAndOrientation(ObjectId)

        contact_points.clear()
        count=0
        for ballId in linkIds:
            contactPoints=p.getContactPoints(robot.robotId,ballId)
            for contactPoint in contactPoints:
                # print("接触点信息")
                # for i in contactPoints:
                #     print(i, end=", ")
                # print()

                linkStateInfoA=p.getLinkState(contactPoint[1],contactPoint[3])
                linkStateInfoB=p.getBasePositionAndOrientation(contactPoint[2])
                # print(f"linkJointA:{linkStateInfoA[0]},linkJointB:{linkStateInfoB[0]}")
                contact_points.append(ContactPointInfo.createContactInfo(contactPoint, linkStateInfoA, linkStateInfoB))

                # contact_points.extend\)

        if contact_points is not None and len(contact_points) > 1:
            print(f"contact_points的大小为{len(contact_points)}")

            for i in contact_points:
                print(i)


        p.stepSimulation()
