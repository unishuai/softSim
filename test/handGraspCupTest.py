# ============================================
# @Project  : softSim
# @File     : constraintForce1114.py
# @Date     : 2024-11-14 21:11
# @Author   : unishuai
# @email    : unishuai@gmail.com
# @des      : 机械臂抓取水杯测试（使用捏的方式）
# ============================================
import argparse
import random

import numpy as np
import pybullet as p
import pybullet_data
import time
# v1
import physicsWorld.ur10FreeHandSim as mySim
# import ur10FreeHandSimMultiEfforts as mySim







if __name__ == "__main__":
    # 创建一个解析器对象，并添加描述
    parser = argparse.ArgumentParser(description='test demo')
    parser.add_argument('--object',choices=['lego','husky',"block"],type=str,default='block',help='choose the grasped object')
    args=parser.parse_args()


    physicsClient = p.connect(p.GUI) # or p.DIRECT for non-graphical version

    p.setAdditionalSearchPath(pybullet_data.getDataPath()) # optionally
    # print(pybullet_data.getDataPath())
    p.setGravity(0,0,-9.8)
    planeId = p.loadURDF("plane.urdf",basePosition=[0, 0, 0])
    objectType:str=args.object
    ObjectId=0
    # if(objectType=='lego'):
    #
    #     ObjectId=p.loadURDF("lego/lego.urdf", np.array([random.uniform(-0.6, 0.6), random.uniform(-0.5, -0.1), 0]))
    # elif objectType=='husky':
    #     ObjectId=p.loadURDF("husky/husky.urdf", np.array([random.uniform(-0.6, 0.6), random.uniform(-0.5, -0.1), 0]))
    # elif objectType=='block':
    #     ObjectId=p.loadURDF("block.urdf", np.array([random.uniform(-0.6, 0.6), random.uniform(-0.5, -0.1), 0]),globalScaling=1.5)
    # robot=mySim.Ur10FreeHnadSimAuto(p,[0,0,0])
    # ObjectId = p.loadURDF("block.urdf", np.array([0.3, -0.6, 0.005]),globalScaling=1.5)
    ObjectId = p.loadURDF("../otherModel/urdf/coffee_cup.urdf", np.array([0.3, -0.6, 0]), globalScaling=1)
    robot=mySim.Ur10FreeHnadSimAuto(p,[0,0,0])



    # # fps = 240.
    # # timeStep = 1. / fps
    # # 设置仿真步长
    # p.setTimeStep(robot.control_dt)
    # # 设置重力
    # p.setGravity(0, 0, -9.8)
    #
    objectPos, objectOrn = p.getBasePositionAndOrientation(ObjectId)
    for i in range(10000):

        time.sleep(0.007)
        # robot.step([0.3, -.4,0.2],0.085,5)
        # 获取LEGO模型的当前位置和方向


        # print(f"LEGO模型当前的位置: {legoPos}")
        # print(f"LEGO模型当前的方向: {legoOrn}")
        # if robot.step(objectPos, [-1.57,3.14,-3.14], 5) is True:
        # if robot.step(objectPos, [-2.0,3.14,-3.14], 0) is True:
        #     for i in range(50):
        #         p.stepSimulation()
        #         time.sleep(0.007)
        #     objectPos, objectOrn = p.getBasePositionAndOrientation(ObjectId)


        contact_points = p.getContactPoints(robot.robotId, ObjectId)
        if contact_points is not None and len(contact_points)>1:
            print(f"contact_points的大小为{len(contact_points)}")
        for point in contact_points:
            normal_force = point[9]  # 正压力（法向力）
            lateral_friction1 = point[10]  # 第一个侧向摩擦力
            lateral_friction2 = point[12]  # 第二个侧向摩擦力
            print(f"正压力 (normal force): {normal_force}")
            print(f"侧向摩擦力1 (lateral friction1): {lateral_friction1}")
            print(f"侧向摩擦力2 (lateral friction2): {lateral_friction2}")



        # robot.moveArm(robot.arm_rest_poses)
        p.stepSimulation()
