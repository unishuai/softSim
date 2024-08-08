# ============================================
# @File    : handGraspcableTest.py
# @Date    : 2024-08-08 11:02
# @Author  : 帅宇昕
# @desc    : 这个类，目前打算是进行测试学姐提出用捏去抓缆线，
# 我目前的想法，捏就是控制每一根手指的底部（这里把手从末端到手掌依次叫远指关节，近指关节，掌指关节）
# ============================================
import argparse
import random

import numpy as np
import pybullet as p
import pybullet_data
import time
# v1
import physicsWorld.ur10FreeHandSim as mySim








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



    for i in [37,38,39,40]:
        dynamics_info = p.getDynamicsInfo(robot.robotId, i)
        print(f"id={i},dynamics_info:{dynamics_info}")



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
        if robot.step(objectPos, [-2.0,3.14,-3.14], 0) is True:
            for i in range(50):
                p.stepSimulation()
                time.sleep(0.007)
            objectPos, objectOrn = p.getBasePositionAndOrientation(ObjectId)
        # robot.moveArm(robot.arm_rest_poses)
        p.stepSimulation()
