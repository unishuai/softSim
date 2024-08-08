# ============================================
# @File    : BulletWorld.py
# @Date    : 2024-07-20 15:46
# @Author  : 帅宇昕
# ============================================
from typing import List, Tuple

import numpy as np
import pybullet as p
import pybullet_data
import time
# v1
# import ur10FreeHandSim as mySim
from PySide6.QtCore import QObject

import physicsWorld.ur10FreeHandSim as mySim
# import physicsWorld.cableSim as mycableSim
import physicsWorld.cableSim as mycableSim


class BulletWorld(QObject):
    def __init__(self):
        super().__init__()


        self.p = p
        print(pybullet_data.getDataPath())
        self.client_id = self.p.connect(self.p.GUI)  # 使用GUI模式
        self.p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.gravity = -9.8
        self._simStatus=False

        self.setWorldGravity(self.gravity)

        self.planId = self.p.loadURDF("plane.urdf", basePosition=[0, 0, 0])
        self.tableId = self.p.loadURDF("table/table.urdf", basePosition=[0, -0.7, 0])
        self.deskId = self.p.loadURDF("table_square/table_square.urdf", basePosition=[0, 0, 0],globalScaling=0.5)

        self.cupId = self.p.loadURDF("../otherModel/urdf/coffee_cup.urdf", np.array([0.3, -0.6, 0]), globalScaling=1)
        self.robot = mySim.Ur10FreeHnadSimAuto(self.p, [0, 0, 0.3])
        self.handClosePose =self.robot.getHandClosedPose()
        self.handOpenPose =self.robot.getHandRestPose()
        self.handLowerLimits=self.robot.getHnadLowerLimit()
        self.handUpperLimits=self.robot.getHandUpperLimit()
        self.handJointRanges=self.robot.getHandJointRange()
        # self.initFingerBendSway()
        # self.cable=mycableSim.CableSim()
        self.cables: List[mycableSim.CableSim] = list()

    def setWorldGravity(self, gravity: float = -9.8):
        self.gravity = gravity
        self.p.setGravity(0, 0, gravity)

    def getWorldGravity(self):
        return self.gravity

    def stepWorldSimulation(self, sleepTime: float = 0.0):
        self.p.stepSimulation()

        if self.startSim is True:
            if len(self.cables) == 0:
                self.startSim = False
                return
            time.sleep(0.08)
            #持续进行仿真抓取
            # print("进入了stepWorldSimulation 仿真里面")

            # print(f"获取缆线的数据objectPos={objectPos},objectOrn={objectOrn}")
            for i in range(30):
                if self.robot.step(self.objectPos, [-2.0,3.14,-3.14], 2) is True:
                    # print("这里说明step为True成功进入")
                    # for i in range(100):
                    #     self.p.stepSimulation()
                        # time.sleep(0.001)
                    self.objectPos, self.objectOrn = self.p.getBasePositionAndOrientation(self.cables[-1].ballIds[-1])


        if sleepTime > 0.0000001:
            time.sleep(sleepTime)

    def isConnected(self) -> bool:
        return self.p.isConnected()

    #
    # def initFingerBendSway(self):
    #     """
    #     初始化灵巧手的参数
    #     :return:
    #     """
    #     fingerRange=self.robot.getFingerRange()
    #     self.fingerBnedRangeNative=dict()
    #     self._fingerSwayRange=dict()
    #     tmpStr="finger"
    #     for i in range(5):
    #         self.fingerBnedRangeNative[tmpStr+str(i+1)]=fingerRange.get(tmpStr+str(i+1))[1:]
    #         self._fingerSwayRange[tmpStr+str(i+1)]=fingerRange.get(tmpStr+str(i+1))[:1]

    # def setfingerSway(self,controlAngle:list=None):
    #     #unishuai created in 2024/7/21 10:32
    #     #description:先进行安全性检查，然后调用角度控制，将数据发送给机器人
    #     # 我觉得这个思路还是不是特别正确，机器人的每次一控制，应该发的是所有的关节的角度才对
    #     # 也就是说，我应当有一个变量(我认为应当定义成一个类)，然后每一次操作的时候，都应当去修改这个类的信息
    #     if controlAngle is None:
    #         controlAngle=[0,0,0,0,0]

    def addCable(self):
        self.cables.append(mycableSim.CableSim(self.p))
        # print(f"robotId={self.robot.robotId}, cableId={self.cables[-1].ballIds[-1]},cablesLen={len(self.cables)}")
        self.objectPos, self.objectOrn = self.p.getBasePositionAndOrientation(self.cables[-1].ballIds[-1])

    def removeCable(self):
        # print("in removeCable")
        if not self.cables:
            return
        tmpSim = self.cables[len(self.cables) - 1]
        # print(f"remove ballId={tmpSim.ballIds[-1]}")
        tmpSim.removeCable()
        # print("removed Cable")
        # self.p.removeBody(self.robot.robotId)
        # self.cables.pop(len(self.cables) - 1)

    def controlHand(self, ctrlSig: Tuple[list,list]):
        assert ctrlSig is not None, "控制信号为空"
        assert len(ctrlSig) >= 2, "接收的控制信号元素少于2"

        bendSigList, swaySigList = ctrlSig
        angleList=list()

        # todo:把控制信号翻译成灵巧手仿真可以理解的语言
        for i in range(len(bendSigList)):

            # print(len(self.handUpperLimits))
            # print(self.handUpperLimits)

            swayAngle=self.handUpperLimits[i*4]*swaySigList[i]/5.0
            angleList.append(swayAngle)

            bendAngle1=self.handClosePose[i*4+1]*bendSigList[i]/10.0
            angleList.append(bendAngle1)
            bendAngle2=self.handClosePose[i*4+2]*bendSigList[i]/10.0
            angleList.append(bendAngle2)
            bendAngle3=self.handClosePose[i*4+3]*bendSigList[i]/10.0
            angleList.append(bendAngle3)

        self.robot.controlHand(angleList)



    @property
    def startSim(self):
        return self._simStatus

    @startSim.setter
    def startSim(self, value):
        self._simStatus=value

    def startGraspSim(self):
        # self.robot.startGraspSim()
        # print("startGraspSim 被调用了")
        self.startSim=True

    def stopGraspSim(self):
        self.startSim=False
