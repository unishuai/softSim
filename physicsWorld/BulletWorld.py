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
from PySide6 import QtCore
from PySide6.QtCore import QObject

import physicsWorld.ur10FreeHandSim as mySim
# import physicsWorld.cableSim as mycableSim
import physicsWorld.cableSim as mycableSim


class BulletWorld(QObject):
    def __init__(self):
        super().__init__()



        self.p = p
        # print(pybullet_data.getDataPath())
        self.client_id = self.p.connect(self.p.GUI)  # 使用GUI模式
        self.p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.gravity = -9.8
        self._simStatus=False
        self.setWorldGravity(self.gravity)

        self.planId = self.p.loadURDF("plane.urdf", basePosition=[0, 0, 0])
        self.tableId = self.p.loadURDF("table/table.urdf", basePosition=[0, -0.7, 0])
        self.deskId = self.p.loadURDF("table_square/table_square.urdf", basePosition=[0, 0, 0],globalScaling=0.5)

        self.cupId = self.p.loadURDF("../otherModel/urdf/coffee_cup.urdf", np.array([0.3, -0.6, 1]), globalScaling=1)
        self.robot = mySim.Ur10FreeHnadSimAuto(self.p, [0, 0, 0.3])
        self.handClosePose =self.robot.getHandClosedPose()
        self.handOpenPose =self.robot.getHandRestPose()
        self.handLowerLimits=self.robot.getHnadLowerLimit()
        self.handUpperLimits=self.robot.getHandUpperLimit()
        self.handJointRanges=self.robot.getHandJointRange()
        # self.initFingerBendSway()
        # self.cable=mycableSim.CableSim()
        self.cables: List[mycableSim.CableSim] = list()

        # region todo:创建缆线的参数
        # 摩擦系数
        self._cableFriction = 0.7
        # 缆线长度
        self._cableLen = 1.2
        # 缆线直径
        self._cableDiameter = 0.008
        # 缆线质量
        self._cableMass = 0.0016
        # endregion

    def setWorldGravity(self, gravity: float = -9.8):
        self.gravity = gravity
        self.p.setGravity(0, 0, gravity)

    def getWorldGravity(self):
        return self.gravity

    @QtCore.Slot()
    def stepWorldSimulation(self, sleepTime: float = 0.0):
        time.sleep(0.005)
        self.p.stepSimulation()


        if self.startSim is True:
            if len(self.cables) == 0:
                self.startSim = False
                return
            # time.sleep(0.08)
            #持续进行仿真抓取
            # print("进入了stepWorldSimulation 仿真里面")

            # print(f"获取缆线的数据objectPos={objectPos},objectOrn={objectOrn}")
            # for i in range(30):
            # self.objectPos, self.objectOrn = self.p.getBasePositionAndOrientation(self.cables[-1].ballIds[0])
            self.objectPos, self.objectOrn = self.p.getBasePositionAndOrientation(self.cables[-1].ballIds[0])
            if self.robot.step(self.objectPos, [-2.0, 3.14, -3.14], 2) is True:
                # print("这里说明step为True成功进入")
                # for i in range(100):
                #     self.p.stepSimulation()
                # time.sleep(0.001)
                # todo: 获取每一次抓取的信息

                # contact_points=self.p.getContactPoints(self.robot.robotId,self.cables[0].ballIds[0])
                # # 遍历接触点信息，提取法向力和摩擦力
                # for point in contact_points:
                #     normal_force = point[9]  # 正压力（法向力）
                #     lateral_friction1 = point[10]  # 第一个侧向摩擦力
                #     lateral_friction2 = point[12]  # 第二个侧向摩擦力
                #
                #     print(f"正压力 (normal force): {normal_force}")
                #     print(f"侧向摩擦力1 (lateral friction1): {lateral_friction1}")
                #     print(f"侧向摩擦力2 (lateral friction2): {lateral_friction2}")


                self.objectPos, self.objectOrn = self.p.getBasePositionAndOrientation(self.cables[-1].ballIds[0])

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
        #这个是旧的版本，使用默认的参数
        # self.cables.append(mycableSim.CableSim(self.p,[0,-0.6,1]))
        # self.cables.append(mycableSim.CableSim(self.p,[0,-0.6,1]))

        #这个是新版本的，更具Qt窗口添加参数
        self.cables.append(mycableSim.CableSim(self.p,[0,-0.8,1],cabFri=self._cableFriction,cabLen=self._cableLen,cabDiameter=self._cableDiameter,cabMass=self._cableMass))
        # print(f"robotId={self.robot.robotId}, cableId={self.cables[-1].ballIds[-1]},cablesLen={len(self.cables)}")


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


    @property
    def cableFriction(self)->float:
        return self._cableFriction

    @cableFriction.setter
    def cableFriction(self, friction :float):
        if float(friction) <= 0:
            raise ValueError("摩擦系数应当为正数")
        else:
            self._cableFriction = friction

    @property
    def cableLen(self)->float:
        return self._cableLen

    @cableLen.setter
    def cableLen(self, cLen:float):
        if float(cLen) <= 0:
            raise ValueError("缆线长度应当为正数")
        else:
            self._cableLen = cLen

    @property
    def cableDiameter(self)->float:
        return self._cableDiameter

    @cableDiameter.setter
    def cableDiameter(self, diameter:float):
        if float(diameter) <= 0:
            raise ValueError("缆线直径应当为正数")
        else:
            self._cableDiameter = diameter

    @property
    def cableRadius(self)->float:
        return self._cableDiameter / 2

    @cableRadius.setter
    def cableRadius(self, radius:float):
        if float(radius) <= 0:
            raise ValueError("缆线半径应当为正数")
        else:
            self._cableDiameter = radius * 2

    @property
    def cableMass(self)->float:
        return self._cableMass

    @cableMass.setter
    def cableMass(self, mass:float):
        if float(mass) < 0:
            raise ValueError("缆线质量应当为非负数")
        else:
            self._cableMass = mass
