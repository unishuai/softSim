# ============================================
# @File    : BulletWorld.py
# @Date    : 2024-07-20 15:46
# @Author  : 帅宇昕
# ============================================
from typing import List

import numpy as np
import pybullet as p
import pybullet_data
import time
# v1
# import ur10FreeHandSim as mySim
from PySide6.QtCore import QObject

import physicsWorld.ur10FreeHandSim as mySim
import physicsWorld.cableSim as mycableSim


class BulletWorld(QObject):
    def __init__(self):
        super().__init__()

        self.p=p
        self.client_id = p.connect(p.GUI)  # 使用GUI模式
        self.p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.gravity=-9.8
        self.setWorldGravity(self.gravity)

        self.planId= p.loadURDF("plane.urdf",basePosition=[0, 0, 0])

        self.cupId=p.loadURDF("../otherModel/urdf/coffee_cup.urdf", np.array([0.3, -0.6, 0]),globalScaling=1)
        self.robot=mySim.Ur10FreeHnadSimAuto(p,[0,0,0])
        # self.initFingerBendSway()
        # self.cable=mycableSim.CableSim()
        self.cables:List[mycableSim.CableSim]=list()


    def setWorldGravity(self, gravity: float = -9.8):
        self.gravity=gravity
        self.p.setGravity(0, 0, gravity)

    def getWorldGravity(self):
        return self.gravity

    def stepWorldSimulation(self,sleepTime:float=0.0):
        self.p.stepSimulation()
        if sleepTime>0.0000001:
            time.sleep(sleepTime)

    def isConnected(self)->bool:
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
        self.cables.append(mycableSim.CableSim())

    def removeCable(self):
        # print("in removeCable")
        if not self.cables:
            return
        tmpSim=self.cables[len(self.cables)-1]
        tmpSim.removeCable()
        # print("removed Cable")
        self.cables.pop(len(self.cables)-1)












