# ============================================
# @File    : BulletWorld.py
# @Date    : 2024-07-20 15:46
# @Author  : 帅宇昕
# ============================================
import argparse
import random

import numpy as np
import pybullet as p
import pybullet_data
import time
# v1
# import ur10FreeHandSim as mySim
from PySide6.QtCore import QObject

import ur10FreeHandSimMultiEfforts as mySim


class BulletWorld(QObject):
    def __init__(self):
        super().__init__()
        self.p=p
        self.client_id = p.connect(p.GUI)  # 使用GUI模式
        self.p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.p.setGravity(0, 0, -9.8)

        self.planId= p.loadURDF("plane.urdf",basePosition=[0, 0, 0])

        self.cupId=p.loadURDF("../otherModel/urdf/coffee_cup.urdf", np.array([0.3, -0.6, 0]),globalScaling=1)
        self.robot=mySim.Ur10FreeHandSimMultiEffortsAuto(p,[0,0,0])

    def setWorldGravity(self, gravity: float = -9.8):
        self.p.setGravity(0, 0, gravity)

    def stepWorldSimulation(self,sleepTime:float=0.0):
        self.p.stepSimulation()
        if sleepTime>0.0000001:
            time.sleep(sleepTime)

    def isConnected(self)->bool:
        return self.p.isConnected()

    def getFingerBendRange(self)->tuple:
        pass

    def getFingerSwayRange(self)->tuple:
        pass



