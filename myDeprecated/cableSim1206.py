# ============================================
# @Project  : softSim
# @File     : cableSim.py
# @Date     : 2024-08-13 10:04
# @Author   : unishuai
# @email    : unishuai@gmail.com
# @des      : function description
# ============================================
from typing import Optional, List, Union

import pybullet as p
import time
import math
import pybullet_data


class CableSim(object):

    def __init__(self, bullet_client: p = p, position: Optional[List[Union[float, int]]] = None, cabFri: float = 0.7,
                 cabLen: float = 0.64, cabDiameter: float = 0.02, cabMass: float = 0.0016) -> None:
        super().__init__()
        if position is None:
            position = [0.3, -.3, 0.1]

        # self.p = p
        self.p: p = bullet_client
        self.ballIds = list()
        self.position = position
        # 添加缆线四个参数，通过函数对外暴露
        # 摩擦系数
        self._cableFriction: float = float(cabFri)
        # 缆线长度
        self._cableLen: float = float(cabLen)
        # 缆线直径
        self._cableDiameter: float = float(cabDiameter)
        # 缆线质量
        self._cableMass: float = float(cabMass)

        # self.height = 0.01
        self.height: float = 0.04
        # self.ballNum = (36 + 45 + 15) // 6
        self.ballNum: int = int(self.cabLen / self.height)

        # self.sphereRadius = self.cableLen / self.ballNum - self.height
        # self.sphereRadius = 0.01
        self.sphereRadius = self.cableRadius
        self.loadCable()

    def loadCable(self):
        if len(self.ballIds) >= 1:
            return

        baseAngle = [math.radians(90), math.radians(0), math.radians(0)]

        # 缆线的质量
        # mass = 0.0001
        mass = self.cableMass / self.ballNum
        # 可视化的数据id，-1就是默认颜色，不自定义可视化
        # visualShapeId = -1
        colBallId = self.p.createCollisionShape(self.p.GEOM_CAPSULE, radius=self.sphereRadius, height=self.height)
        visualShapeId = self.p.createVisualShape(self.p.GEOM_CAPSULE, radius=self.sphereRadius,
                                                 length=self.height * 1, rgbaColor=[1, 0, 0, 1])
        endpointsVisualShapeId = self.p.createVisualShape(self.p.GEOM_CAPSULE, radius=self.sphereRadius,length=self.height, rgbaColor=[1, 0, 0, 1])
        useMaximalCoordinates = True

        basePosition = [0 + self.position[0], self.height + self.position[1], self.sphereRadius + self.position[2]]
        baseOrientation = self.p.getQuaternionFromEuler(baseAngle)

        linkMasses = list()
        linkCollisionShapeIndices = list()
        linkVisualShapeIndices = list()
        linkPositions = list()
        linkOrientations = list()
        linkInertialFramePositions = list()
        linkInertialFrameOrientations = list()
        linkParentIndices = list()
        linkJointTypes = list()
        linkJointAxis = list()

        for i in range(self.ballNum):
            linkMasses.append(mass)
            linkCollisionShapeIndices.append(colBallId)
            if i == 0 or i == self.ballNum - 1:
                linkVisualShapeIndices.append(endpointsVisualShapeId)
            else:
                linkVisualShapeIndices.append(visualShapeId)
            #我想起来了，这个修改成的是胶囊，所以会弄的稍微大一些，正常情况下，需要加上它的半圆形
            linkPositions.append([0, 0, self.height])
            linkOrientations.append(self.p.getQuaternionFromEuler([0, 0, 0]))
            linkInertialFramePositions.append([0, 0, 0])
            linkInertialFrameOrientations.append(self.p.getQuaternionFromEuler([0, 0, 0]))
            linkParentIndices.append(i)
            linkJointTypes.append(self.p.JOINT_FIXED)
            linkJointAxis.append([0, 0, 0])

        ballId = self.p.createMultiBody(
            baseMass=mass,
            baseCollisionShapeIndex=colBallId,
            baseVisualShapeIndex=endpointsVisualShapeId,
            basePosition=basePosition,
            baseOrientation=baseOrientation,

            linkMasses=linkMasses,
            linkCollisionShapeIndices=linkCollisionShapeIndices,
            linkVisualShapeIndices=linkVisualShapeIndices,
            linkPositions=linkPositions,
            linkOrientations=linkOrientations,
            linkInertialFramePositions=linkInertialFramePositions,
            linkInertialFrameOrientations=linkInertialFrameOrientations,
            linkParentIndices=linkParentIndices,
            linkJointTypes=linkJointTypes,
            linkJointAxis=linkJointAxis,
            useMaximalCoordinates=useMaximalCoordinates

        )

        for i in range(p.getNumJoints(ballId) + 1):
            # tmp1=p.getJointInfo(tmp, i)
            # print(f"tmp{i}={tmp1}")
            p.changeDynamics(ballId, i - 1,
                             lateralFriction=self._cableFriction,
                             linearDamping=0.05,
                             angularDamping=0.05,
                             rollingFriction=0.01,
                             )

        self.ballIds.append(ballId)

    def removeCable(self):
        for ballId in self.ballIds:
            self.p.removeBody(ballId)
        self.ballIds.clear()

    def getMeidaPositionAndOrientation(self):
        numJoints = self.p.getNumJoints(self.ballIds[-1])
        mediaJointInfo = self.p.getJointInfo(self.ballIds[-1], numJoints // 2)
        return self.p.getBasePositionAndOrientation(self.ballIds[0])

    @property
    def cableFriction(self)-> float:
        return self._cableFriction

    @cableFriction.setter
    def cableFriction(self, friction: float):
        friction=float(friction)
        if friction <= 0:
            raise ValueError("摩擦系数应当为正数")
        else:
            self._cableFriction = friction

    @property
    def cabLen(self)-> float:
        return self._cableLen

    @cabLen.setter
    def cabLen(self, cLen: float):
        cLen=float(cLen)
        if cLen <= 0:
            raise ValueError("缆线长度应当为正数")
        else:
            self._cableLen = cLen

    @property
    def cableDiameter(self)-> float:
        return self._cableDiameter

    @cableDiameter.setter
    def cableDiameter(self, diameter: float):
        diameter = float(diameter)
        if diameter <= 0:
            raise ValueError("缆线直径应当为正数")
        else:
            self._cableDiameter = diameter

    @property
    def cableRadius(self) -> float:
        return self.cableDiameter / 2

    @cableRadius.setter
    def cableRadius(self, radius: float):
        radius=float(radius)
        if radius <= 0:
            raise ValueError("缆线半径应当为正数")
        else:
            self.cableDiameter = radius * 2

    @property
    def cableMass(self):
        return self._cableMass

    @cableMass.setter
    def cableMass(self, mass:float):
        mass=float(mass)
        if mass < 0:
            raise ValueError("缆线质量应当为非负数")
        else:
            self._cableMass = mass

# p.connect(p.GUI)
# p.setAdditionalSearchPath(pybullet_data.getDataPath())
# plane = p.createCollisionShape(p.GEOM_PLANE)
#
# p.createMultiBody(0, plane)


# colBallId = p.createCollisionShapeArray([p.GEOM_BOX, p.GEOM_SPHERE],radii=[sphereRadius+0.03,sphereRadius+0.03], halfExtents=[[sphereRadius,sphereRadius,sphereRadius],[sphereRadius,sphereRadius,sphereRadius]])
# 修改为一个球体

# p.setPhysicsEngineParameter(constraintSolverType=p.CONSTRAINT_SOLVER_LCP_PGS, globalCFM = 0.00000001)


#
# mass = 1
# visualShapeId = -1
#
# link_Masses = []
# linkCollisionShapeIndices = []
# linkVisualShapeIndices = []
# linkPositions = []
# linkOrientations = []
# linkInertialFramePositions = []
# linkInertialFrameOrientations = []
# indices = []
# jointTypes = []
# axis = []
#
# for i in range(36):
#     link_Masses.append(1)
#     linkCollisionShapeIndices.append(colBallId)
#     linkVisualShapeIndices.append(-1)
#     # if i==0:
#     #   linkPositions.append([0, 0, 0])
#     # else:
#     linkPositions.append([0, sphereRadius * 2.0 + 0.01, 0])
#     linkOrientations.append([0, 0, 0, 1])
#     linkInertialFramePositions.append([0, 0, 0])
#     linkInertialFrameOrientations.append([0, 0, 0, 1])
#     indices.append(i)
#     # 修改关节的连接方式
#     # jointTypes.append(p.JOINT_REVOLUTE)
#     jointTypes.append(p.JOINT_POINT2POINT)
#     # 如果是球形连接的话，也不需要有轴了
#     # axis.append([0, 0, 1])
#     axis.append([0, 0, 0])
#
# basePosition = [0, 0, 1]
# baseOrientation = [0, 0, 0, 1]
# sphereUid = p.createMultiBody(mass,
#                               colBallId,
#                               visualShapeId,
#                               basePosition,
#                               baseOrientation,
#                               linkMasses=link_Masses,
#                               linkCollisionShapeIndices=linkCollisionShapeIndices,
#                               linkVisualShapeIndices=linkVisualShapeIndices,
#                               linkPositions=linkPositions,
#                               linkOrientations=linkOrientations,
#                               linkInertialFramePositions=linkInertialFramePositions,
#                               linkInertialFrameOrientations=linkInertialFrameOrientations,
#                               linkParentIndices=indices,
#                               linkJointTypes=jointTypes,
#                               linkJointAxis=axis,
#                               useMaximalCoordinates=useMaximalCoordinates)
#
# p.setGravity(0, 0, -10)
# p.setRealTimeSimulation(0)
#

#


# while True:
#     p.stepSimulation()
#     time.sleep(0.01)
