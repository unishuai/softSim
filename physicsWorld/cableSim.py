# ============================================
# @File    : cableSim.py
# @Date    : 2024-07-28 19:03
# @Author  : 帅宇昕
# ============================================
from typing import Optional, List, Union

import pybullet as pl
import time
import math
import pybullet_data


class CableSim(object):

    def __init__(self, bullet_client: pl , position:Optional[List[Union[float, int]]]  = None) -> None:
        super().__init__()
        if position is None:
            position = [0.3, -.3, 0.1]
        self.p=bullet_client
        self.ballIds=list()
        self.position=position
        # self.cableLen = 4
        self.ballNum = 36+45+15
        self.height = 0.01
        # self.sphereRadius = self.cableLen / self.ballNum - self.height
        self.sphereRadius=0.01
        self.loadCable()

    def loadCable(self):
        if len(self.ballIds)>=1:
            return

        baseAngle = [math.radians(90),math.radians(0), math.radians(0)]

        # 缆线的质量
        mass = 0.01
        # 可视化的数据id，-1就是默认颜色，不自定义可视化
        # visualShapeId = -1
        colBallId = self.p.createCollisionShape(self.p.GEOM_CYLINDER, radius=self.sphereRadius, height=self.height)
        visualShapeId = self.p.createVisualShape(self.p.GEOM_CYLINDER, radius=self.sphereRadius, length=self.height * 1.8, rgbaColor=[1, 0, 0, 1])
        endpointsVisualShapeId = self.p.createVisualShape(self.p.GEOM_CYLINDER, radius=self.sphereRadius, length=self.height, rgbaColor=[1, 0, 0, 1])
        useMaximalCoordinates = True

        basePosition = [0 + self.position[0], self.height + self.position[1], 1 + self.position[2]]
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
            linkPositions.append([0, 0, self.height])
            linkOrientations.append(self.p.getQuaternionFromEuler([0, 0, 0]))
            linkInertialFramePositions.append([0, 0, 0])
            linkInertialFrameOrientations.append(self.p.getQuaternionFromEuler([0, 0, 0]))
            linkParentIndices.append(i)
            linkJointTypes.append(self.p.JOINT_FIXED)
            linkJointAxis.append([0, 0, 0])

        ballId=self.p.createMultiBody(
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
        self.ballIds.append(ballId)





    def removeCable(self):
        for ballId in self.ballIds:
            self.p.removeBody(ballId)
        self.ballIds.clear()

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
