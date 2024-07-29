# ============================================
# @File    : cableSim.py
# @Date    : 2024-07-28 19:03
# @Author  : 帅宇昕
# ============================================
from typing import Optional, List, Union

import pybullet as p
import time
import math
import pybullet_data


class CableSim(object):

    def __init__(self, position:Optional[List[Union[float,int]]]  = None) -> None:
        super().__init__()
        if position is None:
            position = [0.3, -.3, 0.1]
        self.ballIds=list()
        self.position=position
        # self.cableLen = 4
        self.ballNum = 36
        self.height = 0.1
        # self.sphereRadius = self.cableLen / self.ballNum - self.height
        self.sphereRadius=0.01
        self.loadCable()

    def loadCable(self):
        if len(self.ballIds)>=1:
            return
        preLinkId = -1
        baseAngle = [-90, 0, 0]
        for i in range(self.ballNum):
            colBallId = p.createCollisionShape(p.GEOM_CAPSULE,
                                               radius=self.sphereRadius, height=self.height)
            # 缆线的质量
            mass = 1
            # 可视化的数据id，-1就是默认颜色，不自定义可视化
            visualShapeId = -1
            basePosition = [0 + self.position[0], i * self.sphereRadius * 2 + i * self.height + self.position[1],
                            self.position[2]]
            baseOrientation = p.getQuaternionFromEuler(baseAngle)
            linkId = p.createMultiBody(mass,
                                       colBallId,
                                       visualShapeId,
                                       basePosition,
                                       baseOrientation
                                       )
            self.ballIds.append(linkId)
            if preLinkId != -1:
                constraint_id = p.createConstraint(
                    parentBodyUniqueId=preLinkId,
                    parentLinkIndex=-1,
                    childBodyUniqueId=linkId,
                    childLinkIndex=-1,
                    jointType=p.JOINT_POINT2POINT,
                    jointAxis=[0, 0, 0],
                    parentFramePosition=[0, 0, self.height / 2 + self.sphereRadius],
                    childFramePosition=(0, 0, -self.height / 2 - self.sphereRadius)
                )

                # 设置约束的最大力
                p.changeConstraint(constraint_id, maxForce=100000000000000)
            preLinkId = linkId

    def removeCable(self):
        for ballId in self.ballIds:
            p.removeBody(ballId)
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
