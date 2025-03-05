# ============================================
# @Project  : softSim
# @File     : cableSim.py
# @Date     : 2024-12-06 21:34
# @Author   : unishuai
# @email    : unishuai@gmail.com
# @des      : function description
# ============================================
import threading
from typing import Optional, List, Union

import numpy as np
import pybullet as p
import time
import math
import pybullet_data
import pathlib
import tacto

from utils.TactoBodyDataClass import TactoBodyDataClass


class CableSim(object):

    def __init__(self, bullet_client: p = p, position: Optional[List[Union[float, int]]] = None, cabFri: float = 0.7,
                 cabLen: float = 1.2, cabDiameter: float = 0.006, cabMass: float = 0.0016) -> None:
        super().__init__()
        if position is None:
            position = [0.3, -.3, 0.1]
        # 记录下当前类的文件路径
        self.parentPath=pathlib.Path(__file__).parent.resolve()

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

        # basePosition = [0 + self.position[0], self.height + self.position[1], self.sphereRadius + self.position[2]]
        # baseOrientation = self.p.getQuaternionFromEuler(baseAngle)

        preLinkId=-1
        linkIds=list()
        self.linkIdBodyDatas=[]
        for i in range(self.ballNum):
            # todo:用代码创建模型
            # colBallId = p.createCollisionShape(p.GEOM_CYLINDER, radius=self.sphereRadius, height=self.height)
            # visualShapeId = p.createVisualShape(p.GEOM_CYLINDER, radius=self.sphereRadius, length=self.height,
            #                                     rgbaColor=[1, 0, 0, 1])
            #
            #
            # # 可视化的数据id，-1就是默认颜色，不自定义可视化
            # # visualShapeId = -1
            # basePosition = [0 + self.position[0], i * (self.height) * 1.5 + self.position[1], 0.1 * self.height + self.position[2]]
            # baseOrientation = p.getQuaternionFromEuler(baseAngle)
            # linkId = p.createMultiBody(mass,
            #                            colBallId,
            #                            visualShapeId,
            #                            basePosition,
            #                            baseOrientation
            #                            )

            # linkIds.append(linkId)
            # if preLinkId != -1:
            #     constraint_id = p.createConstraint(
            #         parentBodyUniqueId=preLinkId,
            #         parentLinkIndex=-1,
            #         childBodyUniqueId=linkId,
            #         childLinkIndex=-1,
            #         jointType=p.JOINT_FIXED,
            #         jointAxis=[0, 0, 0],
            #         parentFramePosition=[0, 0, self.height / 2],
            #         childFramePosition=(0, 0, -self.height / 2),
            #
            #     )

            # todo:用urdf导入模型
            basePosition = [0 + self.position[0], i * (self.height) * 1.5 + self.position[1], 0.1 * self.height + self.position[2]]
            baseOrientation = p.getQuaternionFromEuler(baseAngle)
            linkId = p.loadURDF(str(self.parentPath.parent/ "Line1101"/"urdf"/"LineSingle.urdf"), basePosition,
                                baseOrientation)
            linkIdBodyData = TactoBodyDataClass(str(self.parentPath.parent/ "Line1101"/"urdf"/"LineSingle.urdf"), np.array(basePosition),1,
                                linkId)

            self.linkIdBodyDatas.append(linkIdBodyData)



            linkIds.append(linkId)
            if preLinkId != -1:
                constraint_id = p.createConstraint(
                    parentBodyUniqueId=preLinkId,
                    parentLinkIndex=-1,
                    childBodyUniqueId=linkId,
                    childLinkIndex=-1,
                    jointType=p.JOINT_FIXED,
                    jointAxis=[0, 0, 0],
                    # 这个部分也进行了相应的修改
                    parentFramePosition=[0, 0, self.height *19/50 ],
                    childFramePosition=(0, 0, -self.height *19/50 ),

                )


                p.changeConstraint(constraint_id, maxForce=1000, erp=0.9)
            preLinkId = linkId

        for i in linkIds:

            p.changeDynamics(i, -1,
                             lateralFriction=0.7,
                             linearDamping=0.30,
                             angularDamping=0.30,
                             rollingFriction=0.01,
                             )

        # self.ballIds.append(linkIds)
        self.ballIds=linkIds

        #todo:一定时间后，取消固定点
        fixed_constraint = p.createConstraint(
            parentBodyUniqueId=linkIds[0],
            parentLinkIndex=-1,
            childBodyUniqueId=-1,  # -1 表示世界
            childLinkIndex=-1,
            jointType=p.JOINT_POINT2POINT,
            jointAxis=[0, 0, 0],  # 固定关节不需要轴
            parentFramePosition=[0, 0, -self.height / 2],  # 在缆线局部坐标的固定点
            childFramePosition=self.position  # 世界坐标的固定点
        )
        threading.Timer(2,self.afterCreateActions, args=(fixed_constraint,)).start()

    def removeCable(self):
        for ballId in self.ballIds:
            if isinstance(ballId, (list,tuple)):
                for i in ballId:
                    self.p.removeBody(i)
            else:
                self.p.removeBody(ballId)
        self.ballIds.clear()

    def getMeidaPositionAndOrientation(self):
        numJoints = self.p.getNumJoints(self.ballIds[-1])
        mediaJointInfo = self.p.getJointInfo(self.ballIds[-1], numJoints // 2)
        return self.p.getBasePositionAndOrientation(self.ballIds[0])

    def afterCreateActions(self,constraintId,*args, **kwargs):
        """
        删除约束
        :param constraintId:
        :param args:
        :param kwargs:
        :return:
        """
        self.p.removeConstraint(constraintId)

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
