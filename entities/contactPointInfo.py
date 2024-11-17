# ============================================
# @Project  : snakeRectify3.py
# @File     : contactPointInfo.py
# @Date     : 2024-11-14 22:21
# @Author   : unishuai
# @email    : unishuai@gmail.com
# @des      : function description
# ============================================
from dataclasses import dataclass
from typing import Tuple, List


@dataclass
class ContactPointInfo:
    bodyA: str                                # 第一个碰撞体的 ID
    bodyB: str                                # 第二个碰撞体的 ID
    linkIndexA: int                                # 第一个碰撞体的 link ID
    linkIndexB: int                                # 第二个碰撞体的 link ID
    positionA:Tuple[float,float,float]               #第一个碰撞体的位置
    positionB:Tuple[float,float,float]               #第二个碰撞体的位置
    colPositionOnA:Tuple[float, float, float]         # 第一个碰撞体上的接触点位置
    colPositionOnB:Tuple[float, float, float]         # 第二个碰撞体上的接触点位置
    NormalOnB:Tuple[float, float, float]         # 第二个碰撞体上的接触点法向量
    contactNormalOnB:Tuple[float, float, float]         # 第二个碰撞体上的接触点法向量
    contactDistance:float                        # 接触点的距离
    normalForce:float                            # 接触点的法向力
    lateralFriction1:float                        # 接触点的切向力1
    lateralFrictionDir1:Tuple[float, float, float]    # 接触点的切向力1的方向
    lateralFriction2:float                        # 接触点的切向力2
    lateralFrictionDir2:Tuple[float, float, float]    # 接触点的切向力2的方向
    AName="gripper"
    BName="cable"




    @classmethod
    def createContactInfo(cls,contactPoints:list, linkStateInfoA:list,linkStateInfoB:list) -> "ContactPointInfo":
        """

        :param linkStateInfo:
        :param contactPoints:
        :return:
        """

        # 解析 linkStateInfo（假设返回值结构中包含 body 和 link 信息）
        bodyA = cls.AName+ str(contactPoints[1])
        bodyB = cls.BName+ str(contactPoints[2])
        linkIndexA = contactPoints[3]
        linkIndexB = contactPoints[4]
        #就只有这两个是使用linkStateInfo的
        positionA = linkStateInfoA[0]
        positionB = linkStateInfoB[0]

        # 解析 contactPoints
        colPositionOnA = tuple(contactPoints[5])  # Contact position on A
        colPositionOnB = tuple(contactPoints[6])  # Contact position on B
        contactNormalOnB = tuple(contactPoints[7])  # Contact normal on B
        contactDistance = contactPoints[8]  # Contact distance
        normalForce = contactPoints[9]  # Normal force
        lateralFriction1 = contactPoints[10]  # Lateral friction 1
        lateralFrictionDir1 = tuple(contactPoints[11])  # Direction of lateral friction 1
        lateralFriction2 = contactPoints[12]  # Lateral friction 2
        lateralFrictionDir2 = tuple(contactPoints[13])  # Direction of lateral friction 2

        # 创建 ContactPointInfo 对象
        return cls(
            bodyA=bodyA,
            bodyB=bodyB,
            linkIndexA=linkIndexA,
            linkIndexB=linkIndexB,
            positionA=positionA,
            positionB=positionB,
            colPositionOnA=colPositionOnA,
            colPositionOnB=colPositionOnB,
            NormalOnB=contactNormalOnB,  # 假设 NormalOnB 与 contactNormalOnB 相同
            contactNormalOnB=contactNormalOnB,
            contactDistance=contactDistance,
            normalForce=normalForce,
            lateralFriction1=lateralFriction1,
            lateralFrictionDir1=lateralFrictionDir1,
            lateralFriction2=lateralFriction2,
            lateralFrictionDir2=lateralFrictionDir2
        )

