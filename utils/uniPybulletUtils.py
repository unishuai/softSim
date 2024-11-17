# ============================================
# @Project  : snakeRectify3.py
# @File     : uniPybulletUtils.py
# @Date     : 2024-10-25 15:33
# @Author   : unishuai
# @email    : unishuai@gmail.com
# @des      : 我觉得Pybullet的有一些转换好麻烦，先自己按照需求写一个
# ============================================
from enum import Enum

import pybullet as p




class UniBulletUtils:
    #预定义一个枚举类型
    class JointTypeEnum(Enum):
        REVOLUTE = p.JOINT_REVOLUTE
        PRISMATIC = p.JOINT_PRISMATIC
        FIXED = p.JOINT_FIXED
        SPHERICAL = p.JOINT_SPHERICAL
        PLANAR = p.JOINT_PLANAR
        POINT2POINT = p.JOINT_POINT2POINT
        GEAR = p.JOINT_GEAR

    @staticmethod
    def getJointTypeString(jointType:int)->str:
        """
        将pybullet的Int类型转化成人看着舒服的str
        :param jointType:
        :return:
        """
        try:
            return UniBulletUtils.JointTypeEnum(jointType).name
        except ValueError:
            return "UNKNOWN"

    @staticmethod
    def getJointTypeInt(jointType:str)->int:
        """
        将人看着舒服的str转化为pybullet的Int类型
        :param jointType:
        :return:
        """
        try:
            return UniBulletUtils.JointTypeEnum[jointType].value
        except KeyError:
            return -1



