# ============================================
# @File    : ur10FreeHandSimMultiEfforts.py
# @Date    : 2024-07-07 15:17
# @Author  : 帅宇昕
# ============================================

import math
from abc import ABC, abstractmethod

import numpy as np
import pybullet as p
import copy
from collections import namedtuple
from deprecated import deprecated


class Ur10FreeHandSim(object):
    def __init__(self, bullet_client: p = p, offset: list = (0, 0, 0)):
        # todo:初始化一些参数，因为每一个model的参数并不是一样的，
        # 所以这里感觉传入会更加好一点

        # 客户端
        self.bullet_client = bullet_client
        # 初始的位姿
        self.offset = offset

        # 机械臂的自由度
        self.arm_num_dofs = 6
        # 机械臂的初始位置_6个自由度控制
        self.arm_rest_poses = [-1.5690622952052096, -1.5446774605904932, 1.343946009733127, -1.3708613585093699,-1.5707970583733368, 0.0009377758247187636]
        # self.arm_rest_poses = [1.5690622952052096, -1.5446774605904932, 1.343946009733127, -1.3708613585093699, -1.5707970583733368, 0.0009377758247187636]

        # 指抓的10个自由度初始位姿控制
        self.hand_rest_poses = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                1.418, 0.0, 0.0, 0.0]
        self.hand_closed_poses = [0, 2.090, 1.744, 0.743,
                                  0, 2.090, 1.744, 0.743,
                                  0, 2.090, 1.744, 0.743,
                                  0, 2.090, 1.744, 0.743,
                                  2.090,0.0, 0.0, 0.0
                                  ]

        # todo:读取urdf 文件
        # 加载地面
        # self.bullet_client.loadURDF("plane.urdf")
        self.robotId = self.bullet_client.loadURDF(r"../freehand_description/urdf/freehand_description.urdf",
                                                   basePosition=[0, 0, 0],
                                                   )
        if self.robotId < 0:
            # 还是这种抛出异常这样子写的爽一些
            raise ValueError("文件读取异常,检查一下是不是路径或者其他的问题")

        # todo:读取配置文件里面的相关信息
        # 映射，输出方便看
        self.jointTypeMapping = {getattr(p, name): name for name in dir(p) if name.startswith('JOINT_')}
        numJoints = p.getNumJoints(self.robotId)
        # 简单的一个java 类
        jointInfo = namedtuple('jointInfo',
                               ['id', 'name', 'type', 'damping', 'friction', 'lowerLimit', 'upperLimit',
                                'maxForce',
                                'maxVelocity', 'controllable'])
        # 这个记录每个关节信息
        self.joints = []
        # 这个记录可以控制的关节信息
        self.controllable_joints = []
        # 这个是我添加的，因为我觉得机械臂可以换，如果用数字的话肯定会出现一些问题的
        # 用字典的话，可以增强程序的健壮性
        self.jointIdMap = {}
        for i in range(numJoints):
            info = p.getJointInfo(self.robotId, i)
            jointID = info[0]
            jointName = info[1].decode("utf-8")
            jointType = info[2]  # JOINT_REVOLUTE, JOINT_PRISMATIC, JOINT_SPHERICAL, JOINT_PLANAR, JOINT_FIXED
            jointDamping = info[6]
            jointFriction = info[7]
            jointLowerLimit = info[8]
            jointUpperLimit = info[9]
            jointMaxForce = info[10]
            jointMaxVelocity = info[11]
            controllable = (jointType != p.JOINT_FIXED)
            if controllable:
                # 打印输出
                # print("jointID:{0} ,jointName:{1} ,jointType:{2}".format(info[0],info[1].decode('utf-8'),jointTypeMapping[info[2]]))
                # 添加可以控制关节的信息
                self.controllable_joints.append(jointID)
                self.jointIdMap[info[1].decode('utf-8')] = info[0]
                # # 让所有可以控制的关节速度都为0，不让其动
                # deprecated 之前那个代码真的是垃圾代码，不需要有这个
                # p.setJointMotorControl2(robotId, jointID, p.VELOCITY_CONTROL,
                #                         targetVelocity=0, force=0)
            # 添加进这个类似java的类里面
            info = jointInfo(jointID, jointName, jointType, jointDamping, jointFriction, jointLowerLimit,
                             jointUpperLimit,
                             jointMaxForce, jointMaxVelocity, controllable)
            # 然后把所有信息存储起来
            self.joints.append(info)

        assert len(
            self.controllable_joints) > self.arm_num_dofs, "自由度低于6，应该是没有读取到机械臂了,controllable_joints={0},arm_num_dofs{1}".format(
            len(self.controllable_joints), self.arm_num_dofs)

        # ==============手臂==============
        # 单独获取对手臂控制的关节
        self.arm_controllable_joints = self.controllable_joints[:self.arm_num_dofs]
        # 获取手臂的下限
        self.arm_lower_limits = [info.lowerLimit for info in self.joints if info.controllable][:self.arm_num_dofs]
        # 获取手臂的上限
        self.arm_upper_limits = [info.upperLimit for info in self.joints if info.controllable][:self.arm_num_dofs]
        # 获取手臂的移动范围
        self.arm_joint_ranges = [info.upperLimit - info.lowerLimit for info in self.joints if info.controllable][:self.arm_num_dofs]

        # ==============手==============
        # 单独获取对手关节抓控制的关节
        self.hand_controllable_joints = self.controllable_joints[self.arm_num_dofs:]
        # 获取手关节的下限
        self.hand_lower_limits = [info.lowerLimit for info in self.joints if info.controllable][self.arm_num_dofs:]
        # 获取手关节的上限
        self.hand_upper_limits = [info.upperLimit for info in self.joints if info.controllable][self.arm_num_dofs:]
        # 获取手关节的移动范围
        self.hand_joint_ranges = [info.upperLimit - info.lowerLimit for info in self.joints if info.controllable][
                                 self.arm_num_dofs:]

        # todo:初始化位姿，后续就应该是编写相应的指抓函数了
        # self.moveArm(self.arm_rest_poses)
        self.reset()  # 重置状态

    def moveArm(self, action, myMaxVelocity=-1):
        '''
        控制机械臂移动
        :param action:
        :param myMaxVelocity:
        :return:
        '''
        joint_poses = action
        if myMaxVelocity < 0:
            for i, joint_id in enumerate(self.arm_controllable_joints):
                p.setJointMotorControl2(self.robotId, joint_id, p.POSITION_CONTROL,
                                        joint_poses[i],
                                        force=self.joints[joint_id].maxForce,
                                        maxVelocity=self.joints[joint_id].maxVelocity)
        else:
            for i, joint_id in enumerate(self.arm_controllable_joints):
                p.setJointMotorControl2(self.robotId, joint_id, p.POSITION_CONTROL,
                                        joint_poses[i],
                                        force=self.joints[joint_id].maxForce,
                                        maxVelocity=myMaxVelocity)

    def controlHand(self, action):
        joint_poses = action
        for i, joint_id in enumerate(self.hand_controllable_joints):
            p.setJointMotorControl2(self.robotId, joint_id, p.POSITION_CONTROL,
                                    joint_poses[i],
                                    # force=self.joints[joint_id].maxForce,
                                    force=90,
                                    maxVelocity=self.joints[joint_id].maxVelocity)

    def calcJointLocation(self, pos, orn):
        """
        根据 pos 和 orn 计算机械臂的关节位置
        """
        jointPoses = self.bullet_client.calculateInverseKinematics(self.robotId, self.arm_num_dofs, pos, orn,
                                                                   self.arm_lower_limits, self.arm_upper_limits,
                                                                   self.arm_joint_ranges, self.arm_rest_poses,
                                                                   maxNumIterations=20)
        return jointPoses

    # region 额外加的两个函数，一个是多末端执行器控制，一个是逆运动学计算，但不修改机械臂
    # todo: 多末端执行器的控制
    def calcJointLocationMultiEffort(self, pos: list, orn: list):
        """
        多末端执行器控制
        :param pos:
        :param orn:
        :return:
        description:学姐的要求是让手指接触到物体Object
            由于我目前只能通过物体的ID获取到物体的位置和姿态，无法获取物体的长宽高，
        我感觉这个是比较难办的。目前github上暂时还没有具有参考价值的code。嗯，这里的话，
        我觉得把物体值传过来的时候，顺带copy成5份试一试。
        """

        # 需要控制的指抓的列表
        endEffectorLinkIndices = [
            self.jointIdMap.get('fi1-4joint'),
            self.jointIdMap.get('fi2-4joint'),
            self.jointIdMap.get('fi3-4joint'),
            self.jointIdMap.get('fi4-4joint'),
            self.jointIdMap.get('fi5-4joint')
        ]

        # 到达的位置列表
        poses = [copy.deepcopy(pos) for _ in range(len(endEffectorLinkIndices))]

        jointPoses = self.bullet_client.calculateInverseKinematics2(
            self.robotId,
            endEffectorLinkIndices,
            poses,
            orn,
            self.arm_lower_limits,
            self.arm_upper_limits,
            self.arm_joint_ranges,
            self.arm_rest_poses,
            maxNumIterations=20
        )

        return jointPoses

    def calcJointLocationOnlyFingers(self, pos, orn):
        raise NotImplementedError

    # endregion

    def moveAll(self,jointPose):
        joint_poses = copy.deepcopy(jointPose)

        for i, joint_id in enumerate(self.controllable_joints):
            p.setJointMotorControl2(self.robotId, joint_id, p.POSITION_CONTROL,
                                    joint_poses[i],
                                    force=self.joints[joint_id].maxForce if self.joints[joint_id].maxForce >200 else 200,
                                    maxVelocity=self.joints[joint_id].maxVelocity)

    def step(self, pos, angle, gripper_w, g_height=0.275):
        pass

    def reset(self):

        self.moveArm(self.arm_rest_poses)
        self.controlHand(self.hand_rest_poses)

    def update_state(self):
        raise NotImplementedError


class Ur10FreeHandSimMultiEffortsAuto(Ur10FreeHandSim):
    """
    添加根据时间，实现对Ur10FreeHandSim的操作
    """

    def reset(self):
        """
        重置状态
        """

        self.state = 0
        self.state_t = 0
        self.cur_state = 0
        super().reset()
        # self.controlHand(self.hand_rest_poses)
        # self.moveArm(self.arm_rest_poses)

    def __init__(self, bullet_client, offset: list = None):

        if offset is None:
            offset = [0, 0, 0]
        super().__init__(bullet_client, offset)
        self.state = -1
        self.state_t = 0
        self.cur_state = 0
        self.states = [0, 1, 2, 3, 4, 5, 6, 12]
        self.control_dt = 1. / 240
        self.state_durations = [1.0, 0.5, 2.0, 1.0, 0.5, 1.0, 1.0, 0.28]

    def update_state(self):
        self.state_t += self.control_dt
        if self.state_t > self.state_durations[self.cur_state]:
            self.cur_state += 1
            if self.cur_state >= len(self.states):
                self.cur_state = 0
            self.state_t = 0
            self.state = self.states[self.cur_state]



    def step(self, posTmp, angle: list, gripper_w, g_bias=None):
        """
        我觉的step还是得在simAuto,因为本身的sim其实并不参与多少动作，应当降低耦合度才是
        :param gripper_w:
        :param pos: 方位
        :param angle: 角度
        :param g_height:偏置值
        :return:
        """
        if g_bias==None:
            g_bias=[0, -0.19, 0.15]
        pos = [posTmp[x]+g_bias[x] for x in range(len(posTmp))]
        # 更新
        self.update_state()




        # pos[2] += g_height

        if self.state == -1:
            pass

        elif self.state == 0:

            # pos[2] = 0.5
            pos[2] += 0.2
            # euler = [0,0, angle + math.pi / 2]
            euler = angle
            orn = self.bullet_client.getQuaternionFromEuler(euler)  # 机械手方向
            jointPoses = self.calcJointLocation(pos, orn)

            self.moveArm(jointPoses)
            # print(f"指抓需要张开的长度{open_length}")

            return False

        elif self.state == 1:
            # 在抓取点的正上方做一个下降的操作
            pos[2] += 0.05
            orn = self.bullet_client.getQuaternionFromEuler(angle)  # 机械手方向
            jointPoses = self.calcJointLocation(pos, orn)
            self.moveArm(jointPoses)
            # self.controlHand(self.hand_rest_poses)
            forth_list = [x / 4 for x in self.hand_closed_poses]

            # 到达抓取点的正上方
            self.controlHand(self.hand_rest_poses)

            return False

        elif self.state == 2:
            pos[2] += 0.04
            # 这里就到达了预定的抓取位置，机械臂继续下落后，手应该和点重合
            orn = self.bullet_client.getQuaternionFromEuler(angle)  # 机械手方向
            jointPoses = self.calcJointLocation(pos, orn)
            # In fact, I abandoned this parameter -->(myMaxVelocity)
            # 因为我读取urdf的文件的时候有，暂时先不删除
            self.moveArm(jointPoses)
            return False

        elif self.state == 3:
            pos[2] += 0.03
            # 这里就到达了预定的抓取位置，机械臂继续下落后，手应该和点重合
            orn = self.bullet_client.getQuaternionFromEuler(angle)  # 机械手方向
            jointPoses = self.calcJointLocation(pos, orn)
            # In fact, I abandoned this parameter -->(myMaxVelocity)
            # 因为我读取urdf的文件的时候有，暂时先不删除
            self.moveArm(jointPoses)
            # 这里就做了一个抓取的操作
            # 需要对手指进行修改!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            thirds_list = [x / 2 for x in self.hand_closed_poses]
            self.controlHand(thirds_list)
            return False

        elif self.state == 4:
            pos[2] += 0.01
            # 这里就到达了预定的抓取位置，机械臂继续下落后，手应该和点重合
            orn = self.bullet_client.getQuaternionFromEuler(angle)  # 机械手方向
            jointPoses = self.calcJointLocation(pos, orn)

            # 这里手指头完全闭合
            # 需要对手指进行修改!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            self.controlHand(self.hand_closed_poses)
            for _ in range(2):
                p.stepSimulation()
            return False

        elif self.state == 5:
            # print('物体上方(预抓取位置)')
            pos[2] += 0.05
            orn = self.bullet_client.getQuaternionFromEuler(angle)  # 机械手方向
            jointPoses = self.calcJointLocation(pos, orn)
            self.moveArm(jointPoses)

            return False

        elif self.state == 6:
            # print('物体上方')
            # pos[2] = 0.3
            pos[2] += 0.2

            orn = self.bullet_client.getQuaternionFromEuler(angle)  # 机械手方向
            jointPoses = self.calcJointLocation(pos, orn)
            self.moveArm(jointPoses)

            return False


        elif self.state == 12:

            # print("------------")

            self.reset()  # 重置状态
            return True
