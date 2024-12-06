# ============================================
# @Project  : softSim
# @File     : twpgripper.py
# @Date     : 2024-11-28 18:27
# @Author   : unishuai
# @email    : unishuai@gmail.com
# @des      : function description
# ============================================

from typing import List
import pybullet as p

from collections import namedtuple


class gripperSim(object):
    def __init__(self, bullet_client: p = p, offset: list = (0, 0, 0)):
        # todo:初始化一些参数，因为每一个model的参数并不是一样的，
        # 所以这里感觉传入会更加好一点

        # 客户端
        self.p = bullet_client
        # 初始的位姿
        self.offset = offset
        self.hand_rest_poses=0.6
        self.hand_rest_position=[0,0,0.5]



        # todo:读取urdf 文件

        self.robotId = self.p.loadURDF(r"../franka_panda/panda_gripper.urdf",
                                       basePosition=offset,useFixedBase=True,
                                       )
        if self.robotId < 0:
            # 还是这种抛出异常这样子写的爽一些
            raise ValueError("文件读取异常,检查一下是不是路径或者其他的问题")

        # todo:读取配置文件里面的相关信息
        # 映射，输出方便看
        self.jointTypeMapping = {getattr(p, name): name for name in dir(p) if name.startswith('JOINT_')}
        numJoints = self.p.getNumJoints(self.robotId)
        # 简单的一个java 类
        jointInfo = namedtuple('jointInfo',
                               ['id', 'name', 'type', 'damping', 'friction', 'lowerLimit', 'upperLimit',
                                'maxForce',
                                'maxVelocity', 'controllable'])
        # 这个记录每个关节信息
        self.joints = []
        # 这个记录可以控制的关节信息
        self.controllable_joints = []
        for i in range(numJoints):
            info = self.p.getJointInfo(self.robotId, i)
            jointID = info[0]
            jointName = info[1].decode("utf-8")
            jointType = info[2]  # JOINT_REVOLUTE, JOINT_PRISMATIC, JOINT_SPHERICAL, JOINT_PLANAR, JOINT_FIXED
            jointDamping = info[6]
            jointFriction = info[7]
            jointLowerLimit = info[8]
            jointUpperLimit = info[9]
            jointMaxForce = info[10]
            jointMaxVelocity = info[11]
            controllable = (jointType != self.p.JOINT_FIXED)
            if controllable:
                # 打印输出
                # print("jointID:{0} ,jointName:{1} ,jointType:{2}".format(info[0],info[1].decode('utf-8'),jointTypeMapping[info[2]]))
                # 添加可以控制关节的信息
                self.controllable_joints.append(jointID)

                # # 让所有可以控制的关节速度都为0，不让其动
                # deprecated 之前那个代码真的是垃圾代码，不需要有这个
                # self.p.setJointMotorControl2(robotId, jointID, self.p.VELOCITY_CONTROL,
                #                         targetVelocity=0, force=0)
            # 添加进这个类似java的类里面
            info = jointInfo(jointID, jointName, jointType, jointDamping, jointFriction, jointLowerLimit,
                             jointUpperLimit,
                             jointMaxForce, jointMaxVelocity, controllable)
            # 然后把所有信息存储起来
            self.joints.append(info)

        # ==============手==============
        # 单独获取对手关节抓控制的关节
        self.hand_controllable_joints = self.controllable_joints[:]
        # 获取手关节的下限
        self.hand_lower_limits = [info.lowerLimit for info in self.joints if info.controllable][:]
        # 获取手关节的上限
        self.hand_upper_limits = [info.upperLimit for info in self.joints if info.controllable][:]
        # 获取手关节的移动范围
        self.hand_joint_ranges = [info.upperLimit - info.lowerLimit for info in self.joints if info.controllable][:]

        # # todo:初始化位姿，后续就应该是编写相应的指抓函数了
        self.moveArm(self.hand_rest_position)
    #
    # def getHandRestPose(self):
    #     return tuple(self.hand_rest_poses)
    #
    # def getHandClosedPose(self):
    #     return tuple(self.hand_closed_poses)

    def getHnadLowerLimit(self):
        return tuple(self.hand_lower_limits)

    def getHandUpperLimit(self):
        return tuple(self.hand_upper_limits)

    def getHandJointRange(self):
        return tuple(self.hand_joint_ranges)


    def controlHand(self, action):
        self.p.setJointMotorControl2(self.robotId, 1, self.p.POSITION_CONTROL, action, force=10)
        self.p.setJointMotorControl2(self.robotId, 2, self.p.POSITION_CONTROL, action, force=10)


    # def calcJointLocation(self, pos, orn):
    #     """
    #     根据 pos 和 orn 计算机械臂的关节位置
    #     """
    #     # jointPoses = self.p.calculateInverseKinematics(self.id, pandaEndEffectorIndex, pos, orn, ll, ul, jr, rp, maxNumIterations=20)
    #     jointPoses = self.p.calculateInverseKinematics(self.robotId, self.arm_num_dofs, pos, orn,
    #                                                    self.arm_lower_limits, self.arm_upper_limits,
    #                                                    self.arm_joint_ranges, self.arm_rest_poses,
    #                                                    maxNumIterations=20)
    #     return jointPoses
    def moveArm(self, dist, maxVelocity=10):
        """
        设置机械手位置
        """
        # self.p.setJointMotorControl2(self.robotId, 0, self.p.POSITION_CONTROL, dist, force=50., maxVelocity=maxVelocity)

    def step(self ,width,dist):
        pass

    def reset(self):
        # self.moveArm(self.arm_rest_poses)
        self.controlHand(self.hand_rest_poses)

    def update_state(self):
        raise NotImplementedError

    def getFingerRange(self):
        """
        用来返回数据，供页面交互用
        :return: 获取指抓关节的关节角
        """
        fingerRange = dict()
        fingerNum: int = 5
        fingerAngle: int = 4
        startNum: int = 9
        for i in range(fingerNum):
            fingerTmp = "finger" + str(i + 1)
            tmpList = list()
            for j in range(fingerAngle):
                tmpList.append([self.joints[startNum].lowerLimit, self.joints[startNum].upperLimit])
                startNum += 1
            fingerRange[fingerTmp] = tmpList

        return fingerRange


class GripperSimAuto(gripperSim):
    """
    添加根据时间，实现对Ur10FreeHnadSim的操作
    """

    def reset(self):
        """
        重置状态
        """

        self.state = 0
        self.state_t = 0
        self.cur_state = 0
        super().reset()

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


    def step(self,gwidth,dist):
        g_bias = [0, -0.19, 0.175]
        pos=[x for x in self.hand_rest_position]


        # 更新
        self.update_state()


        if self.state == -1:
            pass

        elif self.state == 0:
            # 到达抓取点的正上方

            # pos[2] = 0.5
            pos[2] += 0.2


            self.moveArm(dist)

            self.controlHand(gwidth)
            return False

        elif self.state == 1:
            # 在抓取点的正上方做一个下降的操作
            pos[2] += 0.05

            self.moveArm(dist)

            return False

        elif self.state == 2:
            # 这里就到达了预定的抓取位置，机械臂继续下落后，手应该和点重合

            self.moveArm(dist)
            return False

        elif self.state == 3:

            self.controlHand(gwidth/2)
            return False

        elif self.state == 4:
            # 这里手指头完全闭合
            # 需要对手指进行修改!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            # self.move_gripper(0)
            self.controlHand(gwidth/10)
            return False

        elif self.state == 5:
            # print('物体上方(预抓取位置)')
            pos[2] += 0.05

            self.moveArm(dist)

            return False

        elif self.state == 6:
            # print('物体上方')
            # pos[2] = 0.3
            pos[2] += 0.2


            self.moveArm(dist)

            return False


        elif self.state == 12:

            self.reset()  # 重置状态
            return True
