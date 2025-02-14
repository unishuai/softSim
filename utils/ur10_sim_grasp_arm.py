# ============================================
# @File    : ur10_sim_grasp_arm.py
# @Date    : 2024-01-03 21:11
# @Author  : 帅宇昕
# ============================================


import math

import numpy as np
import pybullet as p

from collections import namedtuple

# useNullSpace = 1
# ikSolver = 0
# pandaEndEffectorIndex = 11  # 8
# pandaNumDofs = 7
#
# ll = [-7] * pandaNumDofs
#
# ul = [7] * pandaNumDofs
#
# jr = [7] * pandaNumDofs

# jointPositions = [0.98, 0.458, 0.31, -2.24, -0.30, 2.66, 2.32, 0.02, 0.02]

# rp = jointPositions


class PandaSim(object):
    def __init__(self, bullet_client: p = p, offset: list = (0, 0, 0)):
        # todo:初始化一些参数，因为每一个model的参数并不是一样的，


        self.eef_id = 7
        self.arm_num_dofs = 6
        self.arm_rest_poses = [-1.5690622952052096, -1.5446774605904932, 1.343946009733127, -1.3708613585093699, -1.5707970583733368, 0.0009377758247187636]

        # 先临时吧p给他
        # self.bullet_client:p=bullet_client
        self.bullet_client = bullet_client
        self.bullet_client.setPhysicsEngineParameter(solverResidualThreshold=0)

        self.offset = np.array(offset)

        # 开启缓存
        flags = self.bullet_client.URDF_ENABLE_CACHED_GRAPHICS_SHAPES

        # orn = [-0.707107, 0.0, 0.0, 0.707107]
        orn = [0.9, 0.0, 0.0, 0.0]
        orn = [0, 0, 0, 1]
        # orn = self.bullet_client.getQuaternionFromEuler([math.radians(-90), 0., 0.])
        orn = self.bullet_client.getQuaternionFromEuler([0, 0., math.radians(-160)])
        eul = self.bullet_client.getEulerFromQuaternion([-0.5, -0.5, -0.5, 0.5])

        # todo:1 读取文件配置
        # -------------------------------------------------
        self.id = self.bullet_client.loadURDF('../myurdf/myur10_Robitq140.urdf', np.array([0, 0, 0]) + self.offset, orn,useFixedBase=True, flags=flags)



        if self.id < 0:
            raise ValueError("myur10_Robitq140文件读取异常,检查一下是不是路径或者其他的问题")

        # -------------------------------------------------

        # todo:2 修改文件配置
        # --------------------------------------------------
        numJoints = self.bullet_client.getNumJoints(self.id)
        jointInfo = namedtuple('jointInfo',
                               ['id', 'name', 'type', 'damping', 'friction', 'lowerLimit', 'upperLimit', 'maxForce',
                                'maxVelocity', 'controllable'])
        self.joints = []
        self.controllable_joints = []
        for i in range(numJoints):

            info = self.bullet_client.getJointInfo(self.id, i)
            jointID = info[0]

            jointName = info[1].decode("utf-8")
            jointType = info[2]  # JOINT_REVOLUTE, JOINT_PRISMATIC, JOINT_SPHERICAL, JOINT_PLANAR, JOINT_FIXED
            jointDamping = info[6]
            jointFriction = info[7]
            jointLowerLimit = info[8]
            jointUpperLimit = info[9]
            jointMaxForce = info[10]
            jointMaxVelocity = info[11]
            controllable = (jointType != self.bullet_client.JOINT_FIXED)
            if controllable:
                self.controllable_joints.append(jointID)
                self.bullet_client.setJointMotorControl2(self.id, jointID, self.bullet_client.VELOCITY_CONTROL,targetVelocity=0, force=0)
            info = jointInfo(jointID, jointName, jointType, jointDamping, jointFriction, jointLowerLimit,jointUpperLimit, jointMaxForce, jointMaxVelocity, controllable)
            self.joints.append(info)


        # print(self.controllable_joints)

        assert len(self.controllable_joints) >= self.arm_num_dofs
        self.arm_controllable_joints = self.controllable_joints[:self.arm_num_dofs]

        self.arm_lower_limits = [info.lowerLimit for info in self.joints if info.controllable][:self.arm_num_dofs]
        self.arm_upper_limits = [info.upperLimit for info in self.joints if info.controllable][:self.arm_num_dofs]
        self.arm_joint_ranges = [info.upperLimit - info.lowerLimit for info in self.joints if info.controllable][:self.arm_num_dofs]




        # --------------------------------------------------

        # todo:3 创建约束,这里是需要创建约束的父节点和子节点
        # ------------------------------------------------
        mimic_parent_name = 'finger_joint'
        mimic_children_names = {'right_outer_knuckle_joint': -1,
                                'left_inner_knuckle_joint': -1,
                                'right_inner_knuckle_joint': -1,
                                'left_inner_finger_joint': 1,
                                'right_inner_finger_joint': 1}

        self.mimic_parent_id = [joint.id for joint in self.joints if joint.name == mimic_parent_name][0]
        self.mimic_child_multiplier = {joint.id: mimic_children_names[joint.name] for joint in self.joints if
                                       joint.name in mimic_children_names}

        for joint_id, multiplier in self.mimic_child_multiplier.items():
            c = self.bullet_client.createConstraint(self.id, self.mimic_parent_id,
                                                    self.id, joint_id,
                                                jointType=self.bullet_client.JOINT_GEAR,
                                                    jointAxis=[0, 1, 0],
                                                    parentFramePosition=[0, 0, 0],
                                                    childFramePosition=[0, 0, 0])
            self.bullet_client.changeConstraint(c, gearRatio=-multiplier, maxForce=300,
                                                erp=1)  # Note: the mysterious `erp` is of EXTREME importance

        # ------------------------------------------------



        # index = 0

        self.state_t = 0
        self.cur_state = 0
        self.state = -1
        self.control_dt = 1. / 240
        # self.finger_target = 0
        # self.gripper_height = 0.2
        #这里初始化位置
        self.move_ee(self.arm_rest_poses, 'joint')
        self.move_gripper(0.085)

        #

        # for j in range(self.bullet_client.getNumJoints(self.id)):
        #     # 消除阻尼
        #     self.bullet_client.changeDynamics(self.id, j, linearDamping=0, angularDamping=0)
        #     # 这里可以获取关节的名字和类型
        #     info = self.bullet_client.getJointInfo(self.id, j)
        #
        #     jointName = info[1]
        #     jointType = info[2]
        #
        #     print(f"id={j},jointName={info[1]},jointName={info[2]}")
        #     # 这里判断它的关节的种类，和前面约束的时候种类名称有点不同
        #     if (jointType == self.bullet_client.JOINT_PRISMATIC):
        #         self.bullet_client.resetJointState(self.id, j, jointPositions[index])
        #         index = index + 1
        #     # 这里是先给旋转的值好吧，后面两个就是平移的
        #     if jointType == self.bullet_client.JOINT_REVOLUTE:
        #         self.bullet_client.resetJointState(self.id, j, jointPositions[index])
        #         index = index + 1
        # self.t = 0

    # todo：func这里我加载两个函数，控制指抓移动
    def move_gripper(self, open_length):
        #这个怎么说呢，就是我给他加的，因为他那个移动指抓的时候是两个移动，我这个是移动一个

        # open_length = np.clip(open_length, *self.gripper_range)
        open_angle = 0.715 - math.asin((open_length - 0.010) / 0.1143)  # angle calculation


        # Control the mimic gripper joint(s)
        self.bullet_client.setJointMotorControl2(self.id, self.mimic_parent_id, self.bullet_client.POSITION_CONTROL,
                                                 targetPosition=open_angle,
                                                 force=10 ,
                                                 maxVelocity=self.joints[self.mimic_parent_id].maxVelocity)

    # todo：func这里我加载两个函数，控制指抓移动
    def move_ee(self, action, control_method='joint', myMaxVelocity=-1):
        assert control_method in ('joint', 'end')
        joint_poses = []
        if control_method == 'end':
            x, y, z, roll, pitch, yaw = action
            pos = (x, y, z)
            orn = self.bullet_client.getQuaternionFromEuler((roll, pitch, yaw))
            joint_poses = self.bullet_client.calculateInverseKinematics(self.id, self.eef_id, pos, orn,
                                                                self.arm_lower_limits, self.arm_upper_limits,
                                                                self.arm_joint_ranges, self.arm_rest_poses,
                                                                maxNumIterations=20)

        elif control_method == 'joint':
            # assert len(action) == self.arm_num_dofs
            # joint_poses = None
            # if len(action) > self.arm_num_dofs:
            #     joint_poses = action[:self.arm_num_dofs + 1]
            # else:
            #     joint_poses = action
            joint_poses=action
        # arm
        if myMaxVelocity < 0:
            for i, joint_id in enumerate(self.arm_controllable_joints):
                self.bullet_client.setJointMotorControl2(self.id, joint_id, self.bullet_client.POSITION_CONTROL,
                                                         joint_poses[i],
                                                         force=self.joints[joint_id].maxForce,
                                                         maxVelocity=self.joints[joint_id].maxVelocity)
        else:
            for i, joint_id in enumerate(self.arm_controllable_joints):
                self.bullet_client.setJointMotorControl2(self.id, joint_id, self.bullet_client.POSITION_CONTROL,
                                                         joint_poses[i],
                                                         force=self.joints[joint_id].maxForce,
                                                         maxVelocity=0.07)

    def setArmPos(self, pos):
        # orn = self.bullet_client.getQuaternionFromEuler([0,0,math.pi / 2])   # 机械手方向
        orn = self.bullet_client.getQuaternionFromEuler([0,0,0])   # 机械手方向
        jointPoses = self.calcJointLocation(pos, orn)
        self.move_ee(jointPoses,control_method='joint')

    def step(self, pos, angle, gripper_w, control_method='joint'):

        """
        action: (x, y, z, roll, pitch, yaw, gripper_opening_length) for End Effector Position Control
                (a1, a2, a3, a4, a5, a6, a7, gripper_opening_length) for Joint Position Control
        control_method:  'end' for end effector position control
                         'joint' for joint position control
        """
        "[grasp_x, grasp_y, grasp_z], grasp_angle, grasp_width/2"
        self.update_state()
        height=0.1
        g_height=0.294+height

        open_length = gripper_w

        # self.state += 1

        pos[2] += 0.00
        if self.state == -1:
            pass

        elif self.state == 0:

            pos[2] = 0.2+g_height
            # euler = [0,0, angle + math.pi / 2]
            euler = [0,0, angle ]
            orn = self.bullet_client.getQuaternionFromEuler(euler)  # 机械手方向
            jointPoses = self.calcJointLocation(pos, orn)

            self.move_ee(jointPoses, 'joint')
            # print(f"指抓需要张开的长度{open_length}")
            self.move_gripper(open_length)
            return False

        elif self.state == 1:
            # print(self.state)

            pos[2] += 0.05+g_height
            orn = self.bullet_client.getQuaternionFromEuler([0, 0, angle])  # 机械手方向
            jointPoses = self.calcJointLocation(pos, orn)
            # self.setArm(jointPoses)
            self.move_ee(jointPoses, 'joint')

            return False

        elif self.state == 2:
            # print(self.state)
            # print('抓取位置')
            orn = self.bullet_client.getQuaternionFromEuler([0,0, angle ])  # 机械手方向
            pos[2] += 0.03 + g_height
            jointPoses = self.calcJointLocation(pos, orn)
            # In fact, I abandoned this parameter -->(myMaxVelocity)
            # 因为我读取urdf的文件的时候有，暂时先不删除
            self.move_ee(jointPoses, 'joint', myMaxVelocity=3)
            # self.setArm(jointPoses, maxVelocity=3)
            return False

        elif self.state == 3:
            pos[2] += 0.03 + g_height
            # print(self.state)
            # print('闭合抓取器')
            # self.setGripper(0)
            self.move_gripper(0)
            return False

        elif self.state == 4:
            # print(self.state)
            # print('物体上方(预抓取位置)')
            pos[2] += 0.07+g_height

            orn = self.bullet_client.getQuaternionFromEuler([0, 0, angle ])  # 机械手方向
            jointPoses = self.calcJointLocation(pos, orn)
            # self.setArm(jointPoses, maxVelocity=0.5)
            self.move_ee(jointPoses, 'joint', myMaxVelocity=10)
            self.move_gripper(0)

            return False
        elif self.state == 5:
            # print(self.state)
            # print('物体上方(预抓取位置)')
            pos[2] += 0.13+g_height
            orn = self.bullet_client.getQuaternionFromEuler([0, 0, angle ])  # 机械手方向
            jointPoses = self.calcJointLocation(pos, orn)
            # self.setArm(jointPoses, maxVelocity=0.5)
            self.move_ee(jointPoses, 'joint', myMaxVelocity=10)
            self.move_gripper(0)

            return False
        elif self.state == 6:
            # print(self.state)
            # print('物体上方(预抓取位置)')
            pos[2] += 0.20+g_height
            orn = self.bullet_client.getQuaternionFromEuler([0, 0, angle ])  # 机械手方向
            jointPoses = self.calcJointLocation(pos, orn)
            # self.setArm(jointPoses, maxVelocity=0.5)
            self.move_ee(jointPoses, 'joint', myMaxVelocity=10)
            self.move_gripper(0)

            return False
        elif self.state == 7:
            # print(self.state)
            # print('物体上方(预抓取位置)')
            pos[2] += 0.27+g_height
            orn = self.bullet_client.getQuaternionFromEuler([0, 0, angle ])  # 机械手方向
            jointPoses = self.calcJointLocation(pos, orn)
            # self.setArm(jointPoses, maxVelocity=0.5)
            self.move_ee(jointPoses, 'joint', myMaxVelocity=10)
            self.move_gripper(0)

            return False
        elif self.state == 8:
            # print(self.state)
            # print('物体上方(预抓取位置)')
            pos[2] += 0.33+g_height
            orn = self.bullet_client.getQuaternionFromEuler([0, 0, angle ])  # 机械手方向
            jointPoses = self.calcJointLocation(pos, orn)
            # self.setArm(jointPoses, maxVelocity=0.5)
            self.move_ee(jointPoses, 'joint', myMaxVelocity=10)
            self.move_gripper(0)

            return False
        elif self.state == 9:
            # print(self.state)
            # print('物体上方(预抓取位置)')
            pos[2] += 0.38+g_height
            orn = self.bullet_client.getQuaternionFromEuler([0, 0, angle ])  # 机械手方向
            jointPoses = self.calcJointLocation(pos, orn)
            # self.setArm(jointPoses, maxVelocity=0.5)
            self.move_ee(jointPoses, 'joint', myMaxVelocity=10)
            self.move_gripper(0)

            return False
        elif self.state == 10:
            # print(self.state)
            # print('物体上方(预抓取位置)')
            pos[2] += 0.40+g_height
            orn = self.bullet_client.getQuaternionFromEuler([0, 0, angle ])  # 机械手方向
            jointPoses = self.calcJointLocation(pos, orn)
            # self.setArm(jointPoses, maxVelocity=0.5)
            self.move_ee(jointPoses, 'joint', myMaxVelocity=10)
            self.move_gripper(0)

            return False
        # elif self.state == 11:
        #     print(self.state)
        #     # print('物体上方(预抓取位置)')
        #     pos[2] += 0.40+g_height
        #     orn = self.bullet_client.getQuaternionFromEuler([0, 0, angle ])  # 机械手方向
        #     jointPoses = self.calcJointLocation(pos, orn)
        #     # self.setArm(jointPoses, maxVelocity=0.5)
        #     self.move_ee(jointPoses, 'joint', myMaxVelocity=10)
        #     self.move_gripper(0)
        #
        #     return False
        # elif self.state == 12:
        #     print(self.state)
        #     # print('物体上方(预抓取位置)')
        #     pos[2] += 0.075+g_height
        #     orn = self.bullet_client.getQuaternionFromEuler([0, 0, angle ])  # 机械手方向
        #     jointPoses = self.calcJointLocation(pos, orn)
        #     # self.setArm(jointPoses, maxVelocity=0.5)
        #     self.move_ee(jointPoses, 'joint', myMaxVelocity=10)
        #     self.move_gripper(0)
        #
        #     return False
        # elif self.state == 13:
        #     print(self.state)
        #     # print('物体上方(预抓取位置)')
        #     pos[2] += 0.080+g_height
        #     orn = self.bullet_client.getQuaternionFromEuler([0, 0, angle ])  # 机械手方向
        #     jointPoses = self.calcJointLocation(pos, orn)
        #     # self.setArm(jointPoses, maxVelocity=0.5)
        #     self.move_ee(jointPoses, 'joint', myMaxVelocity=10)
        #     self.move_gripper(0)
        #
        #     return False
        # elif self.state == 14:
        #     print(self.state)
        #     # print('物体上方(预抓取位置)')
        #     pos[2] += 0.085+g_height
        #     orn = self.bullet_client.getQuaternionFromEuler([0, 0, angle ])  # 机械手方向
        #     jointPoses = self.calcJointLocation(pos, orn)
        #     # self.setArm(jointPoses, maxVelocity=0.5)
        #     self.move_ee(jointPoses, 'joint', myMaxVelocity=10)
        #     self.move_gripper(0)
        #
        #     return False
        # elif self.state == 15:
        #     print(self.state)
        #     # print('物体上方(预抓取位置)')
        #     pos[2] += 0.090+g_height
        #     orn = self.bullet_client.getQuaternionFromEuler([0, 0, angle ])  # 机械手方向
        #     jointPoses = self.calcJointLocation(pos, orn)
        #     # self.setArm(jointPoses, maxVelocity=0.5)
        #     self.move_ee(jointPoses, 'joint', myMaxVelocity=10)
        #     self.move_gripper(0)
        #
        #     return False
        # elif self.state == 16:
        #     print(self.state)
        #     # print('物体上方(预抓取位置)')
        #     pos[2] += 0.095+g_height
        #     orn = self.bullet_client.getQuaternionFromEuler([0, 0, angle ])  # 机械手方向
        #     jointPoses = self.calcJointLocation(pos, orn)
        #     # self.setArm(jointPoses, maxVelocity=0.5)
        #     self.move_ee(jointPoses, 'joint', myMaxVelocity=10)
        #     self.move_gripper(0)
        #
        #     return False
        #
        # elif self.state == 17:
        #     print(self.state)
        #     # print('物体上方')
        #     # pos[2] = 0.3
        #     pos[2] = 0.100+g_height
        #
        #     orn = self.bullet_client.getQuaternionFromEuler([0, 0, angle ])  # 机械手方向
        #     jointPoses = self.calcJointLocation(pos, orn)
        #     self.move_ee(jointPoses, 'joint', myMaxVelocity=10)
        #     self.move_gripper(0)
        #
        #     return False
        # elif self.state == 18:
        #     print(self.state)
        #     # print('物体上方')
        #     # pos[2] = 0.3
        #     pos[2] = 0.105+g_height
        #
        #     orn = self.bullet_client.getQuaternionFromEuler([0, 0, angle ])  # 机械手方向
        #     jointPoses = self.calcJointLocation(pos, orn)
        #     self.move_ee(jointPoses, 'joint', myMaxVelocity=10)
        #     self.move_gripper(0)
        #
        #     return False
        # elif self.state == 19:
        #     print(self.state)
        #     # print('物体上方')
        #     # pos[2] = 0.3
        #     pos[2] = 0.110+g_height
        #
        #     orn = self.bullet_client.getQuaternionFromEuler([0, 0, angle ])  # 机械手方向
        #     jointPoses = self.calcJointLocation(pos, orn)
        #     self.move_ee(jointPoses, 'joint', myMaxVelocity=10)
        #     self.move_gripper(0)
        #
        #     return False
        # elif self.state == 20:
        #     print(self.state)
        #     # print('物体上方')
        #     # pos[2] = 0.3
        #     pos[2] = 0.115+g_height
        #
        #     orn = self.bullet_client.getQuaternionFromEuler([0, 0, angle ])  # 机械手方向
        #     jointPoses = self.calcJointLocation(pos, orn)
        #     self.move_ee(jointPoses, 'joint', myMaxVelocity=10)
        #     self.move_gripper(0)
        #
        #     return False

        elif self.state == 50:
            print(12)

            # print("------------")

            self.reset()  # 重置状态
            return True

        # assert control_method in ('joint', 'end')
        # # self.move_ee(action[:-1], control_method)
        # # self.move_gripper(action[-1])
        # for _ in range(120):  # Wait for a few steps
        #     self.bullet_client.stepSimulation()

    def calcJointLocation(self, pos, orn):
        """
        根据 pos 和 orn 计算机械臂的关节位置
        """
        # jointPoses = self.bullet_client.calculateInverseKinematics(self.id, pandaEndEffectorIndex, pos, orn, ll, ul, jr, rp, maxNumIterations=20)
        jointPoses = self.bullet_client.calculateInverseKinematics(self.id, self.arm_num_dofs, pos, orn,
                                                                   self.arm_lower_limits, self.arm_upper_limits,
                                                                   self.arm_joint_ranges, self.arm_rest_poses,
                                                                   maxNumIterations=20)
        return jointPoses

    def reset(self):
        """
                重置状态
        """
        self.state = -1
        self.state_t = 0
        self.cur_state = 0

    def update_state(self):
       raise NotImplementedError




class PadnaSimAuto(PandaSim):
    def __init__(self, bullet_client, offset):
        PandaSim.__init__(self, bullet_client, offset)
        self.state_t = 0
        self.cur_state = 0
        self.states = [-1,0, 1,
                       2, 3, 4,
                       5,6,7,
                       8,9,10,
                       # 11,
                       # 12,13,
                       # 14,15,16,
                       # 17,18,19,
                       20, 50]

        self.state_durations = [1,2.0, 2.5,
                                2.0, 0.5, 1.0,
                                1.0, 1.0,1.0,
                                0.8, 0.8, 0.8,
                                # 0.8,
                                # 0.8, 0.8,
                                # 0.8, 0.8, 0.8,
                                # 0.8, 0.8, 0.8,
                                1.0, 0.5
                                ]

    def update_state(self):
        self.state_t += self.control_dt
        if self.state_t > self.state_durations[self.cur_state]:
            self.cur_state += 1
            if self.cur_state >= len(self.states):
                self.cur_state = 0
            self.state_t = 0
            self.state = self.states[self.cur_state]
