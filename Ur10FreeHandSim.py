# # ============================================
# # @File    : Ur10FreeHandSim.py
# # @Date    : 2024-06-29 18:14
# # @Author  : 帅宇昕
# # ============================================
# import math
#
# import numpy as np
# import pybullet as p
#
# from collections import namedtuple
#
# class Ur10FreeHnadSim(object):
#     def __init__(self, bullet_client: p = p, offset: list = (0, 0, 0)):
#         #todo:初始化一些参数，因为每一个model的参数并不是一样的，
#         # 所以这里感觉传入会更加好一点
#
#         # self.eef_id = 7
#         # self.arm_num_dofs = 6
#         # self.arm_rest_poses = [-1.5690622952052096, -1.5446774605904932, 1.343946009733127, -1.3708613585093699, -1.5707970583733368, 0.0009377758247187636]
#         #
#         # # 先临时吧p给他
#         # # self.bullet_client:p=bullet_client
#         # self.bullet_client = bullet_client
#         # self.bullet_client.setPhysicsEngineParameter(solverResidualThreshold=0)
#         #
#         # self.offset = np.array(offset)
#         #
#         # # 开启缓存
#         # flags = self.bullet_client.URDF_ENABLE_CACHED_GRAPHICS_SHAPES
#         # self.legos = []
#         #
#         # # 读取下面那个托盘
#         # # 模型，方位，旋转，启用缓存
#         # self.bullet_client.loadURDF("tray/traybox.urdf", [0 + offset[0], 0 + offset[1], -0.6 + offset[2]],
#         #                             [-0.5, -0.5, -0.5, 0.5], flags=flags)
#         #
#         # self.ground = self.bullet_client.loadURDF("plane.urdf", [0 + offset[0], 0 + offset[1], -0.6 + offset[2]],
#         #                                           [-0.5, -0.5, -0.5, 0.5], useFixedBase=True)
#         # # 添加积木,这里就是添加积木的时候没有给旋转
#         # self.legos.append(
#         #     self.bullet_client.loadURDF("lego/lego.urdf", np.array([0.1, 0.3, -0.5]) + self.offset, flags=flags))
#         #
#         # # 修改颜色
#         # self.bullet_client.changeVisualShape(self.legos[0], -1, rgbaColor=[1, 0, 0, 1])
#         # # 继续添加乐高积木
#         # self.legos.append(
#         #     self.bullet_client.loadURDF("lego/lego.urdf", np.array([-0.1, 0.3, -0.5]) + self.offset, flags=flags))
#         #
#         # self.legos.append(
#         #     self.bullet_client.loadURDF("lego/lego.urdf", np.array([0.1, 0.3, -0.7]) + self.offset, flags=flags))
#         #
#         # # 添加小球作为新的变量
#         # self.sphereId = list()
#         # self.sphereId.append(
#         #     self.bullet_client.loadURDF("sphere_small.urdf", np.array([0, 0.3, -0.6] + self.offset), flags=flags))
#         # self.sphereId.append(
#         #     self.bullet_client.loadURDF("sphere_small.urdf", np.array([0, 0.3, -0.5] + self.offset), flags=flags))
#         # self.sphereId.append(
#         #     self.bullet_client.loadURDF("sphere_small.urdf", np.array([0, 0.3, -0.7] + self.offset), flags=flags))
#         #
#         # # orn = [-0.707107, 0.0, 0.0, 0.707107]
#         # orn = [0.9, 0.0, 0.0, 0.]
#         # orn = self.bullet_client.getQuaternionFromEuler([math.radians(-90), 0., 0.])
#         # eul = self.bullet_client.getEulerFromQuaternion([-0.5, -0.5, -0.5, 0.5])
#         #
#         # # 下面重头戏，就是panda
#         # # self.panda = self.bullet_client.loadURDF("franka_panda/panda.urdf", np.array([0, 0, 0]) + self.offset, orn,useFixedBase=True, flags=flags)
#         #
#         # # todo:1 读取文件配置
#         # # -------------------------------------------------
#         # self.id = self.bullet_client.loadURDF('./myurdf/myur10_Robitq140.urdf', np.array([0, 0, 0]) + self.offset,
#         #                                       orn, useFixedBase=True, flags=flags)
#         #
#         # if self.id < 0:
#         #     raise ValueError("myur10_Robitq140文件读取异常,检查一下是不是路径或者其他的问题")
#         #
#         # # -------------------------------------------------
#         #
#         # # todo:2 读取文件的配置(需要修改)
#         # # --------------------------------------------------
#         # numJoints = self.bullet_client.getNumJoints(self.id)
#         # jointInfo = namedtuple('jointInfo',
#         #                        ['id', 'name', 'type', 'damping', 'friction', 'lowerLimit', 'upperLimit', 'maxForce',
#         #                         'maxVelocity', 'controllable'])
#         # self.joints = []
#         # self.controllable_joints = []
#         # for i in range(numJoints):
#         #     info = self.bullet_client.getJointInfo(self.id, i)
#         #     jointID = info[0]
#         #     jointName = info[1].decode("utf-8")
#         #     jointType = info[2]  # JOINT_REVOLUTE, JOINT_PRISMATIC, JOINT_SPHERICAL, JOINT_PLANAR, JOINT_FIXED
#         #     jointDamping = info[6]
#         #     jointFriction = info[7]
#         #     jointLowerLimit = info[8]
#         #     jointUpperLimit = info[9]
#         #     jointMaxForce = info[10]
#         #     jointMaxVelocity = info[11]
#         #     controllable = (jointType != self.bullet_client.JOINT_FIXED)
#         #     if controllable:
#         #         self.controllable_joints.append(jointID)
#         #         self.bullet_client.setJointMotorControl2(self.id, jointID, self.bullet_client.VELOCITY_CONTROL, targetVelocity=0, force=0)
#         #     info = jointInfo(jointID, jointName, jointType, jointDamping, jointFriction, jointLowerLimit,
#         #                      jointUpperLimit, jointMaxForce, jointMaxVelocity, controllable)
#         #     self.joints.append(info)
#         #
#         # assert len(self.controllable_joints) >= self.arm_num_dofs
#         # self.arm_controllable_joints = self.controllable_joints[:self.arm_num_dofs]
#         #
#         # self.arm_lower_limits = [info.lowerLimit for info in self.joints if info.controllable][:self.arm_num_dofs]
#         # self.arm_upper_limits = [info.upperLimit for info in self.joints if info.controllable][:self.arm_num_dofs]
#         # self.arm_joint_ranges = [info.upperLimit - info.lowerLimit for info in self.joints if info.controllable][
#         #                         :self.arm_num_dofs]
#         #
#         # # --------------------------------------------------
#         #
#         # # todo:3 创建约束,这里是需要创建约束的父节点和子节点
#         # # ------------------------------------------------
#         # mimic_parent_name = 'finger_joint'
#         # mimic_children_names = {'right_outer_knuckle_joint': -1,
#         #                         'left_inner_knuckle_joint': -1,
#         #                         'right_inner_knuckle_joint': -1,
#         #                         'left_inner_finger_joint': 1,
#         #                         'right_inner_finger_joint': 1}
#         #
#         # self.mimic_parent_id = [joint.id for joint in self.joints if joint.name == mimic_parent_name][0]
#         # self.mimic_child_multiplier = {joint.id: mimic_children_names[joint.name] for joint in self.joints if
#         #                                joint.name in mimic_children_names}
#         #
#         # for joint_id, multiplier in self.mimic_child_multiplier.items():
#         #     c = self.bullet_client.createConstraint(self.id, self.mimic_parent_id,
#         #                            self.id, joint_id,
#         #                            jointType=self.bullet_client.JOINT_GEAR,
#         #                            jointAxis=[0, 1, 0],
#         #                            parentFramePosition=[0, 0, 0],
#         #                            childFramePosition=[0, 0, 0])
#         #     self.bullet_client.changeConstraint(c, gearRatio=-multiplier, maxForce=100,erp=1)  # Note: the mysterious `erp` is of EXTREME importance
#         #
#         # # ------------------------------------------------
#         #
#         #
#         # #todo:
#         #
#         #
#         #
#         #
#         # # index = 0
#         #
#         # self.state_t = 0
#         # self.cur_state = 0
#         # self.state = 0
#         # self.control_dt = 1. / 240
#         # self.finger_target = 0
#         # self.gripper_height = 0.2
#         #
#         # # self.move_ee(self.arm_rest_poses, 'joint')
#         # # self.move_gripper(0.085)
#         # #
#         # # for j in range(self.bullet_client.getNumJoints(self.id)):
#         # #     # 消除阻尼
#         # #     self.bullet_client.changeDynamics(self.id, j, linearDamping=0, angularDamping=0)
#         # #     # 这里可以获取关节的名字和类型
#         # #     info = self.bullet_client.getJointInfo(self.id, j)
#         # #
#         # #     jointName = info[1]
#         # #     jointType = info[2]
#         # #
#         # #     print(f"id={j},jointName={info[1]},jointName={info[2]}")
#         # #     # 这里判断它的关节的种类，和前面约束的时候种类名称有点不同
#         # #     if (jointType == self.bullet_client.JOINT_PRISMATIC):
#         # #         self.bullet_client.resetJointState(self.id, j, jointPositions[index])
#         # #         index = index + 1
#         # #     # 这里是先给旋转的值好吧，后面两个就是平移的
#         # #     if jointType == self.bullet_client.JOINT_REVOLUTE:
#         # #         self.bullet_client.resetJointState(self.id, j, jointPositions[index])
#         # #         index = index + 1
#         # # self.t = 0