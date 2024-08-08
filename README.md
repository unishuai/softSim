### softSim
<center>软体仿真项目</center>

---
### 关节信息
jointID:1 ,jointName:robot_shoulder_pan_joint ,jointType:JOINT_REVOLUTE
jointID:2 ,jointName:robot_shoulder_lift_joint ,jointType:JOINT_REVOLUTE
jointID:3 ,jointName:robot_elbow_joint ,jointType:JOINT_REVOLUTE
jointID:4 ,jointName:robot_wrist_1_joint ,jointType:JOINT_REVOLUTE
jointID:5 ,jointName:robot_wrist_2_joint ,jointType:JOINT_REVOLUTE
jointID:6 ,jointName:robot_wrist_3_joint ,jointType:JOINT_REVOLUTE
jointID:9 ,jointName:fi1-1joint ,jointType:JOINT_REVOLUTE
jointID:10 ,jointName:fi1-2joint ,jointType:JOINT_REVOLUTE
jointID:11 ,jointName:fi1-3joint ,jointType:JOINT_REVOLUTE
jointID:12 ,jointName:fi1-4joint ,jointType:JOINT_REVOLUTE
jointID:16 ,jointName:fi2-1joint ,jointType:JOINT_REVOLUTE
jointID:17 ,jointName:fi2-2joint ,jointType:JOINT_REVOLUTE
jointID:18 ,jointName:fi2-3joint ,jointType:JOINT_REVOLUTE
jointID:19 ,jointName:fi2-4joint ,jointType:JOINT_REVOLUTE
jointID:23 ,jointName:fi3-1joint ,jointType:JOINT_REVOLUTE
jointID:24 ,jointName:fi3-2joint ,jointType:JOINT_REVOLUTE
jointID:25 ,jointName:fi3-3joint ,jointType:JOINT_REVOLUTE
jointID:26 ,jointName:fi3-4joint ,jointType:JOINT_REVOLUTE
jointID:30 ,jointName:fi4-1joint ,jointType:JOINT_REVOLUTE
jointID:31 ,jointName:fi4-2joint ,jointType:JOINT_REVOLUTE
jointID:32 ,jointName:fi4-3joint ,jointType:JOINT_REVOLUTE
jointID:33 ,jointName:fi4-4joint ,jointType:JOINT_REVOLUTE
jointID:37 ,jointName:fi5-1joint ,jointType:JOINT_REVOLUTE
jointID:38 ,jointName:fi5-2joint ,jointType:JOINT_REVOLUTE
jointID:39 ,jointName:fi5-3joint ,jointType:JOINT_REVOLUTE
jointID:40 ,jointName:fi5-4joint ,jointType:JOINT_REVOLUTE

---
### 开发记录
2024年6月29日18:21:17  前一个模型一直存在问题，找学姐重新要了一个模型，pybullet可以读取成功
2024年6月30日15:09:34  将指抓和手臂组合在一起
2024年7月7日10:39:37   新增了一个多末端控制器的仿真类，实现指抓可以碰到待抓取物体的要求
2024年7月7日14:56:41   如果多末端控制器的话，我感觉一般般不太行
2024年7月29日15:04:31  添加了缆线、添加了Qt控制界面并实现了灵巧手弯曲控制和旋转控制
2024年8月7日21:41:06   重新用添加了缆线，将点对点的约束修改为fix，然后尝试对缆线进行抓取，由于测试尝试的版本太多


