### softSim
<center>软体仿真项目</center>

### 安装运行命令

#### pip install -r requirements.txt


-----
报如下错误运行(没有运行过Qt5以上的程序)：
This application failed to start because no Qt platform plugin could be initialized. Reinstalling the application may fix this problem.

Available platform plugins are: eglfs, vkkhrdisplay, minimalegl, wayland-egl, linuxfb, vnc, xcb, offscreen, wayland, minimal.

已放弃 (核心已转储)

需运行以下命令
#### sudo apt install libxcb-cursor0

-----

报如下错误操作（OpenGL环境发生问题）：
X11 functions dynamically loaded using dlopen/dlsym OK!
X11 functions dynamically loaded using dlopen/dlsym OK!
libGL error: MESA-LOADER: failed to open swrast: /usr/lib/dri/swrast_dri.so: 无法打开共享对象文件: 没有那个文件或目录 (search paths /usr/lib/x86_64-linux-gnu/dri:\$${ORIGIN}/dri:/usr/lib/dri, suffix _dri)
libGL error: failed to load driver: swrast
Creating context
Failed to create GL 3.3 context ... using old-style GLX context
Failed to create an OpenGL context

运行以下命令(软链接路径不存在时，创建安装目录即可)
#### sudo ln -s /usr/lib/x86_64-linux-gnu/dri/swrast_dri.so /usr/lib/dri/swrast_dri.so

---
遇到如下报错：
libGL error: MESA-LOADER: failed to open swrast: /lib/x86_64-linux-gnu/libLLVM-12.so.1: undefined symbol: ffi_type_sint32, version LIBFFI_BASE_7.0 (search paths /usr/lib/x86_64-linux-gnu/dri:\$${ORIGIN}/dri:/usr/lib/dri, suffix _dri)
libGL error: failed to load driver: swrast
Creating context
Failed to create GL 3.3 context ... using old-style GLX context
Failed to create an OpenGL context
参考链接：
https://github.com/GuanxingLu/ManiGaussian/issues/2

运行以下命令（建议重启一下，或者新开一个会话）：
#### echo 'export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libffi.so.7' >> ~/.bashrc
#### source ~/.bashrc  



---
python==3.10.2解释器的运行效果图地址：
https://fastly.jsdelivr.net/gh/unishuai/PicGoImg@main/MatrixTheory202503121704450.png

![image-20250312170434133](https://fastly.jsdelivr.net/gh/unishuai/PicGoImg@main/MatrixTheory202503121704450.png)


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

2024年8月5日21:41:06   重新用添加了缆线，将点对点的约束修改为fix，然后尝试对缆线进行抓取，由于测试尝试的版本太多

2024年8月8日10:56:57   添加了桌子和凳子，然后尝试编写抓取方式对缆线进行抓取

2024年8月9日19:57:44





---

### 状态信息

手指捏东西闭合时候的状态

<img src="https://fastly.jsdelivr.net/gh/unishuai/PicGoImg@main/softSim202408082000575.png" alt="image-20240808200029481" style="zoom:50%;" />

全屏效果

<img src="https://fastly.jsdelivr.net/gh/unishuai/PicGoImg@main/softSim202408082001507.png" alt="image-20240808200107414" style="zoom:50%;" />

此时的关节（第一部分）

![image-20240808194559416](https://fastly.jsdelivr.net/gh/unishuai/PicGoImg@main/softSim202408081945463.png)

*第二部分*

![image-20240808194627001](https://fastly.jsdelivr.net/gh/unishuai/PicGoImg@main/softSim202408081946037.png)