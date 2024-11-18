# ============================================
# @File    : testpybullet.py
# @Date    : 2024-11-18 下午9:19
# @Author  : 帅宇昕
# ============================================
import pybullet as p
import pybullet_data
import time


def main():
    # 启动物理引擎仿真 (GUI 界面模式)
    physics_client = p.connect(p.GUI)

    # 设置搜索路径以加载默认模型
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    # 配置物理仿真参数
    p.setGravity(0, 0, -9.8)  # 设置重力加速度
    p.setTimeStep(1 / 240)    # 设置仿真时间步长

    # 加载平面和一个随机物体
    plane_id = p.loadURDF("plane.urdf")  # 加载地面
    cube_start_pos = [0, 0, 1]           # 初始位置
    cube_start_orientation = p.getQuaternionFromEuler([0, 0, 0])  # 无旋转
    cube_id = p.loadURDF("r2d2.urdf", cube_start_pos, cube_start_orientation)

    # 开始仿真循环
    for i in range(10000000):
        p.stepSimulation()  # 执行一步仿真
        cube_pos, cube_orn = p.getBasePositionAndOrientation(cube_id)  # 获取物体位置和朝向
        print(f"Step {i}, Position: {cube_pos}, Orientation: {cube_orn}")
        time.sleep(1 / 240)  # 控制仿真速度

    # 断开仿真
    p.disconnect()


if __name__ == "__main__":
    main()
