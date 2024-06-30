import pybullet as p
import pybullet_data
import time
import ur10FreeHandSim as mySim








if __name__ == "__main__":
    physicsClient = p.connect(p.GUI) # or p.DIRECT for non-graphical version

    p.setAdditionalSearchPath(pybullet_data.getDataPath()) # optionally
    # print(pybullet_data.getDataPath())
    p.setGravity(0,0,-9.8)
    planeId = p.loadURDF("plane.urdf",basePosition=[0, 0, 0])
    robot=mySim.Ur10FreeHnadSimAuto(p,[0,0,0])
    # robot=mySim.Ur10FreeHnadSim(p,[0,0,0.0])



    # # fps = 240.
    # # timeStep = 1. / fps
    # # 设置仿真步长
    # p.setTimeStep(robot.control_dt)
    # # 设置重力
    # p.setGravity(0, 0, -9.8)
    #
    for i in range(10000):

        time.sleep(0.007)
        # robot.step([0.3, -.4,0.2],0.085,5)
        robot.step([0.3, -.4,0.2],3.14,5)
        # robot.moveArm(robot.arm_rest_poses)
        p.stepSimulation()
