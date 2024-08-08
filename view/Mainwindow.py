# ============================================
# @File    : Mainwindow.py
# @Date    : 2024-07-20 13:56
# @Author  : 帅宇昕
# ============================================
import sys

from PySide6.QtCore import QTimer
from PySide6 import QtCore

from PySide6.QtCore import QFile, QThread

from PySide6.QtUiTools import QUiLoader
from PySide6.QtWidgets import QApplication, QMainWindow, QPushButton, QPlainTextEdit, QMessageBox
from ui.MainWindow_ui import Ui_MainWindow
from physicsWorld.BulletWorld import BulletWorld
from qt_material import apply_stylesheet


class MainWindow(QMainWindow):
    def __init__(self, bulletWorld: BulletWorld):
        super().__init__()

        # todo:使用MainWindow_ui文件
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        # todo:首先获取参数，保存物理世界
        self.world = bulletWorld
        self._initWorld()



        # 初始化部分参数，因为有些参数对UI会起作用
        self.bendSigList = [0, 0, 0, 0, 0]
        self.swaySigList = [0, 0, 0, 0, 0]
        # 初始化UI的控件
        self._initUi()

    # 初始化ui参数，比如滑块滑动距离这些，
    # 应该还是从物理世界读取具体的参数信息
    def _initUi(self):
        # region todo:控件同步
        # 重力控件
        self.ui.hSlider_gravity.valueChanged.connect(
            lambda value: self.ui.doubleSpinBox_gravity.setValue(value / 100)
        )
        self.ui.doubleSpinBox_gravity.valueChanged.connect(
            lambda value: self.ui.hSlider_gravity.setValue(value * 100))

        # 灵巧手弯曲控件
        self.ui.hSlider_bend_finger1.valueChanged.connect(
            lambda value: self.ui.dSpinBox_bend_finger1.setValue(value/100)
        )
        self.ui.dSpinBox_bend_finger1.valueChanged.connect(
            lambda value: self.ui.hSlider_bend_finger1.setValue(value*100)
        )

        self.ui.hSlider_bend_finger2.valueChanged.connect(
            lambda value: self.ui.dSpinBox_bend_finger2.setValue(value / 100)
        )
        self.ui.dSpinBox_bend_finger2.valueChanged.connect(
            lambda value: self.ui.hSlider_bend_finger2.setValue(value * 100)
        )

        self.ui.hSlider_bend_finger3.valueChanged.connect(
            lambda value: self.ui.dSpinBox_bend_finger3.setValue(value / 100)
        )
        self.ui.dSpinBox_bend_finger3.valueChanged.connect(
            lambda value: self.ui.hSlider_bend_finger3.setValue(value * 100)
        )

        self.ui.hSlider_bend_finger4.valueChanged.connect(
            lambda value: self.ui.dSpinBox_bend_finger4.setValue(value / 100)
        )
        self.ui.dSpinBox_bend_finger4.valueChanged.connect(
            lambda value: self.ui.hSlider_bend_finger4.setValue(value * 100)
        )

        self.ui.hSlider_bend_finger5.valueChanged.connect(
            lambda value: self.ui.dSpinBox_bend_finger5.setValue(value / 100)
        )
        self.ui.dSpinBox_bend_finger5.valueChanged.connect(
            lambda value: self.ui.hSlider_bend_finger5.setValue(value * 100)
        )

        # 灵巧手摆动控件
        self.ui.hSlider_sway_finger1.valueChanged.connect(
            lambda value: self.ui.dSpinBox_sway_finger1.setValue(value/100)
        )
        self.ui.dSpinBox_sway_finger1.valueChanged.connect(
            lambda value: self.ui.hSlider_sway_finger1.setValue(value*100)
        )

        self.ui.hSlider_sway_finger2.valueChanged.connect(
            lambda value: self.ui.dSpinBox_sway_finger2.setValue(value / 100)
        )
        self.ui.dSpinBox_sway_finger2.valueChanged.connect(
            lambda value: self.ui.hSlider_sway_finger2.setValue(value * 100)
        )

        self.ui.hSlider_sway_finger3.valueChanged.connect(
            lambda value: self.ui.dSpinBox_sway_finger3.setValue(value / 100)
        )
        self.ui.dSpinBox_sway_finger3.valueChanged.connect(
            lambda value: self.ui.hSlider_sway_finger3.setValue(value * 100)
        )

        self.ui.hSlider_sway_finger4.valueChanged.connect(
            lambda value: self.ui.dSpinBox_sway_finger4.setValue(value / 100)
        )
        self.ui.dSpinBox_sway_finger4.valueChanged.connect(
            lambda value: self.ui.hSlider_sway_finger4.setValue(value * 100)
        )

        self.ui.hSlider_sway_finger5.valueChanged.connect(
            lambda value: self.ui.dSpinBox_sway_finger5.setValue(value / 100)
        )
        self.ui.dSpinBox_sway_finger5.valueChanged.connect(
            lambda value: self.ui.hSlider_sway_finger5.setValue(value * 100)
        )


        # endregion

        # region todo:功能绑定
        # 修改重力的微调框
        self.ui.doubleSpinBox_gravity.valueChanged.connect(self.setGravitySlot)
        # 添加缆线的功能
        self.ui.btn_addCable.clicked.connect(self.addCableSlot)
        # 删除缆线的功能
        self.ui.btn_removeCable.clicked.connect(self.removeCableSlot)

        # 灵巧手的弯曲功能
        self.ui.dSpinBox_bend_finger1.valueChanged.connect(self.controlHandSlot)
        self.ui.dSpinBox_bend_finger2.valueChanged.connect(self.controlHandSlot)
        self.ui.dSpinBox_bend_finger3.valueChanged.connect(self.controlHandSlot)
        self.ui.dSpinBox_bend_finger4.valueChanged.connect(self.controlHandSlot)
        self.ui.dSpinBox_bend_finger5.valueChanged.connect(self.controlHandSlot)

        # 灵巧手的摆动功能
        self.ui.dSpinBox_sway_finger1.valueChanged.connect(self.controlHandSlot)
        self.ui.dSpinBox_sway_finger2.valueChanged.connect(self.controlHandSlot)
        self.ui.dSpinBox_sway_finger3.valueChanged.connect(self.controlHandSlot)
        self.ui.dSpinBox_sway_finger4.valueChanged.connect(self.controlHandSlot)
        self.ui.dSpinBox_sway_finger5.valueChanged.connect(self.controlHandSlot)

        #开始暂停抓取仿真功能
        self.ui.btn_startgraspSim.clicked.connect(self.startGraspSimSlot)
        self.ui.btn_stopGraspSim.clicked.connect(self.stopGraspSimSlot)



        # endregion

        # region todo:初始值设置
        self.ui.doubleSpinBox_gravity.setValue(self.getWorldGravity())
        # endregion

    # 这个我准备用来初始化物理世界
    def _initWorld(self):
        # 创建定时器并绑定时间间隔
        self.timer = QTimer()
        # 这里的话，需要将函数的本身而不是函数的结果添加到绑定的计时器中
        self.timer.timeout.connect(self.updateWorld)
        self.timer.start(1000.0 / 240.0)

    def updateWorld(self):
        # 更新世界之前先检查是否处于连接状态，当pybullet断开时，Qt界面也自动断开
        if not self.world.isConnected():
            self.close()
            return
        self.world.stepWorldSimulation()

    @QtCore.Slot(float)
    def setGravitySlot(self, value):
        # gravityChange=value
        # QMessageBox.information(self,"期望重力","重力大小为{0}".format(gravityChange))
        # # 测试成功，确实是可以使用的
        # # self.ui.label.setText(str(value))
        self.world.setWorldGravity(value)

    @QtCore.Slot()
    def addCableSlot(self):
        self.world.addCable()

    @QtCore.Slot()
    def removeCableSlot(self):
        self.world.removeCable()

    @QtCore.Slot()
    def controlHandSlot(self):
        """
        这里的话，self.bendSigList的参数范围为0.0 ~ 10.0
        self.swaySigList的参数范围为-5.0 ~ 5.0
        :return:
        """
        # 获取手指弯曲的信息
        self.bendSigList[0] = self.ui.dSpinBox_bend_finger1.value()
        self.bendSigList[1] = self.ui.dSpinBox_bend_finger2.value()
        self.bendSigList[2] = self.ui.dSpinBox_bend_finger3.value()
        self.bendSigList[3] = self.ui.dSpinBox_bend_finger4.value()
        self.bendSigList[4] = self.ui.dSpinBox_bend_finger5.value()

        # 获取手指摇摆的信息
        self.swaySigList[0] = self.ui.dSpinBox_sway_finger1.value()
        self.swaySigList[1] = self.ui.dSpinBox_sway_finger2.value()
        self.swaySigList[2] = self.ui.dSpinBox_sway_finger3.value()
        self.swaySigList[3] = self.ui.dSpinBox_sway_finger4.value()
        self.swaySigList[4] = self.ui.dSpinBox_sway_finger5.value()

        # 封装信息，然后调用bullet相关的函数
        self.world.controlHand((self.bendSigList,self.swaySigList))

    @QtCore.Slot()
    def startGraspSimSlot(self):
        self.world.startGraspSim()

    @QtCore.Slot()
    def stopGraspSimSlot(self):
        self.world.stopGraspSim()



    def getWorldGravity(self):
        return self.world.getWorldGravity()




def run():
    app = QApplication([])
    apply_stylesheet(app, theme='dark_teal.xml')
    mainWindow = MainWindow(BulletWorld())
    mainWindow.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    app = QApplication([])
    apply_stylesheet(app, theme='dark_teal.xml')
    mainWindow = MainWindow(BulletWorld())
    mainWindow.show()
    sys.exit(app.exec())
