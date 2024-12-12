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
from PySide6.QtGui import QValidator,QDoubleValidator

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
        #初始化部分UI的参数，这里的参数需要和世界里面的参数保持一致
        self._initUiParam()


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
        # self.ui.btn_removeCable.clicked.connect(self.removeCableSlot)

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

        # region todo:缆线物理属性的设置显示
        # 这里我先对每一个显示参数设置一个验证器
        friValidator=QDoubleValidator(0,1,4)
        self.ui.le_cableFriction.setValidator(friValidator)
        lenValidator=QDoubleValidator(0,1,4)
        self.ui.le_cableLen.setValidator(lenValidator)
        diameterValidator=QDoubleValidator(0,1,4)
        self.ui.le_cableDiameter.setValidator(diameterValidator)
        massValidator=QDoubleValidator(0,1,4)
        # self.ui.le_cableMass.setValidator(massValidator)

        # 缆线这里有4个物理参数，同一使用按钮进行导入
        self.ui.btn_applyCableParam.clicked.connect(self.changeCableParamSlot)

        # endregion

    def _initUiParam(self):
        self.ui.le_cableFriction.setPlaceholderText(format(self.world.cableFriction))
        self.ui.le_cableLen.setPlaceholderText(format(self.world.cableLen))
        self.ui.le_cableDiameter.setPlaceholderText(format(self.world.cableDiameter))
        # self.ui.le_cableMass.setPlaceholderText(format(self.world.cableMass))



    # 这个我准备用来初始化物理世界
    def _initWorld(self):
        self.bulletThread=QThread(self)
        # #把物理世界移动过去
        # self.world.moveToThread(self.bulletThread)
        # 创建定时器并绑定时间间隔
        self.timer = QTimer()
        # 这里的话，需要将函数的本身而不是函数的结果添加到绑定的计时器中
        self.timer.timeout.connect(self.updateWorld)
        self.timer.start(0.03)

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

    @QtCore.Slot()
    def changeCableParamSlot(self):
        """
        每一次点击按钮，就可以对缆线的四个参数进行检查
        然后一次进行判断，去掉不合适的
        先判断是否有错误，如果数据有错误就提示并退出
        如果没有错误，就依次更新输入过值的数据
        :return:
        """
        #设置参数的字符串数据
        cableFriction=self.ui.le_cableFriction.text().strip()
        cableLen=self.ui.le_cableLen.text().strip()
        cableDiameter=self.ui.le_cableDiameter.text().strip()
        # cableMass=self.ui.le_cableMass.text().strip()
        #然后进行判断，是否存在不为空，且无法转化的数据
        if cableFriction :
            try:
                float(cableFriction)
            except ValueError:
                QMessageBox.information(self,'信息',f"摩擦系数数据值：{cableFriction},转化失败")
                return

        if cableLen:
            try:
                float(cableLen)
            except ValueError:
                QMessageBox.information(self,'信息',f"缆线长度数据值：{cableLen},转化失败")
                return

        if cableDiameter:
            try:
                float(cableDiameter)
            except ValueError:
                QMessageBox.information(self,'信息',f"缆线直径数据值：{cableDiameter},转化失败")
                return

        # if cableMass:
        #     try:
        #         float(cableMass)
        #     except ValueError:
        #         QMessageBox.information(self,'信息',f"缆线质量数据值：{cableMass},转化失败")
        #         return

        #开始更新每一个缆线的参数
        if cableFriction:
            self.setLeCableFriction(cableFriction)
        if cableLen:
            self.setLeCableLen(cableLen)
        if cableDiameter:
            self.setLeCableDiameter(cableDiameter)
        # if cableMass:
        #     self.setLeCableMass(cableMass)

        # self.world.cableLen=cableLen
        # self.world.cableDiameter=cableDiameter
        # self.world.cableMass=cableMass

    #这是通过界面更新缆线函数的时候，需要调用的函数
    def setLeCableFriction(self,cableFriction):
        self.world.cableFriction = cableFriction
        self.ui.le_cableFriction.clear()
        self.ui.le_cableFriction.setPlaceholderText(cableFriction)

    def setLeCableLen(self,cableLen):
        self.world.cableLen=cableLen
        self.ui.le_cableLen.clear()
        self.ui.le_cableLen.setPlaceholderText(cableLen)

    def setLeCableDiameter(self,cableDiameter):
        self.world.cableDiameter=cableDiameter
        self.ui.le_cableDiameter.clear()
        self.ui.le_cableDiameter.setPlaceholderText(cableDiameter)

    # def setLeCableMass(self,cableMass):
    #     self.world.cableMass=cableMass
    #     self.ui.le_cableMass.clear()
    #     self.ui.le_cableMass.setPlaceholderText(cableMass)



def run():
    app = QApplication([])
    apply_stylesheet(app, theme='light_purple.xml')
    mainWindow = MainWindow(BulletWorld())
    mainWindow.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    app = QApplication([])
    apply_stylesheet(app, theme='light_purple.xml')
    mainWindow = MainWindow(BulletWorld())
    mainWindow.show()
    sys.exit(app.exec())
