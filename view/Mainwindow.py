# ============================================
# @File    : Mainwindow.py
# @Date    : 2024-07-20 13:56
# @Author  : 帅宇昕
# ============================================
import sys

from PySide6.QtCore import QTimer
from PySide6 import QtCore

from PySide6.QtCore import QFile,QThread

from PySide6.QtUiTools import QUiLoader
from PySide6.QtWidgets import QApplication, QMainWindow, QPushButton, QPlainTextEdit, QMessageBox
from ui.MainWindow_ui import Ui_MainWindow
from physicsWorld.BulletWorld import BulletWorld
from qt_material import apply_stylesheet

class MainWindow(QMainWindow):
    def __init__(self,bulletWorld:BulletWorld):
        super().__init__()

        # 首先获取参数，保存物理世界
        self.world = bulletWorld
        self._initWorld()

        # todo:使用MainWindow_ui文件
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        # self.initUi()





        #控件同步
        self.ui.hSlider_gravity.valueChanged.connect(self.ui.doubleSpinBox_gravity.setValue)
        self.ui.doubleSpinBox_gravity.valueChanged.connect(self.ui.hSlider_gravity.setValue)
        #修改重力的微调框
        self.ui.doubleSpinBox_gravity.valueChanged.connect(self.setGravitySlot)

    #初始化ui参数，比如滑块滑动距离这些，
    # 应该还是从物理世界读取具体的参数信息
    # def _initUi(self):


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
    def setGravitySlot(self,value):
        # gravityChange=value
        # QMessageBox.information(self,"期望重力","重力大小为{0}".format(gravityChange))
        # # 测试成功，确实是可以使用的
        # # self.ui.label.setText(str(value))
        self.world.setWorldGravity(value)




if __name__=="__main__":
    app=QApplication([])
    apply_stylesheet(app,theme='dark_teal.xml')
    mainWindow=MainWindow(BulletWorld())
    mainWindow.show()
    sys.exit(app.exec())
