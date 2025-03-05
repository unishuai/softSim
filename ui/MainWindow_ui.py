# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'MainWindow.ui'
##
## Created by: Qt User Interface Compiler version 6.6.3
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QApplication, QDoubleSpinBox, QGridLayout, QGroupBox,
    QHBoxLayout, QLabel, QLineEdit, QMainWindow,
    QMenuBar, QPushButton, QSizePolicy, QSlider,
    QSpacerItem, QVBoxLayout, QWidget)
import resource.img_rc

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(928, 585)
        icon = QIcon()
        icon.addFile(u":/Satoru02.jpg", QSize(), QIcon.Normal, QIcon.Off)
        MainWindow.setWindowIcon(icon)
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.horizontalLayout_7 = QHBoxLayout(self.centralwidget)
        self.horizontalLayout_7.setObjectName(u"horizontalLayout_7")
        self.horizontalSpacer_3 = QSpacerItem(234, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.horizontalLayout_7.addItem(self.horizontalSpacer_3)

        self.horizontalLayout = QHBoxLayout()
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.groupBox_4 = QGroupBox(self.centralwidget)
        self.groupBox_4.setObjectName(u"groupBox_4")
        self.verticalLayout_10 = QVBoxLayout(self.groupBox_4)
        self.verticalLayout_10.setObjectName(u"verticalLayout_10")
        self.groupBox = QGroupBox(self.groupBox_4)
        self.groupBox.setObjectName(u"groupBox")
        self.verticalLayout_4 = QVBoxLayout(self.groupBox)
        self.verticalLayout_4.setObjectName(u"verticalLayout_4")
        self.gridLayout = QGridLayout()
        self.gridLayout.setObjectName(u"gridLayout")
        self.label_2 = QLabel(self.groupBox)
        self.label_2.setObjectName(u"label_2")

        self.gridLayout.addWidget(self.label_2, 0, 0, 1, 1)

        self.hSlider_bend_finger1 = QSlider(self.groupBox)
        self.hSlider_bend_finger1.setObjectName(u"hSlider_bend_finger1")
        self.hSlider_bend_finger1.setMinimumSize(QSize(60, 0))
        self.hSlider_bend_finger1.setMaximum(1000)
        self.hSlider_bend_finger1.setOrientation(Qt.Horizontal)

        self.gridLayout.addWidget(self.hSlider_bend_finger1, 0, 2, 1, 1)

        self.label_3 = QLabel(self.groupBox)
        self.label_3.setObjectName(u"label_3")

        self.gridLayout.addWidget(self.label_3, 1, 0, 1, 1)

        self.hSlider_bend_finger2 = QSlider(self.groupBox)
        self.hSlider_bend_finger2.setObjectName(u"hSlider_bend_finger2")
        self.hSlider_bend_finger2.setMinimumSize(QSize(60, 0))
        self.hSlider_bend_finger2.setMaximum(1000)
        self.hSlider_bend_finger2.setOrientation(Qt.Horizontal)

        self.gridLayout.addWidget(self.hSlider_bend_finger2, 1, 2, 1, 1)

        self.label_4 = QLabel(self.groupBox)
        self.label_4.setObjectName(u"label_4")

        self.gridLayout.addWidget(self.label_4, 2, 0, 1, 1)

        self.hSlider_bend_finger3 = QSlider(self.groupBox)
        self.hSlider_bend_finger3.setObjectName(u"hSlider_bend_finger3")
        self.hSlider_bend_finger3.setMinimumSize(QSize(60, 0))
        self.hSlider_bend_finger3.setMaximum(1000)
        self.hSlider_bend_finger3.setOrientation(Qt.Horizontal)

        self.gridLayout.addWidget(self.hSlider_bend_finger3, 2, 2, 1, 1)

        self.label_5 = QLabel(self.groupBox)
        self.label_5.setObjectName(u"label_5")

        self.gridLayout.addWidget(self.label_5, 3, 0, 1, 1)

        self.hSlider_bend_finger4 = QSlider(self.groupBox)
        self.hSlider_bend_finger4.setObjectName(u"hSlider_bend_finger4")
        self.hSlider_bend_finger4.setMinimumSize(QSize(60, 0))
        self.hSlider_bend_finger4.setMaximum(1000)
        self.hSlider_bend_finger4.setOrientation(Qt.Horizontal)

        self.gridLayout.addWidget(self.hSlider_bend_finger4, 3, 2, 1, 1)

        self.label_6 = QLabel(self.groupBox)
        self.label_6.setObjectName(u"label_6")

        self.gridLayout.addWidget(self.label_6, 4, 0, 1, 1)

        self.hSlider_bend_finger5 = QSlider(self.groupBox)
        self.hSlider_bend_finger5.setObjectName(u"hSlider_bend_finger5")
        self.hSlider_bend_finger5.setMinimumSize(QSize(60, 0))
        self.hSlider_bend_finger5.setMaximum(1000)
        self.hSlider_bend_finger5.setOrientation(Qt.Horizontal)

        self.gridLayout.addWidget(self.hSlider_bend_finger5, 4, 2, 1, 1)

        self.dSpinBox_bend_finger1 = QDoubleSpinBox(self.groupBox)
        self.dSpinBox_bend_finger1.setObjectName(u"dSpinBox_bend_finger1")
        self.dSpinBox_bend_finger1.setMinimumSize(QSize(50, 0))
        self.dSpinBox_bend_finger1.setMaximum(10.000000000000000)

        self.gridLayout.addWidget(self.dSpinBox_bend_finger1, 0, 1, 1, 1)

        self.dSpinBox_bend_finger2 = QDoubleSpinBox(self.groupBox)
        self.dSpinBox_bend_finger2.setObjectName(u"dSpinBox_bend_finger2")
        self.dSpinBox_bend_finger2.setMinimumSize(QSize(50, 0))
        self.dSpinBox_bend_finger2.setMaximum(10.000000000000000)

        self.gridLayout.addWidget(self.dSpinBox_bend_finger2, 1, 1, 1, 1)

        self.dSpinBox_bend_finger3 = QDoubleSpinBox(self.groupBox)
        self.dSpinBox_bend_finger3.setObjectName(u"dSpinBox_bend_finger3")
        self.dSpinBox_bend_finger3.setMinimumSize(QSize(50, 0))
        self.dSpinBox_bend_finger3.setMaximum(10.000000000000000)

        self.gridLayout.addWidget(self.dSpinBox_bend_finger3, 2, 1, 1, 1)

        self.dSpinBox_bend_finger4 = QDoubleSpinBox(self.groupBox)
        self.dSpinBox_bend_finger4.setObjectName(u"dSpinBox_bend_finger4")
        self.dSpinBox_bend_finger4.setMinimumSize(QSize(50, 0))
        self.dSpinBox_bend_finger4.setMaximum(10.000000000000000)

        self.gridLayout.addWidget(self.dSpinBox_bend_finger4, 3, 1, 1, 1)

        self.dSpinBox_bend_finger5 = QDoubleSpinBox(self.groupBox)
        self.dSpinBox_bend_finger5.setObjectName(u"dSpinBox_bend_finger5")
        self.dSpinBox_bend_finger5.setMinimumSize(QSize(50, 0))
        self.dSpinBox_bend_finger5.setMaximum(10.000000000000000)

        self.gridLayout.addWidget(self.dSpinBox_bend_finger5, 4, 1, 1, 1)


        self.verticalLayout_4.addLayout(self.gridLayout)


        self.verticalLayout_10.addWidget(self.groupBox)

        self.groupBox_2 = QGroupBox(self.groupBox_4)
        self.groupBox_2.setObjectName(u"groupBox_2")
        self.verticalLayout_5 = QVBoxLayout(self.groupBox_2)
        self.verticalLayout_5.setObjectName(u"verticalLayout_5")
        self.gridLayout_2 = QGridLayout()
        self.gridLayout_2.setObjectName(u"gridLayout_2")
        self.label_7 = QLabel(self.groupBox_2)
        self.label_7.setObjectName(u"label_7")

        self.gridLayout_2.addWidget(self.label_7, 0, 0, 1, 1)

        self.hSlider_sway_finger1 = QSlider(self.groupBox_2)
        self.hSlider_sway_finger1.setObjectName(u"hSlider_sway_finger1")
        self.hSlider_sway_finger1.setMinimumSize(QSize(60, 0))
        self.hSlider_sway_finger1.setMinimum(-500)
        self.hSlider_sway_finger1.setMaximum(500)
        self.hSlider_sway_finger1.setOrientation(Qt.Horizontal)

        self.gridLayout_2.addWidget(self.hSlider_sway_finger1, 0, 2, 1, 1)

        self.label_8 = QLabel(self.groupBox_2)
        self.label_8.setObjectName(u"label_8")

        self.gridLayout_2.addWidget(self.label_8, 1, 0, 1, 1)

        self.hSlider_sway_finger2 = QSlider(self.groupBox_2)
        self.hSlider_sway_finger2.setObjectName(u"hSlider_sway_finger2")
        self.hSlider_sway_finger2.setMinimumSize(QSize(60, 0))
        self.hSlider_sway_finger2.setMinimum(-500)
        self.hSlider_sway_finger2.setMaximum(500)
        self.hSlider_sway_finger2.setOrientation(Qt.Horizontal)

        self.gridLayout_2.addWidget(self.hSlider_sway_finger2, 1, 2, 1, 1)

        self.label_9 = QLabel(self.groupBox_2)
        self.label_9.setObjectName(u"label_9")

        self.gridLayout_2.addWidget(self.label_9, 2, 0, 1, 1)

        self.hSlider_sway_finger3 = QSlider(self.groupBox_2)
        self.hSlider_sway_finger3.setObjectName(u"hSlider_sway_finger3")
        self.hSlider_sway_finger3.setMinimumSize(QSize(60, 0))
        self.hSlider_sway_finger3.setMinimum(-500)
        self.hSlider_sway_finger3.setMaximum(500)
        self.hSlider_sway_finger3.setOrientation(Qt.Horizontal)

        self.gridLayout_2.addWidget(self.hSlider_sway_finger3, 2, 2, 1, 1)

        self.label_10 = QLabel(self.groupBox_2)
        self.label_10.setObjectName(u"label_10")

        self.gridLayout_2.addWidget(self.label_10, 3, 0, 1, 1)

        self.hSlider_sway_finger4 = QSlider(self.groupBox_2)
        self.hSlider_sway_finger4.setObjectName(u"hSlider_sway_finger4")
        self.hSlider_sway_finger4.setMinimumSize(QSize(60, 0))
        self.hSlider_sway_finger4.setMinimum(-500)
        self.hSlider_sway_finger4.setMaximum(500)
        self.hSlider_sway_finger4.setValue(0)
        self.hSlider_sway_finger4.setOrientation(Qt.Horizontal)

        self.gridLayout_2.addWidget(self.hSlider_sway_finger4, 3, 2, 1, 1)

        self.label_11 = QLabel(self.groupBox_2)
        self.label_11.setObjectName(u"label_11")

        self.gridLayout_2.addWidget(self.label_11, 4, 0, 1, 1)

        self.hSlider_sway_finger5 = QSlider(self.groupBox_2)
        self.hSlider_sway_finger5.setObjectName(u"hSlider_sway_finger5")
        self.hSlider_sway_finger5.setMinimumSize(QSize(60, 0))
        self.hSlider_sway_finger5.setMinimum(-500)
        self.hSlider_sway_finger5.setMaximum(500)
        self.hSlider_sway_finger5.setValue(0)
        self.hSlider_sway_finger5.setOrientation(Qt.Horizontal)

        self.gridLayout_2.addWidget(self.hSlider_sway_finger5, 4, 2, 1, 1)

        self.dSpinBox_sway_finger1 = QDoubleSpinBox(self.groupBox_2)
        self.dSpinBox_sway_finger1.setObjectName(u"dSpinBox_sway_finger1")
        self.dSpinBox_sway_finger1.setMinimumSize(QSize(50, 0))
        self.dSpinBox_sway_finger1.setMinimum(-5.000000000000000)
        self.dSpinBox_sway_finger1.setMaximum(5.000000000000000)

        self.gridLayout_2.addWidget(self.dSpinBox_sway_finger1, 0, 1, 1, 1)

        self.dSpinBox_sway_finger2 = QDoubleSpinBox(self.groupBox_2)
        self.dSpinBox_sway_finger2.setObjectName(u"dSpinBox_sway_finger2")
        self.dSpinBox_sway_finger2.setMinimumSize(QSize(50, 0))
        self.dSpinBox_sway_finger2.setMinimum(-5.000000000000000)
        self.dSpinBox_sway_finger2.setMaximum(5.000000000000000)

        self.gridLayout_2.addWidget(self.dSpinBox_sway_finger2, 1, 1, 1, 1)

        self.dSpinBox_sway_finger3 = QDoubleSpinBox(self.groupBox_2)
        self.dSpinBox_sway_finger3.setObjectName(u"dSpinBox_sway_finger3")
        self.dSpinBox_sway_finger3.setMinimumSize(QSize(50, 0))
        self.dSpinBox_sway_finger3.setMinimum(-5.000000000000000)
        self.dSpinBox_sway_finger3.setMaximum(5.000000000000000)

        self.gridLayout_2.addWidget(self.dSpinBox_sway_finger3, 2, 1, 1, 1)

        self.dSpinBox_sway_finger4 = QDoubleSpinBox(self.groupBox_2)
        self.dSpinBox_sway_finger4.setObjectName(u"dSpinBox_sway_finger4")
        self.dSpinBox_sway_finger4.setMinimumSize(QSize(50, 0))
        self.dSpinBox_sway_finger4.setMinimum(-5.000000000000000)
        self.dSpinBox_sway_finger4.setMaximum(5.000000000000000)

        self.gridLayout_2.addWidget(self.dSpinBox_sway_finger4, 3, 1, 1, 1)

        self.dSpinBox_sway_finger5 = QDoubleSpinBox(self.groupBox_2)
        self.dSpinBox_sway_finger5.setObjectName(u"dSpinBox_sway_finger5")
        self.dSpinBox_sway_finger5.setMinimumSize(QSize(50, 0))
        self.dSpinBox_sway_finger5.setMinimum(-5.000000000000000)
        self.dSpinBox_sway_finger5.setMaximum(5.000000000000000)

        self.gridLayout_2.addWidget(self.dSpinBox_sway_finger5, 4, 1, 1, 1)


        self.verticalLayout_5.addLayout(self.gridLayout_2)


        self.verticalLayout_10.addWidget(self.groupBox_2)


        self.horizontalLayout.addWidget(self.groupBox_4)

        self.verticalLayout_3 = QVBoxLayout()
        self.verticalLayout_3.setObjectName(u"verticalLayout_3")
        self.groupBox_5 = QGroupBox(self.centralwidget)
        self.groupBox_5.setObjectName(u"groupBox_5")
        self.verticalLayout_2 = QVBoxLayout(self.groupBox_5)
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")
        self.horizontalLayout_3 = QHBoxLayout()
        self.horizontalLayout_3.setObjectName(u"horizontalLayout_3")
        self.horizontalLayout_2 = QHBoxLayout()
        self.horizontalLayout_2.setObjectName(u"horizontalLayout_2")
        self.label = QLabel(self.groupBox_5)
        self.label.setObjectName(u"label")
        sizePolicy = QSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label.sizePolicy().hasHeightForWidth())
        self.label.setSizePolicy(sizePolicy)

        self.horizontalLayout_2.addWidget(self.label)

        self.doubleSpinBox_gravity = QDoubleSpinBox(self.groupBox_5)
        self.doubleSpinBox_gravity.setObjectName(u"doubleSpinBox_gravity")
        self.doubleSpinBox_gravity.setMinimum(-20.000000000000000)
        self.doubleSpinBox_gravity.setMaximum(20.000000000000000)

        self.horizontalLayout_2.addWidget(self.doubleSpinBox_gravity)

        self.hSlider_gravity = QSlider(self.groupBox_5)
        self.hSlider_gravity.setObjectName(u"hSlider_gravity")
        self.hSlider_gravity.setMinimumSize(QSize(30, 0))
        self.hSlider_gravity.setMaximumSize(QSize(80, 16777215))
        self.hSlider_gravity.setMinimum(-2000)
        self.hSlider_gravity.setMaximum(2000)
        self.hSlider_gravity.setOrientation(Qt.Horizontal)

        self.horizontalLayout_2.addWidget(self.hSlider_gravity)


        self.horizontalLayout_3.addLayout(self.horizontalLayout_2)

        self.horizontalSpacer = QSpacerItem(40, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.horizontalLayout_3.addItem(self.horizontalSpacer)


        self.verticalLayout_2.addLayout(self.horizontalLayout_3)

        self.groupBox_3 = QGroupBox(self.groupBox_5)
        self.groupBox_3.setObjectName(u"groupBox_3")
        sizePolicy1 = QSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Preferred)
        sizePolicy1.setHorizontalStretch(0)
        sizePolicy1.setVerticalStretch(2)
        sizePolicy1.setHeightForWidth(self.groupBox_3.sizePolicy().hasHeightForWidth())
        self.groupBox_3.setSizePolicy(sizePolicy1)
        self.groupBox_3.setMinimumSize(QSize(60, 120))
        self.verticalLayout_6 = QVBoxLayout(self.groupBox_3)
        self.verticalLayout_6.setObjectName(u"verticalLayout_6")
        self.gridLayout_3 = QGridLayout()
        self.gridLayout_3.setObjectName(u"gridLayout_3")
        self.label_12 = QLabel(self.groupBox_3)
        self.label_12.setObjectName(u"label_12")

        self.gridLayout_3.addWidget(self.label_12, 0, 0, 1, 1)

        self.le_cableFriction = QLineEdit(self.groupBox_3)
        self.le_cableFriction.setObjectName(u"le_cableFriction")
        self.le_cableFriction.setMinimumSize(QSize(60, 0))

        self.gridLayout_3.addWidget(self.le_cableFriction, 0, 1, 1, 1)

        self.label_13 = QLabel(self.groupBox_3)
        self.label_13.setObjectName(u"label_13")

        self.gridLayout_3.addWidget(self.label_13, 0, 2, 1, 1)

        self.le_cableLen = QLineEdit(self.groupBox_3)
        self.le_cableLen.setObjectName(u"le_cableLen")
        self.le_cableLen.setMinimumSize(QSize(60, 0))

        self.gridLayout_3.addWidget(self.le_cableLen, 0, 3, 1, 1)

        self.label_15 = QLabel(self.groupBox_3)
        self.label_15.setObjectName(u"label_15")

        self.gridLayout_3.addWidget(self.label_15, 1, 0, 1, 1)

        self.le_cableDiameter = QLineEdit(self.groupBox_3)
        self.le_cableDiameter.setObjectName(u"le_cableDiameter")
        self.le_cableDiameter.setMinimumSize(QSize(60, 0))

        self.gridLayout_3.addWidget(self.le_cableDiameter, 1, 1, 1, 1)


        self.verticalLayout_6.addLayout(self.gridLayout_3)

        self.btn_applyCableParam = QPushButton(self.groupBox_3)
        self.btn_applyCableParam.setObjectName(u"btn_applyCableParam")

        self.verticalLayout_6.addWidget(self.btn_applyCableParam)

        self.btn_addCable = QPushButton(self.groupBox_3)
        self.btn_addCable.setObjectName(u"btn_addCable")

        self.verticalLayout_6.addWidget(self.btn_addCable)

        self.btn_startgraspSim = QPushButton(self.groupBox_3)
        self.btn_startgraspSim.setObjectName(u"btn_startgraspSim")

        self.verticalLayout_6.addWidget(self.btn_startgraspSim)

        self.btn_stopGraspSim = QPushButton(self.groupBox_3)
        self.btn_stopGraspSim.setObjectName(u"btn_stopGraspSim")

        self.verticalLayout_6.addWidget(self.btn_stopGraspSim)


        self.verticalLayout_2.addWidget(self.groupBox_3)


        self.verticalLayout_3.addWidget(self.groupBox_5)

        self.verticalSpacer = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.verticalLayout_3.addItem(self.verticalSpacer)


        self.horizontalLayout.addLayout(self.verticalLayout_3)

        self.horizontalLayout.setStretch(0, 2)
        self.horizontalLayout.setStretch(1, 2)

        self.horizontalLayout_7.addLayout(self.horizontalLayout)

        self.horizontalSpacer_4 = QSpacerItem(233, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.horizontalLayout_7.addItem(self.horizontalSpacer_4)

        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QMenuBar(MainWindow)
        self.menubar.setObjectName(u"menubar")
        self.menubar.setGeometry(QRect(0, 0, 928, 27))
        MainWindow.setMenuBar(self.menubar)

        self.retranslateUi(MainWindow)

        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"\u8f6f\u4f53\u4eff\u771f\u5efa\u6a21\u8f6f\u4ef6", None))
        self.groupBox_4.setTitle(QCoreApplication.translate("MainWindow", u"\u6307\u6293\u63a7\u5236", None))
        self.groupBox.setTitle(QCoreApplication.translate("MainWindow", u"\u5f2f\u66f2\u63a7\u5236", None))
        self.label_2.setText(QCoreApplication.translate("MainWindow", u"\u624b\u63071", None))
        self.label_3.setText(QCoreApplication.translate("MainWindow", u"\u624b\u63072", None))
        self.label_4.setText(QCoreApplication.translate("MainWindow", u"\u624b\u63073", None))
        self.label_5.setText(QCoreApplication.translate("MainWindow", u"\u624b\u63074", None))
        self.label_6.setText(QCoreApplication.translate("MainWindow", u"\u624b\u63075", None))
        self.groupBox_2.setTitle(QCoreApplication.translate("MainWindow", u"\u6446\u52a8\u63a7\u5236", None))
        self.label_7.setText(QCoreApplication.translate("MainWindow", u"\u624b\u63071", None))
        self.label_8.setText(QCoreApplication.translate("MainWindow", u"\u624b\u63072", None))
        self.label_9.setText(QCoreApplication.translate("MainWindow", u"\u624b\u63073", None))
        self.label_10.setText(QCoreApplication.translate("MainWindow", u"\u624b\u63074", None))
        self.label_11.setText(QCoreApplication.translate("MainWindow", u"\u624b\u63075", None))
        self.groupBox_5.setTitle(QCoreApplication.translate("MainWindow", u"\u53c2\u6570\u8bbe\u7f6e", None))
        self.label.setText(QCoreApplication.translate("MainWindow", u"\u91cd\u529b\u5927\u5c0f", None))
        self.groupBox_3.setTitle(QCoreApplication.translate("MainWindow", u"\u7f06\u7ebf\u53c2\u6570", None))
        self.label_12.setText(QCoreApplication.translate("MainWindow", u"\u6469\u64e6\u7cfb\u6570\uff1a", None))
        self.label_13.setText(QCoreApplication.translate("MainWindow", u"\u7f06\u7ebf\u957f\u5ea6\uff1a", None))
        self.label_15.setText(QCoreApplication.translate("MainWindow", u"\u7f06\u7ebf\u76f4\u5f84\uff1a", None))
        self.btn_applyCableParam.setText(QCoreApplication.translate("MainWindow", u"\u5e94\u7528\u53c2\u6570", None))
        self.btn_addCable.setText(QCoreApplication.translate("MainWindow", u"\u6dfb\u52a0\u7f06\u7ebf", None))
        self.btn_startgraspSim.setText(QCoreApplication.translate("MainWindow", u"\u5f00\u59cb\u6293\u53d6\u4eff\u771f", None))
        self.btn_stopGraspSim.setText(QCoreApplication.translate("MainWindow", u"\u505c\u6b62\u6293\u53d6\u4eff\u771f", None))
    # retranslateUi

