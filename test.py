import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QGraphicsScene, QGraphicsView, QGraphicsLineItem, \
    QVBoxLayout, QWidget, QInputDialog
from PyQt5.QtCore import Qt, QPointF


class CableManager(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Cable Manager")
        self.setGeometry(100, 100, 800, 600)

        # 创建主窗口的小部件
        self.main_widget = QWidget()
        self.setCentralWidget(self.main_widget)

        # 创建布局
        self.layout = QVBoxLayout()
        self.main_widget.setLayout(self.layout)

        # 创建按钮
        self.add_cable_button = QPushButton("Add Cable")
        self.delete_cable_button = QPushButton("Delete Cable")
        self.layout.addWidget(self.add_cable_button)
        self.layout.addWidget(self.delete_cable_button)

        # 创建图形场景和视图
        self.scene = QGraphicsScene()
        self.view = QGraphicsView(self.scene)
        self.layout.addWidget(self.view)

        # 缆线列表
        self.cables = []

        # 连接按钮事件
        self.add_cable_button.clicked.connect(self.add_cable)
        self.delete_cable_button.clicked.connect(self.delete_cable)

    def add_cable(self):
        # 获取缆线的起点和终点位置
        x1, ok1 = QInputDialog.getInt(self, "Input", "Enter x1:")
        y1, ok2 = QInputDialog.getInt(self, "Input", "Enter y1:")
        x2, ok3 = QInputDialog.getInt(self, "Input", "Enter x2:")
        y2, ok4 = QInputDialog.getInt(self, "Input", "Enter y2:")

        if ok1 and ok2 and ok3 and ok4:
            # 创建缆线
            cable = QGraphicsLineItem(x1, y1, x2, y2)
            self.scene.addItem(cable)
            self.cables.append(cable)

    def delete_cable(self):
        # 获取要删除的缆线索引
        index, ok = QInputDialog.getInt(self, "Input", "Enter cable index to delete:", min=0, max=len(self.cables) - 1)

        if ok:
            cable = self.cables.pop(index)
            self.scene.removeItem(cable)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = CableManager()
    window.show()
    sys.exit(app.exec_())
