import sys
from PySide6.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QMessageBox


def on_button_click():
    """按钮点击事件"""
    QMessageBox.information(None, "消息", "按钮被点击了！", QMessageBox.StandardButton.Ok)


def main():
    # 创建应用程序对象
    app = QApplication(sys.argv)

    # 创建主窗口
    window = QWidget()
    window.setWindowTitle("PyQt6 窗口测试")
    window.setGeometry(100, 100, 300, 200)  # 设置窗口位置和大小

    # 创建按钮和布局
    layout = QVBoxLayout(window)
    button = QPushButton("点击我")
    layout.addWidget(button)

    # 绑定按钮点击事件
    button.clicked.connect(on_button_click)

    # 显示窗口
    window.show()

    # 运行应用程序事件循环
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
