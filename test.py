import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QListWidget, QPushButton, QMessageBox

class MyWidget(QWidget):
    def __init__(self):
        super().__init__()

        # 创建布局
        self.layout = QVBoxLayout()

        # 创建 QListWidget
        self.list_widget = QListWidget()
        self.layout.addWidget(self.list_widget)

        # 添加一些初始项
        for i in range(10):
            self.list_widget.addItem(f"Item {i}")

        # 创建删除按钮
        self.delete_button = QPushButton("Delete Selected Item")
        self.delete_button.clicked.connect(self.delete_selected_item)
        self.layout.addWidget(self.delete_button)

        # 设置窗口布局
        self.setLayout(self.layout)
        self.setWindowTitle("Qt ListWidget Example")

    def delete_selected_item(self):
        selected_items = self.list_widget.selectedItems()
        if not selected_items:
            QMessageBox.warning(self, "Warning", "No item selected")
            return
        for item in selected_items:
            self.list_widget.takeItem(self.list_widget.row(item))

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MyWidget()
    window.show()
    sys.exit(app.exec_())
