from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel

class TabHome(QWidget):
    def __init__(self):
        super().__init__()

        layout = QVBoxLayout(self)
        title = QLabel("Sagara Pandhalungan\nRobot Control Station")
        title.setStyleSheet("font-size:22px; font-weight:bold;")
        layout.addWidget(title)
        layout.addStretch()
