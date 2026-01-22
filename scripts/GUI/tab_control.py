from PyQt5.QtWidgets import *
import pyqtgraph as pg

class TabControl(QWidget):
    def __init__(self, signals):
        super().__init__()
        self.signals = signals

        self._build_ui()

    def _build_ui(self):
        main_layout = QHBoxLayout(self)

        # ================= LEFT PANEL =================
        left_panel = QVBoxLayout()

        # ---- Path Entry ----
        path_box = QGroupBox("Path Entry")
        path_layout = QVBoxLayout(path_box)
        self.path_edit = QLineEdit("10 20 30 40 50")

        path_layout.addWidget(QLabel("Enter coordinates:"))
        path_layout.addWidget(self.path_edit)

        btns = QHBoxLayout()
        btns.addWidget(QPushButton("Load"))
        btns.addWidget(QPushButton("Add"))
        btns.addWidget(QPushButton("Clear"))
        path_layout.addLayout(btns)

        left_panel.addWidget(path_box)

        # ---- PID ----
        pid_box = QGroupBox("PID Control")
        pid_layout = QGridLayout(pid_box)

        self.kp = QLineEdit("0")
        self.ki = QLineEdit("0")
        self.kd = QLineEdit("0")

        pid_layout.addWidget(QLabel("Kp"), 0, 0)
        pid_layout.addWidget(self.kp, 0, 1)
        pid_layout.addWidget(QLabel("Ki"), 1, 0)
        pid_layout.addWidget(self.ki, 1, 1)
        pid_layout.addWidget(QLabel("Kd"), 2, 0)
        pid_layout.addWidget(self.kd, 2, 1)

        btn_save = QPushButton("Save PID")
        btn_save.clicked.connect(self.save_pid)
        pid_layout.addWidget(btn_save, 3, 0, 1, 2)

        left_panel.addWidget(pid_box)

        # ---- Robot Control ----
        ctrl_box = QGroupBox("Robot Control")
        ctrl_layout = QVBoxLayout(ctrl_box)

        btn_start = QPushButton("GO")
        btn_stop = QPushButton("STOP")
        btn_reset = QPushButton("RESET")

        btn_start.clicked.connect(lambda: self.signals.control_state.emit(1))
        btn_stop.clicked.connect(lambda: self.signals.control_state.emit(0))
        btn_reset.clicked.connect(lambda: self.signals.control_state.emit(2))

        ctrl_layout.addWidget(btn_start)
        ctrl_layout.addWidget(btn_stop)
        ctrl_layout.addWidget(btn_reset)

        left_panel.addWidget(ctrl_box)
        left_panel.addStretch()

        main_layout.addLayout(left_panel, 1)

        # ================= RIGHT PANEL =================
        right_panel = QVBoxLayout()

        self.plot = pg.PlotWidget(title="Heading Preview")
        self.curve = self.plot.plot([], [], pen='b')
        right_panel.addWidget(self.plot)

        main_layout.addLayout(right_panel, 3)

    def save_pid(self):
        try:
            kp = float(self.kp.text())
            ki = float(self.ki.text())
            kd = float(self.kd.text())
            self.signals.pid_updated.emit(kp, ki, kd)
        except ValueError:
            QMessageBox.warning(self, "Error", "PID harus angka")
