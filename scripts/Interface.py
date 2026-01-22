import sys
import random
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
import pyqtgraph as pg


class RobotGUI(QMainWindow):
    pid_updated = pyqtSignal(float, float, float)
    control_state = pyqtSignal(int)
    telemetry_updated = pyqtSignal(float, float)


    def __init__(self):
        super().__init__()

        self.setWindowTitle("Sagara Pandhalungan - Robot Control Station")
        self.setGeometry(100, 100, 1400, 800)

        self.init_ui()

    def init_ui(self):
        tabs = QTabWidget()
        tabs.setTabPosition(QTabWidget.West)   # TAB SAMPING
        self.setCentralWidget(tabs)

        # =========================
        # HOME TAB
        # =========================
        
        tab_home = QWidget()
        layout1 = QVBoxLayout()
        layout1.addWidget(QLabel("Ini isi Tab 1"))
        tab_home.setLayout(layout1)

        # =========================
        # MONITOR TAB
        # =========================
        
        tab_monitor = QWidget()
        layout1 = QVBoxLayout()
        layout1.addWidget(QLabel("Ini isi Tab 2"))
        tab_monitor.setLayout(layout1)

        # =========================
        # CONTROL TAB
        # =========================
        tab_control = QWidget()
        main_layout = QHBoxLayout(tab_control)

        self.time_data = []
        self.heading_data = []
        self.t = 0

        # ================= LEFT PANEL =================
        left_panel = QVBoxLayout()

        # ---- Path Entry ----
        path_box = QGroupBox("Path Entry")
        path_layout = QVBoxLayout()

        self.path_edit = QLineEdit("10 20 30 40 50")
        path_layout.addWidget(QLabel("Enter coordinates:"))
        path_layout.addWidget(self.path_edit)

        btns = QHBoxLayout()
        btns.addWidget(QPushButton("Load"))
        btns.addWidget(QPushButton("Add"))
        btns.addWidget(QPushButton("Clear"))
        path_layout.addLayout(btns)

        path_box.setLayout(path_layout)
        left_panel.addWidget(path_box)

        # ---- Force panel ----
        force_box = QGroupBox("Calculate Leg Forces")
        f_layout = QGridLayout()
        self.fx = QLineEdit("0")
        self.fy = QLineEdit("0")
        self.fz = QLineEdit("0")

        f_layout.addWidget(QLabel("Kp"), 0, 0)
        f_layout.addWidget(self.fx, 0, 1)
        f_layout.addWidget(QLabel("Ki"), 1, 0)
        f_layout.addWidget(self.fy, 1, 1)
        f_layout.addWidget(QLabel("Kd"), 2, 0)
        f_layout.addWidget(self.fz, 2, 1)

        self.btn_save_PID_param = QPushButton("Save")
        self.btn_save_PID_param.clicked.connect(self.save_PID_param) 
        f_layout.addWidget(self.btn_save_PID_param, 3, 0, 1, 2) 
    
        force_box.setLayout(f_layout)
        left_panel.addWidget(force_box)

        # ---- Port control ----
        port_box = QGroupBox("Port Control")
        p_layout = QVBoxLayout()
        self.port_status = QLabel("Port Closed")
        self.port_status.setStyleSheet("color:red;")

        p_layout.addWidget(QPushButton("Open Port"))
        p_layout.addWidget(QPushButton("Close Port"))
        p_layout.addWidget(self.port_status)
        port_box.setLayout(p_layout)
        left_panel.addWidget(port_box)

        # ---- Robot Control ----
        ctrl_box = QGroupBox("Robot Control")
        c_layout = QVBoxLayout()
        c_layout.addWidget(QPushButton("Start Position"))
        c_layout.addWidget(QPushButton("Torque"))
        c_layout.addWidget(QPushButton("Reset Z"))
        ctrl_box.setLayout(c_layout)
        left_panel.addWidget(ctrl_box)

        main_layout.addLayout(left_panel, 1)

        # ================= RIGHT PANEL =================
        right_panel = QVBoxLayout()

        self.plot = pg.PlotWidget(title="Robot Heading")
        self.plot.setLabel('left', 'Heading (deg)')
        self.plot.setLabel('bottom', 'Time (s)')
        self.plot.showGrid(x=True, y=True)

        self.path = self.plot.plot([], [], pen='b', symbol='o')
        self.heading_curve = self.plot.plot([], [], pen='y')

        right_panel.addWidget(self.plot)

        # ---- GO / STOP / RESET ----
        btn_layout = QHBoxLayout()
        go = QPushButton("GO")
        stop = QPushButton("STOP")
        reset = QPushButton("RESET")

        go.setStyleSheet("font-size:20px; background:lightgreen;")
        stop.setStyleSheet("font-size:20px; background:red; color:white;")
        reset.setStyleSheet("font-size:20px; background:yellow;")

        go.clicked.connect(self.start_robot)
        stop.clicked.connect(self.stop_robot)
        reset.clicked.connect(self.reset_robot)

        btn_layout.addWidget(go)
        btn_layout.addWidget(stop)
        btn_layout.addWidget(reset)

        right_panel.addLayout(btn_layout)

        main_layout.addLayout(right_panel, 3)

        

        tabs.addTab(tab_home, "Home")
        tabs.addTab(tab_control, "Control")
        tabs.addTab(tab_monitor, "Monitor")

        self.setCentralWidget(tabs)

    def start_robot(self):
        self.control_state.emit(1)

    def stop_robot(self):
        self.control_state.emit(0)
        

    def update_graph(self):
        if len(self.x) > 100:
            self.x.pop(0)
            self.y.pop(0)

        if len(self.x) == 0:
            self.x.append(0)
            self.y.append(0)
        else:
            self.x.append(self.x[-1] + random.randint(0, 5))
            self.y.append(self.y[-1] + random.randint(-5, 5))

        self.path.setData(self.x, self.y)
    
    #--- Function
    def save_PID_param(self):
        try:
            self.kp = float(self.fx.text())
            self.ki = float(self.fy.text())
            self.kd = float(self.fz.text())
            self.PID_param = {"Kp": self.kp, "Ki": self.ki, "Kd": self.kd}
            # print(self.PID_param)
            self.pid_updated.emit(self.kp, self.ki, self.kd)

        except ValueError:
            QMessageBox.warning(self, "Input Error", "Masukkan angka yang valid!")

    def update_telemetry(self, time, heading):
        self.time_data.append(time)
        self.heading_data.append(heading)
        if len(self.time_data) > 200:
            self.time_data.pop(0)
            self.heading_data.pop(0)
        self.heading_curve.setData(self.time_data, self.heading_data)

    def reset_robot(self):
        self.t = 0.0
        self.feedback = 0.0
        self.control = 0.0
        self.time_data.clear()
        self.heading_data.clear()
        self.heading_curve.setData(self.time_data, self.heading_data)
        self.control_state.emit(2)



def main():
    app = QApplication(sys.argv)
    win = RobotGUI()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
