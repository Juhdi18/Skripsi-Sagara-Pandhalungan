import sys
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QTimer

from Interface import RobotGUI      # GUI kamu
from PIDAlgorithm import PID        # PID Ivmech
from PyQt5.QtCore import QObject, pyqtSignal


# =========================
#   Robot Controller
# =========================

class RobotController(QObject):
    telemetry = pyqtSignal(float, float)

    def __init__(self):
        super().__init__()
        self.pid = PID(0.0, 0.0, 0.0)
        self.pid.SetPoint = 0.0
        self.feedback = 180.0
        self.control = 0.0
        self.t = 0
        self.control_state = 0

    # Terima Kp Ki Kd dari GUI
    def update_pid(self, kp, ki, kd):
        self.pid.setKp(kp)
        self.pid.setKi(ki)
        self.pid.setKd(kd)

        print("PID updated →", kp, ki, kd)
    
    def update_state(self, state):
        self.control_state = state
        print("State updated →", state)

    # Loop kontrol (dipanggil QTimer)
    def reset_robot(self):
        self.t = 0.0
        self.feedback = 180
        self.control = 0.0
        self.pid.clear()


    def control_step(self):
        if self.control_state == 1:
            self.feedback += self.control * 0.01
            self.pid.update(self.feedback)
            self.control = self.pid.output
            self.t += 0.1
        elif self.control_state == 2:
            self.reset_robot()
            

        self.telemetry.emit(self.t, self.feedback)



# =========================
#   MAIN
# =========================
app = QApplication(sys.argv)
gui = RobotGUI()
robot = RobotController()
gui.pid_updated.connect(robot.update_pid)
gui.control_state.connect(robot.update_state)
robot.telemetry.connect(gui.update_telemetry)


# Control loop 10 Hz
timer = QTimer()
timer.timeout.connect(robot.control_step)
timer.start(100)

# Tampilkan GUI
gui.show()
sys.exit(app.exec_())
