from PyQt5.QtCore import QObject, pyqtSignal

class RobotSignals(QObject):
    pid_updated = pyqtSignal(float, float, float)
    control_state = pyqtSignal(int)
    telemetry_updated = pyqtSignal(float, float)
