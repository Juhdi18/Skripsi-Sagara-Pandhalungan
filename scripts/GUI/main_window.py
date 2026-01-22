from PyQt5.QtWidgets import QMainWindow, QTabWidget
from signals import RobotSignals
from GUI.tab_home import TabHome
from GUI.tab_control import TabControl
from GUI.tab_monitor import TabMonitor


class RobotGUI(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Sagara Pandhalungan - Robot Control Station")
        self.setGeometry(100, 100, 1400, 800)

        # ---------- SIGNAL TERPUSAT ----------
        self.signals = RobotSignals()

        self._build_ui()

    def _build_ui(self):
        tabs = QTabWidget()
        tabs.setTabPosition(QTabWidget.West)
        self.setCentralWidget(tabs)

        # ---------- TAB ----------
        self.tab_home = TabHome()
        self.tab_control = TabControl(self.signals)
        self.tab_monitor = TabMonitor()

        # ---------- CONNECT SIGNAL ----------
        self.signals.telemetry_updated.connect(
            self.tab_monitor.update_telemetry
        )

        tabs.addTab(self.tab_home, "Home")
        tabs.addTab(self.tab_control, "Control")
        tabs.addTab(self.tab_monitor, "Monitor")

        tabs.setStyleSheet("""
        QTabBar::tab {
            background: #2C3E50;
            color: white;
            padding: 12px;
            width: 140px;
        }
        QTabBar::tab:selected {
            background: #1ABC9C;
            font-weight: bold;
        }
        """)
