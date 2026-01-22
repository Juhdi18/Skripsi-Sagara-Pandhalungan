from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel
import pyqtgraph as pg

class TabMonitor(QWidget):
    def __init__(self):
        super().__init__()

        self.time_data = []
        self.heading_data = []

        layout = QVBoxLayout(self)

        self.plot = pg.PlotWidget(title="Robot Heading")
        self.plot.setLabel('left', 'Heading (deg)')
        self.plot.setLabel('bottom', 'Time (s)')
        self.plot.showGrid(x=True, y=True)

        self.heading_curve = self.plot.plot([], [], pen='y')
        layout.addWidget(self.plot)

    def update_telemetry(self, time, heading):
        self.time_data.append(time)
        self.heading_data.append(heading)
        if len(self.time_data) > 200:
            self.time_data.pop(0)
            self.heading_data.pop(0)
        self.heading_curve.setData(self.time_data, self.heading_data)
