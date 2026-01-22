#!/usr/bin/env python3

# =========================
#   IMPORT
# =========================
import sys
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry

from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QObject, pyqtSignal, QTimer

from GUI.main_window import RobotGUI
from PIDAlgorithm import PID


# =========================
#   ROBOT CONTROLLER
# =========================

class RosListener:
    def __init__(self):
        self.cmd_vx = 0.0
        self.cmd_wz = 0.0
        self.distance = 0.0
        self.odom_x = 0.0

        rospy.init_node('Main_Node')

        rospy.Subscriber('/cmd_vel', Twist, self.cmd_callback)
        # rospy.Subscriber('/distance', Float32, self.dist_callback)
        # rospy.Subscriber('/odom', Odometry, self.odom_callback)

    def cmd_callback(self, msg):
        self.cmd_vx = msg.linear.x
        self.cmd_wz = msg.angular.z

    # def dist_callback(self, msg):
    #     self.distance = msg.data

    # def odom_callback(self, msg):
    #     self.odom_x = msg.pose.pose.position.x



class RobotController(QObject):
    telemetry = pyqtSignal(float, float)

    def __init__(self):
        super().__init__()

        # ---------- PID ----------
        self.pid = PID(0.0, 0.0, 0.0)
        self.pid.SetPoint = 0.0

        self.feedback = 180.0
        self.control = 0.0
        self.t = 0.0
        self.control_state = 0

        # ---------- ROS ----------
        rospy.init_node('robot_gui_node', anonymous=True)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # ===== SLOT DARI GUI =====
    def update_pid(self, kp, ki, kd):
        self.pid.setKp(kp)
        self.pid.setKi(ki)
        self.pid.setKd(kd)
        rospy.loginfo(f"PID updated → Kp={kp}, Ki={ki}, Kd={kd}")

    def update_state(self, state):
        self.control_state = state
        rospy.loginfo(f"State updated → {state}")

    # ===== RESET =====
    def reset_robot(self):
        self.t = 0.0
        self.feedback = 180.0
        self.control = 0.0
        self.pid.clear()

    # ===== LOOP KONTROL (10 Hz) =====
    def control_step(self):
        twist = Twist()

        if self.control_state == 1:
            # Simulasi feedback
            self.feedback += self.control * 0.01

            # PID update
            self.pid.update(self.feedback)
            self.control = self.pid.output
            self.t += 0.1

            # PID → cmd_vel
            twist.linear.x = max(min(self.control, 0.5), -0.5)
            twist.angular.z = 0.0

        elif self.control_state == 2:
            self.reset_robot()

        # Publish ke ROS
        self.cmd_pub.publish(twist)

        # Kirim telemetry ke GUI
        self.telemetry.emit(self.t, self.feedback)


# =========================
#   MAIN
# =========================
def main():
    app = QApplication(sys.argv)

    gui = RobotGUI()
    robot = RobotController()

    # ---------- CONNECT SIGNAL ----------
    gui.signals.pid_updated.connect(robot.update_pid)
    gui.signals.control_state.connect(robot.update_state)
    robot.telemetry.connect(gui.signals.telemetry_updated)

    # ---------- TIMER 10 Hz ----------
    timer = QTimer()
    timer.timeout.connect(robot.control_step)
    timer.start(100)  # ms

    gui.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
