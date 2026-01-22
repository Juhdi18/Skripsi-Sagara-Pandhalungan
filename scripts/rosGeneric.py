#!/usr/bin/env python3
import rospy
from threading import Lock


class GenericListener:
    """
    Generic ROS subscriber manager
    Bisa subscribe banyak topic & message type
    """
    def __init__(self):
        self.data = {}
        self.lock = Lock()

    def _callback(self, topic, msg):
        with self.lock:
            self.data[topic] = msg

    def subscribe(self, topic, msg_type, queue_size=10):
        rospy.Subscriber(
            topic,
            msg_type,
            lambda msg: self._callback(topic, msg),
            queue_size=queue_size
        )

    def get(self, topic):
        with self.lock:
            return self.data.get(topic, None)


class GenericPublisher:
    """
    Generic ROS publisher manager
    Bisa publish banyak topic & message type
    """
    def __init__(self):
        self.publishers = {}

    def register(self, topic, msg_type, queue_size=10):
        if topic not in self.publishers:
            self.publishers[topic] = rospy.Publisher(
                topic,
                msg_type,
                queue_size=queue_size
            )

    def publish(self, topic, msg):
        if topic in self.publishers:
            self.publishers[topic].publish(msg)


# usage

# #!/usr/bin/env python3
# import rospy
# from geometry_msgs.msg import Twist
# from ros_generic import GenericListener, GenericPublisher


# def main():
#     rospy.init_node('controller_node')

#     listener = GenericListener()
#     publisher = GenericPublisher()

#     # Subscribe topic
#     listener.subscribe('/cmd_vel_raw', Twist)

#     # Register publisher
#     publisher.register('/cmd_vel', Twist)

#     rate = rospy.Rate(10)

#     while not rospy.is_shutdown():
#         raw = listener.get('/cmd_vel_raw')

#         if raw:
#             cmd = Twist()
#             cmd.linear.x = raw.linear.x * 0.5
#             cmd.angular.z = raw.angular.z
#             publisher.publish('/cmd_vel', cmd)

#         rate.sleep()


# if __name__ == '__main__':
#     main()

