# subscriber_manager.py

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String

class SubscriberManager:
    def __init__(self):
        self.subscribers = {}

    def create_subscriber(self, topic, msg_type, callback):
        self.subscribers[topic] = rospy.Subscriber(topic, msg_type, callback)