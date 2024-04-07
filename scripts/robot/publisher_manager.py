# publisher_manager.py

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Image

class PublisherManager:
    def __init__(self):
        self.publishers = {}

    def create_publisher(self, topic, msg_type, queue_size=10):
        self.publishers[topic] = rospy.Publisher(topic, msg_type, queue_size)