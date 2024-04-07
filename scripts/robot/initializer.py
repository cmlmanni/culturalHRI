# robot_initializer.py
import json
import rospy

from ..robot.robot_database_interface import get_robot_number
from ..rasa.run_rasa import run_rasa
from .publisher_manager import PublisherManager
from .subscriber_manager import SubscriberManager

class RobotInitializer:
    def __init__(self, robot):
        self.robot = robot
        # self.publisher_manager = PublisherManager()
        # self.subscriber_manager = SubscriberManager()
        # self.create_publishers()
        # self.create_subscribers()

        # # Create publishers
        # self.publisher_manager.create_publisher('/cmd_vel', Twist)
        # self.publisher_manager.create_publisher('/say', String)
        # self.publisher_manager.create_publisher('/move_base_simple/goal', PoseStamped)
        # self.publisher_manager.create_publisher('/screen/raw_image', Image)

        # # Create subscribers
        # self.subscriber_manager.create_subscriber('/scan', LaserScan, self.scan_callback)
        # self.subscriber_manager.create_subscriber('/action', String, self.action_callback)
        # self.subscriber_manager.create_subscriber('/destination', String, self.destination_callback)
        # self.subscriber_manager.create_subscriber("/odom", Odometry, self.odom_callback)


    def initialize_robot(self):
        self.robot.robot_number = get_robot_number()
        run_rasa(self.robot.robot_number)
        print("Robot number: %d" % self.robot.robot_number)
        parameters = self.set_parameters()

    def set_parameters(self):
        rospy.logwarn("Setting parameters for robot number: %d" % self.robot.robot_number)

        # Load the parameters from the JSON file
        with open('scripts/robot/robot_parameters.json', 'r') as f:
            data = json.load(f)
        
        parameters = data['robot_parameters']
        # Get the parameters for the robot number, or use default values if the robot number is not found
        params = parameters.get(str(self.robot.robot_number), parameters.get("default"))

        for key, value in params.items():
            setattr(self.robot, key, value)

    # def create_publishers(self):
    #     self.publisher_manager.create_publisher('/cmd_vel', Twist)
    #     self.publisher_manager.create_publisher('/say', String)
    #     self.publisher_manager.create_publisher('/move_base_simple/goal', PoseStamped)
    #     self.publisher_manager.create_publisher('/screen/raw_image', Image)

    # def create_subscribers(self):
    #     self.subscriber_manager.create_subscriber('/scan', LaserScan, self.scan_callback)
    #     self.subscriber_manager.create_subscriber('/action', String, self.action_callback)
    #     self.subscriber_manager.create_subscriber('/destination', String, self.destination_callback)
    #     self.subscriber_manager.create_subscriber("/odom", Odometry, self.odom_callback)