# robot.py
import rospy
from pyhri import HRIListener
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import String, Header
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image

import tf
import os
import json

import math

import numpy as np
import cv2
from cv_bridge import CvBridge

# import sys

from .initializer import RobotInitializer
# from publisher_manager import PublisherManager
# from subscriber_manager import SubscriberManager

class Robot:
    def __init__(self, node_name="pyhri_test", reference_frame="sellion_link"):

        self.initializer = RobotInitializer(self)
        self.initializer.initialize_robot()

        self.create_publishers()
        self.create_subscribers()

        # Timer and rate for controlling the main loop
        self.timer = rospy.Timer(rospy.Duration(0.5), self.timer_callback)
        self.rate = rospy.Rate(10)

        # Variables for tracking the robot's state
        self.obstacle_in_front = False
        self.state = "searching"
        self.last_state = "searching"
        self.reached_person = False
        self.searching = False
        self.following = False

        # Variables for handling destinations
        self.destination = None
        self.previous_destination = None
        self.destination_pose = PoseStamped()

        # Variables for tracking the robot's position and orientation
        self.current_position = None
        self.current_orientation = None

        # Other variables
        self.count = 0
        self.serving_person = None

        # HRI listener for tracking persons
        self.hri = HRIListener()

        # Reference frame for the robot's movement
        self.reference_frame = reference_frame

        self.current_angular_velocity = 0

    def odom_callback(self, data):
        self.current_position = data.pose.pose.position
        self.current_orientation = data.pose.pose.orientation

    def scan_callback(self, msg):
        # Check if there's an obstacle in front of the robot
        self.obstacle_in_front = min(msg.ranges) < 1.0  # 1.0 is the distance threshold for obstacles
    
    def destination_callback(self, msg):
        print("Destination: " + msg.data)
        self.destination = msg.data

    def action_callback(self, msg):
        rospy.logwarn("Got action: %s" % msg.data)
        action_to_state = {
            "action_follow": "following",
            "action_do_not_follow": "stopped",
            "action_go_to": "moving",
            "action_offer_drink": "drinks",
            "search": "searching",
            "stop_search": "idle"
        }
        self.state = action_to_state.get(msg.data, self.state)
        if self.state == "searching":
            self.reached_person = False

    def timer_callback(self, event):
        if self.state == "searching":
            self.handle_searching_state()
        elif self.state == "following":
            self.handle_following_state()
        elif self.state == "moving":
            self.handle_moving_state()
        elif self.state == "drinks":
            self.handle_drinks_state()
        elif self.state in ["stopped", "idle", "lost"]:
            self.stop()

    def handle_searching_state(self):
        self.search_for_person()
        self.handle_persons()
        if self.reached_person:
            rospy.logwarn("State: %s" % self.state)
            if self.state == "drinks":
                self.utters(self.offerDrink)
            self.state = "following"

    def handle_following_state(self):
        self.search_for_person()
        self.handle_persons()
        if not self.reached_person:
            self.state = "lost"

    def handle_moving_state(self):
        self.moveToCertainLocation(self.destination)

    def handle_drinks_state(self):
        rospy.logwarn("Handling drinks state...")
        self.moveToCertainLocation("pantry")
        rospy.sleep(15)

        self.publish_image(self.image_path)

        rospy.logwarn("Offering drink to person: %s" % self.serving_person.id)
        self.handle_searching_state()

    def handle_persons(self):
        persons = self.hri.tracked_persons
        if persons:
            for id, person in persons.items():
                if person.face is not None:
                    self.move_towards_person(person)

    def search_for_person(self):
        # print("Searching for person...")
        twist = Twist()
        twist.angular.z = self.rotation_speed  # Adjust this value to change the rotation speed
        self.cmd_vel_pub.publish(twist)

    def set_searching(self, searching):
        self.searching = searching

    def get_transform(self, person_from_robot):
            trans = [
                person_from_robot.transform.translation.x,
                person_from_robot.transform.translation.y,
                person_from_robot.transform.translation.z,
            ]
            rot = [
                person_from_robot.transform.rotation.x,
                person_from_robot.transform.rotation.y,
                person_from_robot.transform.rotation.z,
                person_from_robot.transform.rotation.w,
            ]
            return trans, rot

    def get_inverse_transform(self, trans, rot):
        transform = transformations.concatenate_matrices(
            transformations.translation_matrix(trans),
            transformations.quaternion_matrix(rot),
        )
        return transformations.inverse_matrix(transform)

    def calculate_direction_and_distance(self, person):
        # Get the transformation from the robot to the person
        person_from_robot = person.face.gaze_transform(from_frame=self.reference_frame)
        trans, rot = self.get_transform(person_from_robot)

        # Calculate the direction towards the person in the robot's frame
        direction = [trans[0], trans[1], 0]  # Ignore z component

        # Calculate the distance to the person
        distance = math.sqrt(trans[0]**2 + trans[1]**2)

        # Calculate the angle between the direction and the robot's current heading
        yaw = math.atan2(direction[1], direction[0])  # Assumes the robot is initially facing the x direction

        return direction, distance, yaw

    def move_towards_person(self, person):
        self.serving_person = person

        direction, distance, yaw = self.calculate_direction_and_distance(person)

        # Create a Twist message
        twist = Twist()

        # If the distance is greater than the threshold, move towards the person
        if distance > self.distance_threshold:
            print("Moving towards person: " + person.id + " at distance: " + str(distance) + ".")

            # If the yaw angle is small enough, stop turning
            if abs(yaw) < 0.01:  # Change this value to adjust the precision
                twist.angular.z = 0
            else:
                twist.angular.z = 1 * yaw  # Positive because a positive yaw means a rotation to the right

            twist.linear.x = self.speed
        else:
            # print("Reached person: " + person.id + " at distance: " + str(distance) + ". Stopping.")
            twist.linear.x = 0
            # Stop turning when the robot has reached the person and the yaw is small enough
            if abs(yaw) < 0.01:  # Change this value to adjust the precision
                twist.angular.z = 0
            else:
                twist.angular.z = 1 * yaw

            self.stop()
            self.reached_person = True  # Set the state to indicate that the robot has reached a person

        # Publish the Twist message
        self.cmd_vel_pub.publish(twist)
        return distance
        
    def stop(self):
        if self.count == 0:
            self.utters(self.greeting)
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        self.cmd_vel_pub.publish(twist)

    def utters(self, msg):
        print("Saying: " + msg)
        self.say_pub.publish(msg)
        self.count += 1

    def calculate_orientation_and_twist(self, current_position, target_position, current_orientation):
        dx = target_position.x - current_position.x
        dy = target_position.y - current_position.y
        target_angle = math.atan2(dy, dx)
        
        # Convert the quaternion to euler angles
        _, _, current_angle = tf.transformations.euler_from_quaternion([current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w])

        twist_angle = target_angle - current_angle
        if twist_angle > math.pi:
            twist_angle -= 2 * math.pi
        elif twist_angle < -math.pi:
            twist_angle += 2 * math.pi

        quaternion = tf.transformations.quaternion_from_euler(0, 0, target_angle)
        return Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3]), twist_angle

    def moveToCertainLocation(self, destination):
        # Load the destinations from the JSON file
        with open('scripts/robot/destination_parameters.json', 'r') as f:
            destinations = json.load(f)

        if destination != self.previous_destination:
            rospy.logwarn("Got destination: %s" % destination)
            self.previous_destination = destination

        if destination in destinations:
            target_position = Point(**destinations[destination])
        else:
            rospy.logwarn("Unknown destination: %s" % destination)
            self.stop()
            return  # Exit the method if the destination is unknown

        current_position = self.get_current_position()
        current_orientation = self.get_current_orientation()
       
        orientation, twist_angle = self.calculate_orientation_and_twist(current_position, target_position, current_orientation)
    
        # Normalize the twist angle to the range [-pi, pi]
        while twist_angle > math.pi:
            twist_angle -= 2 * math.pi
        while twist_angle < -math.pi:
            twist_angle += 2 * math.pi

        # Create a Twist message for the rotation
        twist = Twist()
        twist.angular.z = self.rotation_speed if twist_angle >= 0 else -self.rotation_speed

        # Calculate the time required for the rotation
        rotation_time = abs(twist_angle / self.rotation_speed)

        # Publish the Twist message to rotate the robot
        self.cmd_vel_pub.publish(twist)

        # Wait for the robot to finish rotating
        rospy.sleep(rotation_time)

        # Stop the rotation
        twist.angular.z = 0
        self.cmd_vel_pub.publish(twist)

        # Create and publish the PoseStamped message for the movement
        move_base_msgs = PoseStamped(
            header=Header(frame_id="map"),
            pose=Pose(
                position=Point(x=target_position.x, y=target_position.y, z=target_position.z),
                orientation=Quaternion(x=orientation.x, y=orientation.y, z=orientation.z, w=orientation.w)
            )
        )   

        self.move_base_pub.publish(move_base_msgs)

    def get_current_position(self):
        return self.current_position

    def get_current_orientation(self):
        return self.current_orientation

    def publish_image(self, image_path):
        img_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), image_path)
        img_data = cv2.imread(img_path)

        # Ensure the image data is not empty and is a numpy array
        if img_data is not None and isinstance(img_data, np.ndarray):
            # Convert the numpy array to a ROS Image message
            image = CvBridge().cv2_to_imgmsg(img_data, "bgr8")

            # Publish the image
            self.image_pub.publish(image)
        else:
            print("Failed to read image or image data is not a numpy array")

    def create_publishers(self):
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.say_pub = rospy.Publisher('/say', String, queue_size=10)
        self.move_base_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.image_pub = rospy.Publisher('/screen/raw_image', Image, queue_size=10)
 
    def create_subscribers(self):
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.action_sub = rospy.Subscriber('/action', String, self.action_callback)
        self.destination_sub = rospy.Subscriber('/destination', String, self.destination_callback)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)

if __name__ == "__main__":
    rospy.init_node('engagement_node')
    robot = Robot()
    rospy.spin()