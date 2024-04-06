# engagement.py
import rospy
from pyhri import HRIListener
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import String, Header
from nav_msgs.msg import Odometry

import tf
import os
import json

import math

import numpy as np
import cv2
from cv_bridge import CvBridge

import sys

from sensor_msgs.msg import LaserScan, Image

from ..robot.robot_database_interface import get_robot_number

class Engagement:
    def __init__(self, node_name="pyhri_test", reference_frame="sellion_link"):
        self.robot_number = get_robot_number()
        print("Robot number: %d" % self.robot_number)
        self.set_parameters()

        self.create_publisher('/cmd_vel', Twist, 'cmd_vel_pub')
        self.create_publisher('/say', String, 'say_pub')
        self.create_publisher('/move_base_simple/goal', PoseStamped, 'move_base_pub')
        self.create_publisher('/screen/raw_image', Image, 'image_pub')

        self.create_subscriber('/scan', LaserScan, self.scan_callback, 'scan_sub')
        self.create_subscriber('/action', String, self.action_callback, 'action_sub')
        self.create_subscriber('/destination', String, self.destination_callback, 'destination_sub')
        self.create_subscriber("/odom", Odometry, self.odom_callback, 'odom_sub')

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

    def get_parameters():
        with open('parameters.json', 'r') as f:
            parameters = json.load(f)
        return parameters

    def set_parameters(self):
        rospy.logwarn("Setting parameters for robot number: %d" % self.robot_number)

        parameters = {
            1: {"speed": 0.5, "rotation_speed": 0.25, "distance_threshold": 3.0, "greeting": "Morning mate. Fancy a cuppa?", "offerDrink": "A Nice cuppa for you! Enjoy!", "image_path": "uk.jpg"},
            2: {"speed": 1.5, "rotation_speed": 1.5, "distance_threshold": 2.0, "greeting": "哈佬！我係月鵝。請問有咩幫到你？", "offerDrink": "你想飲咩？", "image_path": "hk.jpg"},
            3: {"speed": 0.75, "rotation_speed": 0.75, "distance_threshold": 1.25, "greeting": "你好！我是嘉文。請問有咩幫到你？", "offerDrink": "你想喝啥？", "image_path": "cn.jpg"},
        }

        # Get the parameters for the robot number, or use default values if the robot number is not found
        params = parameters.get(self.robot_number, {"speed": 1.0, "rotation_speed": 0.5, "distance_threshold": 1.0, "greeting": "Hello!", "offerDrink": None, "image_path": None})

        for key, value in params.items():
            setattr(self, key, value)

    def create_publisher(self, topic, msg_type, attr_name):
        setattr(self, attr_name, rospy.Publisher(topic, msg_type, queue_size=10))

    def create_subscriber(self, topic, msg_type, callback, attr_name):
        setattr(self, attr_name, rospy.Subscriber(topic, msg_type, callback))

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
            """
            Get the translation and rotation from the person's frame to the robot's frame.

            Args:
                person_from_robot (geometry_msgs.msg.TransformStamped): The transform from the person's frame to the robot's frame.

            Returns:
                tuple: A tuple containing the translation and rotation as lists.
            """
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
            # print(trans)
            # print(rot)
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
        if destination != self.previous_destination:
            rospy.logwarn("Got destination: %s" % destination)
            self.previous_destination = destination

        if self.destination in ["kitchen", "pantry", "廚房", "厨房"]:
            # print("Hello")
            target_position = Point(x=1.105859, y=-9.634515, z=0.0)
        elif self.destination in ["entrance", "入口"]:
            # print("here")
            target_position = Point(x=-0.355893, y=-1.37417, z=0.0)
        else:
            rospy.logwarn("Unknown destination: %s" % self.destination)
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

if __name__ == "__main__":
    rospy.init_node('engagement_node')
    engagement = Engagement()
    rospy.spin()