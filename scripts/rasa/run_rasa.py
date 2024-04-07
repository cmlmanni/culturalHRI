# run_rasa.py
import os
import subprocess
import psutil
import rospy

from ..robot.robot_database_interface import get_robot_number


def is_process_running(process_name):
    for proc in psutil.process_iter(['name']):
        if process_name.lower() in proc.info['name'].lower():
            return True
    return False

def run_rasa(robot_number):
    # robot_number = robot_number

    if not is_process_running('rasa'):
        # Map robot numbers to model paths
        robots = {
            1: "hall",
            2: "yuetNgor",
            3: "jiaWen",
        }

        # Check if the robot number is valid
        if robot_number in robots:
            rospy.logwarn("Starting Rasa for robot number: %s %s" %(robot_number, robots[robot_number]))            # Run Rasa for the given robot number
            command = f"rasa shell -m {os.path.join(os.path.expanduser('rasa/'), robots[robot_number], 'models')} --endpoints {os.path.join(os.path.expanduser('rasa/'), robots[robot_number], 'endpoints.yml')}"
            subprocess.Popen(["gnome-terminal", "--", "/bin/bash", "-c", command])
        else:
            print(f"Invalid robot number: {robot_number}")
    else:
        rospy.logwarn("Rasa is already running: Robot number: %s" % robot_number)

if __name__ == "__main__":
    run_rasa(robot_number)