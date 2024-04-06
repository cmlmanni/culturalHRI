#robot.py

from database_manager import get_robot_number
import simple_ros_bridge
from run_rasa import run_rasa
from engagement import Engagement

import subprocess
import os

if __name__ == "__main__":
    robot_number = get_robot_number()
    run_rasa(robot_number)
    Engagement(robot_number)
    subprocess.Popen(["gnome-terminal", "--", "/bin/bash", "-c", "python3 simple_ros_bridge.py"])    
