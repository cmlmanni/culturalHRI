# Import the necessary classes and constants
from ..database.database_connection import get_connection

from .robot_factory import RobotFactory
from ..database.pg_config import DB_CONFIG

import psycopg2
import json

# Load the robot numbers from the JSON file
with open('scripts/robot/robot_parameters.json') as f:
    data = json.load(f)
ROBOT_NUMBERS = data['robot_numbers']
DEFAULT = "HK"

# Function to get an identification
def get_identification():
    # Create a RobotFactory for the desired identification type
    factory = RobotFactory('nationality')
    # Use the factory to get an identification
    identification = factory.get_identification()
    return identification

# Function to get a robot number
def get_robot_number():
    try:
        # Get a database connection
        conn = get_connection()
        # Execute a query to get the latest survey response
        row = conn.execute_query("SELECT * FROM survey1.responses WHERE id = (SELECT MAX(id) FROM survey1.responses);")

        # If the query returned a result and the identification is in the result
        if row is not None:
            # Get the robot preference from the result
            culturalBackgroundIdentification = row['culturalbackgroundidentification']
            print(f"Got Cultural Background Identification: {culturalBackgroundIdentification}")
            robot_preference = row[culturalBackgroundIdentification]
            print(f"Got Robot Preference: {robot_preference}")
            # Return the corresponding robot number
            return ROBOT_NUMBERS.get(get_identification(), {}).get(robot_preference, 1)

    except psycopg2.OperationalError as e:
        print(f"Database error: {e}")
        print("Using default robot due to database error.")

    # If the query didn't return a result, the identification is not in the result, or a database error occurred
    print(f"Using default robot: {DEFAULT}") 
    # Return the default robot number
    return ROBOT_NUMBERS.get(get_identification(), {}).get(DEFAULT, 1)