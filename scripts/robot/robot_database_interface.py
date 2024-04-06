# Import the necessary classes and constants
from ..database.database_connection import get_connection

from .robot_factory import RobotFactory
from ..database.pg_config import DB_CONFIG

import psycopg2

# Robot numbers
# Map national culture background preferences to robot numbers
# ROBOT_NUMBERS = {
#     "nationality": {
#         "GB": 1,
#         "HK": 2,
#         "CN": 3,
#         # Add more nationalities here
#     },
#     # Add more robots here
# }
ROBOT_NUMBERS = {
    "nationality": {
        "GB": 1,
        "HK": 2,
        "CN": 3,
        # Add more nationalities here
    },
    "gender": {
        "male": 4,
        "female": 5,
        "non-binary": 6,
        # Add more genders here
    },
    "race": {
        "white": 7,
        "black": 8,
        "asian": 9,
        # Add more races here
    },
    # Add more robots here
}


DEFAULT = "GB" # Default robot

# Function to get an identification
def get_identification():
    # Create an RobotFactory for nationality
    factory = RobotFactory('nationality')
    # Use the factory to get an identification
    return factory.get_identification()

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

class RobotFactory:
    def __init__(self, identification_type):
        self.identification_type = identification_type

    def get_identification(self):
        if self.identification_type == 'nationality':
            return "nationality"
        elif self.identification_type == 'race':
            # return race identification
            pass
        elif self.identification_type == 'gender':
            # return gender identification
            pass
        else:
            raise ValueError(f"Unsupported identification type: {self.identification_type}")