# database_factory.py
# Define an abstract base class for a database connection and a concrete class for database connection.

from abc import ABC, abstractmethod
import psycopg2
from psycopg2.extras import DictCursor

# Abstract base class for a database connection
class DatabaseConnection(ABC):
    def __init__(self, db_config):
        # Store the database configuration
        self.db_config = db_config

    # Method to set the database configuration
    def set_config(self, db_config):
        self.db_config = db_config

    # Abstract method to get a connection
    @abstractmethod
    def get_connection(self):
        pass

    # Abstract method to execute a query
    @abstractmethod
    def execute_query(self, query):
        pass

# Concrete class for a PostgreSQL connection
class PostgresConnection(DatabaseConnection):
    def __init__(self, db_config):
        # Call the base class constructor
        super().__init__(db_config)
        # Create a PostgreSQL connection
        self.conn = psycopg2.connect(**self.db_config)

    def get_connection(self):
        return self.conn

    # Implement the abstract method to execute a query
    def execute_query(self, query):
        try:
            # Create a cursor that returns rows as dictionaries
            cur = self.conn.cursor(cursor_factory=DictCursor)
            # Execute the query
            cur.execute(query)
            # Fetch the first row of the result
            row = cur.fetchone()
            # Close the cursor and the connection
            cur.close()
            self.conn.close()
            # Return the result
            return row
        except psycopg2.Error as e:
            # Handle database errors
            print(f"Database error: {e}")
            print("Please check your database connection.")
            return None
        except Exception as e:
            # Handle other errors
            print(f"Unexpected error: {e}")
            return None

# Abstract base class for a database connection factory
class DatabaseConnectionFactory(ABC):
    # Abstract method to get a connection
    @abstractmethod
    def get_connection(self):
        pass

# Concrete class for a PostgreSQL connection factory
class PostgresConnectionFactory(DatabaseConnectionFactory):
    def __init__(self, db_config):
        # Store the database configuration
        self.db_config = db_config

    # Implement the abstract method to get a connection
    def get_connection(self):
        # Return a new PostgreSQL connection
        return PostgresConnection(self.db_config)

# Add more concrete factory classes for other types of databases as needed