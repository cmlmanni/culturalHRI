# database_connection.py

from .database_factory import DatabaseConnectionFactory

# Function to get a database connection
def get_connection():
    # Create a DatabaseConnectionFactory for PostgreSQL
    factory = DatabaseConnectionFactory('postgresql')
    # Use the factory to get a database connection
    return factory.get_connection()