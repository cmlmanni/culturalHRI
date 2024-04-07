# database_connection.py

"""
# Database Package

This package provides classes for establishing database connections using the Factory design pattern. 

## Why Factory Pattern?

The Factory pattern allows for flexible and extensible database connection creation. It enables easy support for new types of databases by adding new subclasses for future experimentations.

## Enhancements

To enhance functionality, add new subclasses of `DatabaseConnectionFactory` and `DatabaseConnection` for new types of databases. Also, consider adding new methods to the `DatabaseConnection` class to support more database operations.
"""

from .database_factory import PostgresConnectionFactory
from .pg_config import DB_CONFIG

# Function to get a database connection
def get_connection(new_config=None):
    factory = PostgresConnectionFactory(DB_CONFIG)
    if new_config is not None:
        factory.set_config(new_config)
    return factory.get_connection()
from .database_factory import PostgresConnectionFactory
from .pg_config import DB_CONFIG

# Function to get a database connection
def get_connection(new_config=None):
    # Create a PostgresConnectionFactory
    factory = PostgresConnectionFactory(DB_CONFIG)
    # If a new configuration is provided, set the configuration
    if new_config is not None:
        factory.set_config(new_config)
    # Use the factory to get a database connection
    return factory.get_connection()