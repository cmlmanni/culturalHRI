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