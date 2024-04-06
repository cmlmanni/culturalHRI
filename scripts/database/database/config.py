import os

# Database configuration
# Use environment variables to set the database configuration
# Local defaults are provided for development
DB_CONFIG = { 
    "host": os.getenv("DB_HOST", "localhost"), 
    "port": os.getenv("DB_PORT", 5432), 
    "dbname": os.getenv("DB_NAME", "survey_responses"), 
    "user": os.getenv("DB_USER", "postgres"), #
    "password": os.getenv("DB_PASS", "postgres"),
    "connect_timeout": 10,
    "sslmode": 'prefer'
}

