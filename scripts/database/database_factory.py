import psycopg2
from psycopg2.extras import DictCursor
from .config import DB_CONFIG

class DatabaseConnectionFactory:
    def __init__(self, db_type):
        self.db_type = db_type

    def get_connection(self):
        if self.db_type == 'postgresql':
            conn = psycopg2.connect(**DB_CONFIG)
            return DatabaseConnectionWrapper(conn)
        elif self.db_type == 'mysql':
            # return a connection to a MySQL database
            pass
        elif self.db_type == 'sqlite':
            # return a connection to a SQLite database
            pass
        else:
            raise ValueError(f"Unsupported database type: {self.db_type}")

class DatabaseConnectionWrapper:
    def __init__(self, conn):
        self.conn = conn

    def execute_query(self, query):
        try:
            cur = self.conn.cursor(cursor_factory=DictCursor)
            cur.execute(query)
            row = cur.fetchone()
            cur.close()
            self.conn.close()
            return row
        except psycopg2.Error as e:
            print(f"Database error: {e}")
            print("Please check your database connection.")
            return None
        except Exception as e:
            print(f"Unexpected error: {e}")
            return None