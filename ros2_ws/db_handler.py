import sqlite3

''' Initialize the database, with the table reminders '''
def init_db():
    conn = sqlite3.connect('reminders.db')
    c = conn.cursor()
    c.execute("""
        CREATE TABLE IF NOT EXISTS reminders (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            description TEXT NOT NULL,
            date DATETIME NOT NULL
        )
    """)
    conn.commit()
    conn.close()