from langchain_core.tools import tool
import sqlite3
import json


class ToolReminder:
    
    db_name = 'reminders.db'
    def __init__(self, db_name = 'reminders.db'):
        self.db_name = db_name

    @staticmethod
    @tool("Adicionar lembrete", return_direct=True)
    def add_reminder(data) -> str:
        """ Adiciona um lembrete ao banco de dados.
        Exemplo de entrada: {"description": "Tomar remédio", "date_time": "2025-03-04 08:00"}
        """
        # converte data para dicionário
        data = json.loads(data)


        try:
            conn = sqlite3.connect(ToolReminder.db_name)  # Usa o nome do banco da classe
            c = conn.cursor()
            c.execute(
                "INSERT INTO reminders (description, date) VALUES (?, ?)",
                (data["description"], data["date_time"])
            )
            conn.commit()
            conn.close()
            return "Lembrete adicionado com sucesso!"
        except Exception as e:
            return f"Erro ao adicionar lembrete: {str(e)}"

        
    @tool("Listar lembretes", return_direct=True)
    def list_reminders():
        """ List all reminders from the database """
        try:
            conn = sqlite3.connect(ToolReminder.db_name) 
            c = conn.cursor()
            c.execute("SELECT * FROM reminders")
            reminders = c.fetchall()
            conn.close()
            return reminders
        except Exception as e:
            return f"Erro ao listar lembretes: {str(e)}"
        
    @tool("Remover lembrete", return_direct=True)
    def remove_reminder(data) -> str:
        """ Remove lembrete do banco de dados, recebe o lembrete e suas informações e remove aquele que corresponde aos dados passados.
        Exemplo de entrada: {"description": "Tomar remédio", "date_time": "2025-03-04 08:00"}
        """
        data = json.loads(data)

        try:
            conn = sqlite3.connect(ToolReminder.db_name) 
            c = conn.cursor()
            c.execute(
                "DELETE FROM reminders WHERE description = ? AND date = ?",
                (data["description"], data["date_time"])
            )
            conn.commit()
            conn.close()
            return "Lembrete removido com sucesso!"
        except Exception as e:
            return f"Erro ao remover lembrete: {str(e)}"