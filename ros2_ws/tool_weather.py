from langchain_core.tools import tool
import os
from dotenv import load_dotenv

load_dotenv()
API_KEY = os.getenv("OPENWEATHER_API")

@tool("Obter previsão do tempo", return_direct=True)
def get_weather(location: str) -> str:
    """Retorna a previsão do tempo para uma determinada localização."""
    import requests

    location = location.replace('"', '')
    url = f"http://api.openweathermap.org/data/2.5/weather?q={location}&APPID={API_KEY}&units=metric"

    response = requests.get(url)

    if response.status_code == 200:
        data = response.json()
        weather = data["weather"][0]["description"]
        temperature = data["main"]["temp"]
        return f"A previsão do tempo em {location} é de {weather} com temperatura de {temperature}°C."
    else:
        return "Desculpe, não consegui obter a previsão do tempo."
