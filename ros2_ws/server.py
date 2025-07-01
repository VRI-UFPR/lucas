from fastapi import FastAPI
from pydantic import BaseModel
from dotenv import load_dotenv
import db_handler
import os 
import datetime
from tool_reminder import ToolReminder
from langchain.agents import Tool
from langchain.agents import ZeroShotAgent, Tool, AgentExecutor
from langchain.chains import LLMChain
from langchain.memory import ConversationBufferMemory
import tool_weather
from langchain.llms import Ollama

app = FastAPI()

class AudioRequest(BaseModel):
    user_input: str
    speaker_name: str

load_dotenv()
api = os.getenv("DEEPSEEK_API_KEY")

llm = Ollama(model="llama3:8b")

db_handler.init_db()
reminder = ToolReminder()

tools = [
    Tool(
        name="Adicionar lembrete",
        func=reminder.add_reminder,
        description="Ferramenta para adicionar lembretes ao banco de dados utilizando uma descrição e uma data/hora, a estrutura deverá ser um json com as chaves 'description' e 'date_time'"
    ),
    Tool(
        name="Listar lembretes",
        func=reminder.list_reminders,
        description="Ferramenta para listar lembretes armazenados, a resposta deverá ser em português."
    ),
    Tool(
        name="Remover lembrete",
        func=reminder.remove_reminder,
        description="Ferramenta para remover lembretes armazenados utilizando uma descrição e uma data/hora, a estrutura deverá ser um json com as chaves 'description' e 'date_time'"
    ),
    Tool(
        name="Obter previsão do tempo",
        func=tool_weather.get_weather,
        description=(
        "Retorna a previsão do tempo para uma determinada localização "
        "O Action Input deve ser uma string com o nome da cidade. Exemplo: Curitiba "
    )
    ),
]

prefix = """\
    Você é um assistente para idosos desenvolvido pelo Laboratório Visão Robótica e Imagem da Universidade Federal do Paraná. \
    Seu nome é Lucas e você deve assumir a identidade de um jovem que está sempre disposto a ajudar. \

    Suas respostas devem ser:
    - Claras, amigáveis e objetivas
    - Em linguagem simples e educada
    - SEMPRE em português (sem uso de emojis)
    - Sempre considere que suas respostas serão faladas em voz alta, então evite usar formatações complexas ou listas
    
    Caso o nome do usuário seja reconhecido, utilize-o para se referir a ele.
    Você receberá um input de um usuário (provavelmente um idoso), junto com seu nome (caso tenha sido reconhecido) e o horário de envio da mensagem.

    Você deve usar ferramentas (tools) SOMENTE quando a pergunta do usuário **realmente exigir** uma informação externa (como previsão do tempo ou lembretes).
    Caso você consiga responder diretamente com base no seu conhecimento, apenas responda diretamente — **sem usar nenhuma ferramenta** e sem escrever "Thought", "Action" ou "Action Input".

    IMPORTANTE: Quando quiser usar uma ferramenta, você DEVE obrigatoriamente seguir este formato:

    Thought: explique o motivo de usar uma ferramenta
    Action: [nome_exato_da_ferramenta]
    Após usar uma ferramenta (Action Input), pare. A resposta ao usuário será construída automaticamente com base no resultado (Observation). Não continue escrevendo após isso.

    Por exemplo, para consultar o tempo em Curitiba, use:

    Action: Obter previsão do tempo  
    Action Input: "Curitiba"
    A resposta deve ser algo como: "A previsão do tempo em Curitiba é de sol com algumas nuvens e temperatura de 25 graus Celsius." Sempre traduzindo termos em inglês.
    """

suffix = """Comece agora respondendo o usuário.
User Input: {input}
{agent_scratchpad}"""

prompt = ZeroShotAgent.create_prompt(
    tools,
    prefix=prefix,
    suffix=suffix,
    input_variables=["input", "agent_scratchpad"],
)

llm_chain = LLMChain(llm = llm, prompt=prompt)
agent = ZeroShotAgent(llm_chain=llm_chain, tools=tools, verbose=True)
agent_chain = AgentExecutor.from_agent_and_tools(agent=agent, tools=tools, verbose=True, handle_parsing_errors=True)


@app.post("/processing")
async def process_audio(request: AudioRequest):
    user_input = request.user_input
    speaker_name = request.speaker_name

    ''' Prompt'''
    message_date = datetime.datetime.now().strftime("%d/%m/%Y %H:%M:%S")
    combined_input = f"{user_input}\n[Data da mensagem: {message_date} | Nome do usuário: {speaker_name}]"

    response = agent_chain.run(
        input=combined_input
    )

    return {"processed_text": response}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="127.0.0.1", port=8080)