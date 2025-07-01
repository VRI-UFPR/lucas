# Better Call Lucas: A Conversational Assistant for Older Adult Support

*An assistant entirely based on open-source tools for supporting older adults.*

<img src="lucas.gif" alt="Lucas" style="zoom:50%;" />

# Lucas Installation and Configuration
This repository contains the necessary code for the installation and configuration of Lucas, a virtual assistant developed by the Vision, Robotics, and Image laboratory at UFPR, which interacts with the user through voice commands.

First, it is necessary to have ROS 2 Humble installed. To do this, follow the instructions on the official [ROS 2](https://docs.ros.org/en/humble/Installation.html) website.

## Dependencies
For Lucas to work correctly, it is necessary to install some dependencies. Run the following commands in the terminal:
```bash
sudo apt update
pip install -r requirements.txt
```

### Porcupine
Porcupine is used for keyword detection in audio. Thus, it is responsible for detecting when the user says "Opa Lucas".

To get it ready for use, you need to create an account on the [Picovoice](https://picovoice.ai/), website, copy the generated access token, and use the following command:

```export PORCUPINE_KEY=<your_access_token>```

## Initializing Lucas
Three terminals are required to initialize Lucas: one for the voice command, another for the voice interface, and another for running the LLM server.

Only two terminals will use ROS2, the one responsible for voice commands and the interface. Follow these steps in both terminals:Three terminals are required to initialize Lucas: one for the voice command, another for the voice interface, and another for running the LLM server.

Only two terminals will use ROS2, the one responsible for voice commands and the interface. Follow these steps in both terminals:
1. ```source /opt/ros/humble/setup.bash```
2. ```cd ros2_ws```
3. ```colcon build```
4. ```source install/setup.bash```
5. ```ros2 run voice_interface <node>```

The "node" variable should be replaced with the name of the node you want to run. The available nodes are: interaction_node and wake_word_node.

Finally, in a new terminal, run the LLM server algorithm with the following command:

```bash
python3 server.py
```

## 📁 Assets

* All GIFs of Lucas can be found in: `gifs/`
* The prompt used for the assistant is located in: `prompt/`
*  The dataset used to produce the results is in: `dataset/`

## **🔍** Architecture

<img src="architecture.png" alt="architecture" style="zoom:55%;" />

## 📝 Citation

```
@article{LucasTheAssistant,
  author    = {Luan Matheus Trindade Dalmazo, Luize Duarte, Vitor Last Pintarelli, Eduardo Todt},
  title     = {Better Call Lucas: A Conversational Assistant for Older Adult Support},
  journal   = {XXX},
  year      = {2025},
  doi       = {10.XXXX/XXXXX}
}
```