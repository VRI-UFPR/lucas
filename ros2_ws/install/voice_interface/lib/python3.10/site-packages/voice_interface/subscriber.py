import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import pygame
import requests
import time
from PIL import Image
import voice_interface.utils as utils

GIF_IDLE = "../gifs/idle.gif"
GIF_SPEAKING = "../gifs/speaking.gif"
GIF_LISTENING = "../gifs/listening.gif"
IMG_IDLE_TO_LISTEN = "../gifs/Idle_to_Listen.png"
URL = "http://127.0.0.1:8080/processing"

class InteractionNode(Node):
    def __init__(self):
        super().__init__('interaction_node')
        self.subscription = self.create_subscription(
            String,
            'wake_word_detected',
            self.on_wake_word_detected,
            10
        )

        self.state = "idle" 
        self.speaking_text = ""

        self.speaking_frames = utils.load_gif_frames(GIF_SPEAKING)
        self.idle_frames = utils.load_gif_frames(GIF_IDLE)
        self.listening_frames = utils.load_gif_frames(GIF_LISTENING)
        raw_image = pygame.image.load(IMG_IDLE_TO_LISTEN)
        self.listen_image = pygame.transform.scale(raw_image, (400, 400))


        self.processing = False
        self.running = True

        self.ui_thread = threading.Thread(target=self.show_ui_loop)
        self.ui_thread.start()

    def on_wake_word_detected(self, msg):
        if not self.processing:
            self.processing = True
            self.state = "idle_to_listen"
            threading.Thread(target=self.transition_to_listening_and_handle).start()

    def transition_to_listening_and_handle(self):
        time.sleep(0.8)  
        self.state = "listening"
        self.handle_interaction()

    def handle_interaction(self):
        try:
            user_input, speaker_name = utils.get_input()
            payload = {"user_input": user_input, "speaker_name": speaker_name}
            response = requests.post(URL, json=payload)
            response_text = response.json()["processed_text"]

            self.speaking_text = response_text
            self.state = "speaking"
            utils.play_text(response_text)
            time.sleep(1)
        except Exception as e:
            print("Erro:", e)
        finally:
            self.state = "idle"
            self.processing = False

    def show_ui_loop(self):
        pygame.init()
        screen = pygame.display.set_mode((1280, 720))
        pygame.display.set_caption("Lucas")
        clock = pygame.time.Clock()
        font = pygame.font.SysFont('arial', 48, bold=True)

        idle_index = 0
        speak_index = 0
        listen_index = 0
        text_index = 0
        text_speed = 2
        last_frame_time = time.time()

        while self.running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False

            screen.fill((0, 0, 0))

            if self.state == "idle":
                now = time.time()
                if now - last_frame_time > 0.2:
                    idle_index = (idle_index + 1) % len(self.idle_frames)
                    last_frame_time = now
                frame = self.idle_frames[idle_index]
                screen.blit(frame, frame.get_rect(center=(640, 360)))

            elif self.state == "idle_to_listen":
                screen.blit(self.listen_image, self.listen_image.get_rect(center=(640, 360)))

            elif self.state == "listening":
                now = time.time()
                if now - last_frame_time > 0.2:
                    listen_index = (listen_index + 1) % len(self.listening_frames)
                    last_frame_time = now
                frame = self.listening_frames[listen_index]
                screen.blit(frame, frame.get_rect(center=(640, 360)))

            elif self.state == "speaking":
                now = time.time()
                if now - last_frame_time > 0.1:
                    speak_index = (speak_index + 1) % len(self.speaking_frames)
                    last_frame_time = now
                frame = self.speaking_frames[speak_index]
                screen.blit(frame, frame.get_rect(center=(960, 360)))

                if text_index < len(self.speaking_text):
                    text_index += text_speed

                visible_text = self.speaking_text[:text_index]
                text_surface = utils.render_text_with_outline(
                    visible_text, font, (255, 255, 255), (0, 0, 0), 600
                )
                screen.blit(text_surface, (50, 100))

            pygame.display.flip()
            clock.tick(60)

        pygame.quit()


def main(args=None):
    rclpy.init(args=args)
    node = InteractionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.ui_thread.join()
        node.destroy_node()
        rclpy.shutdown()