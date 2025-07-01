import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pvporcupine
import sounddevice as sd
import struct
from dotenv import load_dotenv
import os

load_dotenv()
class WakeWordNode(Node):
    
    def __init__(self):
        
        super().__init__('wake_word_node')
        self.publisher_ = self.create_publisher(String, 'wake_word_detected', 10)

        base_dir = os.path.dirname(__file__)
        keyword_path = os.path.join(base_dir, "lucas.ppn")
        model_path = os.path.join(base_dir, "porcupine_params_pt.pv")
        self.porcupine = pvporcupine.create(
            access_key=os.getenv("PORCUPINE_KEY"),
            keyword_paths=[keyword_path],
            model_path=model_path,
        )

        self.get_logger().info('Wake word node iniciado! Aguardando Lucas...')

        self.stream = sd.RawInputStream(
            samplerate=self.porcupine.sample_rate,
            blocksize=self.porcupine.frame_length,
            channels=1,
            dtype='int16',
            callback=self.audio_callback
        )
        self.stream.start()

    def audio_callback(self, in_data, frames, time, status):
        pcm = struct.unpack_from("h" * self.porcupine.frame_length, in_data)
        result = self.porcupine.process(pcm)
        if result >= 0:
            msg = String()
            msg.data = 'wake word detectada'
            self.publisher_.publish(msg)
            self.get_logger().info('Wake word detectada!')

    def destroy_node(self):
        super().destroy_node()
        self.stream.stop()
        self.stream.close()
        self.porcupine.delete()

def main(args=None):
    rclpy.init(args=args)
    node = WakeWordNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()