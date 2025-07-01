from PIL import Image
import pygame
from pydub import AudioSegment
from pydub.playback import play
import edge_tts
import pyaudio
import wave
import os
import subprocess
import time
from speechbrain.pretrained import SpeakerRecognition

def load_gif_frames(gif_path):
    gif = Image.open(gif_path)
    frames = []
    try:
        while True:
            frame = gif.copy().convert("RGBA")
            frame = frame.resize((400, 400))
            pygame_image = pygame.image.fromstring(frame.tobytes(), frame.size, frame.mode)
            frames.append(pygame_image)
            gif.seek(gif.tell() + 1)
    except EOFError:
        pass
    return frames

def play_text(text):
    VOICE = "pt-BR-AntonioNeural"
    OUTPUT = "output.mp3"
    comunicate = edge_tts.Communicate(text, VOICE)
    with open(OUTPUT, "wb") as f:
        for chunk in comunicate.stream_sync():
            if chunk["type"] == "audio":
                f.write(chunk["data"])
    sound = AudioSegment.from_mp3(OUTPUT)
    play(sound)

def render_text_with_outline(text, font, color, outline_color, max_width):
    words = text.split(' ')
    lines = []
    current_line = ""
    for word in words:
        test_line = current_line + word + " "
        if font.size(test_line)[0] < max_width:
            current_line = test_line
        else:
            lines.append(current_line)
            current_line = word + " "
    lines.append(current_line)
    surfaces = []
    for line in lines:
        surfaces.append(render_text_line_with_outline(line.strip(), font, color, outline_color))
    total_height = sum(s.get_height() for s in surfaces)
    surface = pygame.Surface((max_width, total_height), pygame.SRCALPHA)
    y = 0
    for s in surfaces:
        surface.blit(s, (0, y))
        y += s.get_height()
    return surface

def render_text_line_with_outline(text, font, color, outline_color):
    base = font.render(text, True, color)
    width, height = base.get_size()
    surface = pygame.Surface((width + 4, height + 4), pygame.SRCALPHA)
    for dx in [-2, 0, 2]:
        for dy in [-2, 0, 2]:
            if dx != 0 or dy != 0:
                outline = font.render(text, True, outline_color)
                surface.blit(outline, (dx + 2, dy + 2))
    surface.blit(base, (2, 2))
    return surface

''' ----- Speech to text ----- '''
def get_input():
    """
    Records audio from the microphone, saves it to a WAV file, optionally identifies the speaker,
    and transcribes the audio using the Whisper model.
 
    Returns:
    tuple: A tuple containing the transcribed text (str) and the speaker name.
    """

    FORMAT = pyaudio.paInt16
    CHANNELS = 2
    RATE = 44100
    CHUNK = 1024
    WAVE_OUTPUT_FILENAME = "output.wav"
    RECORD_SECONDS = 5
    speaker_name = None
    should_save = False

    audio = pyaudio.PyAudio()

    ''' Recording '''
    stream = audio.open(format=FORMAT, channels=CHANNELS,
                        rate=RATE, input=True,
                        frames_per_buffer=CHUNK)
    print("Gravando...")
    frames = []

    for _ in range (0, int(RATE / CHUNK * RECORD_SECONDS)):
        data = stream.read(CHUNK)
        frames.append(data)

    print("Gravação finalizada")

    stream.stop_stream()
    stream.close()
    audio.terminate()

    ''' Saving '''
    waveFile = wave.open(WAVE_OUTPUT_FILENAME, 'wb')
    waveFile.setnchannels(CHANNELS)
    waveFile.setsampwidth(audio.get_sample_size(FORMAT))
    waveFile.setframerate(RATE)
    waveFile.writeframes(b''.join(frames))
    waveFile.close()

    model_name = "tiny"
    subprocess.run([
        "whisper",
        "--language", "pt",
        "--model", model_name,
        "--output_dir", f"output-{model_name}",
        WAVE_OUTPUT_FILENAME
    ])

    transcribed_file = f"output-{model_name}/{WAVE_OUTPUT_FILENAME}.txt"
    with open(transcribed_file, "r") as f:
        text = f.read()
    
    speaker_name = identify_speaker(WAVE_OUTPUT_FILENAME)

    return text, speaker_name
    
''' Speaker recognition '''
def identify_speaker(voice_current_speaker): 
    """
    Identifies the speaker by comparing the given voice sample with all saved speakers in 'voices/'.
    Each speaker is represented by a folder with multiple .wav recordings.

    Returns:
        str: Name of the best matching speaker folder, or "unknown" if confidence is too low.
    """

    verification = SpeakerRecognition.from_hparams(
        source="speechbrain/spkrec-ecapa-voxceleb",
        savedir="pretrained_models/spkrec-ecapa-voxceleb"
    )

    best_score = 0
    best_match = "unknown"

    base_dir = os.path.dirname(os.path.abspath(__file__))
    voices_dir = os.path.join(base_dir, "voices")

    for speaker_folder in os.listdir(voices_dir):
        speaker_path = os.path.join(voices_dir, speaker_folder)
        if not os.path.isdir(speaker_path):
            continue

        scores = []
        for file in os.listdir(speaker_path):
            if file.endswith(".wav"):
                file_path = os.path.join(speaker_path, file)
                score, _ = verification.verify_files(voice_current_speaker, file_path)
                scores.append(score.item())
                print(f"Comparando com {speaker_folder}/{file} → score: {score.item()}")

        if scores:
            avg_score = sum(scores) / len(scores)
            print(f"Média de {speaker_folder}: {avg_score:.3f}")

            if avg_score > best_score:
                best_score = avg_score
                best_match = speaker_folder

    if best_score < 0.1:
        print("Nenhuma correspondência com confiança suficiente.")
        return "unknown"

    print(f"Identificado como {best_match} com média {best_score:.3f}")
    return best_match
