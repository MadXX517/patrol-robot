#!/usr/bin/env python3
# encoding: utf-8
import io
import json
import os
import threading
import time
import urllib.error
import urllib.request
import wave

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .command_parser import (
    VOICE_COMMAND_PROMPT,
    command_to_json,
    extract_json_objects,
    parse_fixed_command,
    validate_command,
)


def first_env(*names):
    for name in names:
        value = os.environ.get(name)
        if value:
            return value
    return ''


class VoiceFrontend(Node):
    def __init__(self):
        super().__init__('voice_frontend')

        self.declare_parameter('serial_port', '/dev/aibox')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('enable_serial', True)
        self.declare_parameter('enable_asr', True)
        self.declare_parameter('enable_llm', True)
        self.declare_parameter('enable_tts', True)
        self.declare_parameter('api_key', '')
        self.declare_parameter('base_url', 'https://dashscope.aliyuncs.com/compatible-mode/v1')
        self.declare_parameter('asr_model', 'paraformer-realtime-v2')
        self.declare_parameter('llm_model', 'qwen-plus')
        self.declare_parameter('tts_model', 'qwen-tts')
        self.declare_parameter('tts_voice', 'Serena')
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('asr_timeout_sec', 8.0)
        self.declare_parameter('llm_timeout_sec', 20.0)
        self.declare_parameter('command_topic', '/car_voice/command')
        self.declare_parameter('text_topic', '/car_voice/text')
        self.declare_parameter('status_topic', '/car_voice/status')
        self.declare_parameter('say_topic', '/car_voice/say')
        self.declare_parameter('mock_text_topic', '/car_voice/mock_text')

        self.serial_port = self.get_parameter('serial_port').value
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.enable_serial = bool(self.get_parameter('enable_serial').value)
        self.enable_asr = bool(self.get_parameter('enable_asr').value)
        self.enable_llm = bool(self.get_parameter('enable_llm').value)
        self.enable_tts = bool(self.get_parameter('enable_tts').value)
        self.api_key = self.get_parameter('api_key').value or first_env(
            'DASHSCOPE_API_KEY',
            'DASHCOPE_API_KEY',
            'OPENAI_API_KEY',
        )
        self.base_url = self.get_parameter('base_url').value
        self.asr_model = self.get_parameter('asr_model').value
        self.llm_model = self.get_parameter('llm_model').value
        self.tts_model = self.get_parameter('tts_model').value
        self.tts_voice = self.get_parameter('tts_voice').value
        self.sample_rate = int(self.get_parameter('sample_rate').value)
        self.asr_timeout_sec = max(1.0, float(self.get_parameter('asr_timeout_sec').value))
        self.llm_timeout_sec = max(1.0, float(self.get_parameter('llm_timeout_sec').value))

        self.command_pub = self.create_publisher(
            String,
            self.get_parameter('command_topic').value,
            10,
        )
        self.text_pub = self.create_publisher(
            String,
            self.get_parameter('text_topic').value,
            10,
        )
        self.status_pub = self.create_publisher(
            String,
            self.get_parameter('status_topic').value,
            10,
        )
        self.create_subscription(
            String,
            self.get_parameter('mock_text_topic').value,
            self.mock_text_callback,
            10,
        )
        self.create_subscription(
            String,
            self.get_parameter('say_topic').value,
            self.say_callback,
            10,
        )

        self.shutdown_requested = threading.Event()
        self.asr_busy = threading.Event()
        self.serial_thread = None
        if self.enable_serial:
            self.serial_thread = threading.Thread(target=self.serial_loop, daemon=True)
            self.serial_thread.start()

        self.publish_status('ready', 'voice_frontend started')

    def mock_text_callback(self, msg):
        self.handle_text(msg.data)

    def say_callback(self, msg):
        text = msg.data.strip()
        if not text:
            return
        if not self.enable_tts:
            self.get_logger().info(f'TTS disabled, say: {text}')
            return
        threading.Thread(target=self.play_tts, args=(text,), daemon=True).start()

    def serial_loop(self):
        try:
            import serial
        except ImportError:
            self.publish_status('error', 'pyserial is not installed')
            return

        while not self.shutdown_requested.is_set() and rclpy.ok():
            try:
                with serial.Serial(
                    port=self.serial_port,
                    baudrate=self.baudrate,
                    bytesize=serial.EIGHTBITS,
                    stopbits=serial.STOPBITS_ONE,
                    parity=serial.PARITY_NONE,
                    timeout=1,
                ) as ser:
                    self.publish_status('ready', f'serial opened: {self.serial_port}')
                    ser.write(b'llm_mode')
                    while not self.shutdown_requested.is_set() and rclpy.ok():
                        data = ser.read(64)
                        if not data:
                            continue
                        text = data.decode('utf-8', errors='ignore')
                        if 'llm_mic_start' in text:
                            self.start_asr_once(ser)
            except Exception as exc:
                self.publish_status('error', f'serial error: {exc}')
                time.sleep(3.0)

    def start_asr_once(self, ser=None):
        if self.asr_busy.is_set():
            self.publish_status('busy', 'ASR is already running')
            return
        self.asr_busy.set()
        threading.Thread(target=self.asr_worker, args=(ser,), daemon=True).start()

    def asr_worker(self, ser=None):
        try:
            if not self.enable_asr:
                self.publish_status('rejected', 'ASR disabled')
                return
            text = self.recognize_once()
            if text:
                self.handle_text(text)
        except Exception as exc:
            self.publish_status('error', f'ASR failed: {exc}')
        finally:
            if ser is not None:
                try:
                    ser.write(b'llm_mic_end')
                except Exception as exc:
                    self.publish_status('error', f'failed to send llm_mic_end: {exc}')
            self.asr_busy.clear()

    def recognize_once(self):
        if not self.api_key:
            raise RuntimeError('DASHSCOPE_API_KEY is not configured')
        try:
            import pyaudio
            from dashscope.audio.asr import Recognition, RecognitionCallback
        except ImportError as exc:
            raise RuntimeError('pyaudio and dashscope are required for ASR') from exc

        result = {'text': ''}
        done = threading.Event()

        class Callback(RecognitionCallback):
            def on_event(self, recognition_result):
                sentence = extract_sentence_text(recognition_result)
                if sentence:
                    result['text'] = sentence

            def on_complete(self):
                done.set()

            def on_error(self, message):
                done.set()

        callback = Callback()
        recognizer = Recognition(
            model=self.asr_model,
            format='pcm',
            sample_rate=self.sample_rate,
            callback=callback,
            api_key=self.api_key,
        )
        audio = pyaudio.PyAudio()
        stream = audio.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=self.sample_rate,
            input=True,
            frames_per_buffer=3200,
        )

        try:
            recognizer.start()
            deadline = time.time() + self.asr_timeout_sec
            while time.time() < deadline and not done.is_set():
                recognizer.send_audio_frame(stream.read(3200, exception_on_overflow=False))
            recognizer.stop()
        finally:
            stream.stop_stream()
            stream.close()
            audio.terminate()

        return result['text'].strip()

    def handle_text(self, text):
        text = str(text or '').strip()
        if not text:
            return
        self.publish_text(text)

        fixed = parse_fixed_command(text)
        if fixed is not None:
            self.publish_command(fixed, source='fixed')
            return

        if not self.enable_llm:
            self.publish_status('rejected', 'LLM disabled', text=text)
            return
        if not self.api_key:
            self.publish_status('error', 'DASHSCOPE_API_KEY is not configured')
            return

        try:
            command = self.parse_with_llm(text)
        except Exception as exc:
            self.publish_status('error', f'LLM parse failed: {exc}', text=text)
            return
        self.publish_command(command, source='llm')

    def parse_with_llm(self, text):
        payload = {
            'model': self.llm_model,
            'messages': [
                {'role': 'system', 'content': VOICE_COMMAND_PROMPT},
                {'role': 'user', 'content': text},
            ],
            'temperature': 0.0,
        }
        data = json.dumps(payload, ensure_ascii=False).encode('utf-8')
        request = urllib.request.Request(
            self.base_url.rstrip('/') + '/chat/completions',
            data=data,
            headers={
                'Authorization': f'Bearer {self.api_key}',
                'Content-Type': 'application/json',
            },
            method='POST',
        )
        try:
            with urllib.request.urlopen(request, timeout=self.llm_timeout_sec) as response:
                raw = response.read().decode('utf-8')
        except urllib.error.HTTPError as exc:
            body = exc.read().decode('utf-8', errors='replace')
            raise RuntimeError(f'HTTP {exc.code}: {body}') from exc
        except urllib.error.URLError as exc:
            raise RuntimeError(str(exc)) from exc

        result = json.loads(raw)
        content = result['choices'][0]['message']['content']
        if isinstance(content, list):
            content = ''.join(str(item.get('text', item)) for item in content)
        for candidate in extract_json_objects(str(content)):
            try:
                return validate_command(candidate)
            except ValueError:
                continue
        raise RuntimeError(f'no supported command in LLM response: {content}')

    def play_tts(self, text):
        if not self.api_key:
            self.publish_status('error', 'DASHSCOPE_API_KEY is not configured for TTS')
            return
        try:
            from openai import OpenAI
        except ImportError:
            self.publish_status('error', 'openai package is required for TTS playback')
            return

        try:
            client = OpenAI(api_key=self.api_key, base_url=self.base_url)
            response = client.audio.speech.create(
                model=self.tts_model,
                voice=self.tts_voice,
                input=text,
                response_format='wav',
            )
            data = response.read() if hasattr(response, 'read') else bytes(response)
            play_wav_bytes(data)
        except Exception as exc:
            self.publish_status('error', f'TTS failed: {exc}', text=text)

    def publish_text(self, text):
        msg = String()
        msg.data = text
        self.text_pub.publish(msg)
        self.publish_status('heard', text)

    def publish_command(self, command, source):
        msg = String()
        msg.data = command_to_json(command)
        self.command_pub.publish(msg)
        self.publish_status('command', f'{source} command published', command=command)

    def publish_status(self, state, message, **extra):
        payload = {
            'state': state,
            'message': message,
            'time': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
        }
        payload.update(extra)
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(msg)
        if state == 'error':
            self.get_logger().error(message)
        else:
            self.get_logger().info(message)

    def destroy_node(self):
        self.shutdown_requested.set()
        super().destroy_node()


def extract_sentence_text(recognition_result):
    if recognition_result is None:
        return ''
    if hasattr(recognition_result, 'get_sentence'):
        sentence = recognition_result.get_sentence()
        if isinstance(sentence, dict):
            return str(sentence.get('text') or '').strip()
        return str(sentence or '').strip()
    if isinstance(recognition_result, dict):
        output = recognition_result.get('output') or {}
        sentence = output.get('sentence') or {}
        return str(sentence.get('text') or output.get('text') or '').strip()
    output = getattr(recognition_result, 'output', None)
    if isinstance(output, dict):
        sentence = output.get('sentence') or {}
        return str(sentence.get('text') or output.get('text') or '').strip()
    return ''


def play_wav_bytes(data):
    try:
        import pyaudio
    except ImportError as exc:
        raise RuntimeError('pyaudio is required for local playback') from exc

    with wave.open(io.BytesIO(data), 'rb') as wav:
        audio = pyaudio.PyAudio()
        stream = audio.open(
            format=audio.get_format_from_width(wav.getsampwidth()),
            channels=wav.getnchannels(),
            rate=wav.getframerate(),
            output=True,
        )
        try:
            while True:
                chunk = wav.readframes(1024)
                if not chunk:
                    break
                stream.write(chunk)
        finally:
            stream.stop_stream()
            stream.close()
            audio.terminate()


def main(args=None):
    rclpy.init(args=args)
    node = VoiceFrontend()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
