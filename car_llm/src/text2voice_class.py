# text2voice_class.py
# 明文重写版(替换原 pyarmor 加密模块),适配 Python 3.10。
# 文字转语音:DashScope qwen-tts / qwen3-tts,经 pyaudio 播放。
#
# 契约(由 llm_main.py 反推):
#   TTSPlayer(api_key, voice, volume_percent, rate, model)
#     .say(text)   # 阻塞播放一段文本
#     .close()     # 释放音频资源
#
# 参考:dashscope.MultiModalConversation.call(model=..., text=..., voice=..., stream=True)
#       流式 chunk.output.audio.data 为 base64 PCM(int16, 24kHz, 单声道)

import base64
import wave
import tempfile
import os

import numpy as np
import pyaudio
import dashscope

# 北京区域 API 地址(如用新加坡区域改为 dashscope-intl)
dashscope.base_http_api_url = 'https://dashscope.aliyuncs.com/api/v1'


class TTSPlayer:
    def __init__(self, api_key=None, voice='Serena', volume_percent=85,
                 rate=24000, model='qwen-tts', language_type='Chinese',
                 output_device_index=None):
        if not api_key:
            raise ValueError("TTSPlayer 需要 DashScope API Key")
        self.api_key = api_key
        dashscope.api_key = api_key

        self.voice = voice
        self.model = model
        self.rate = int(rate)
        self.language_type = language_type
        self.output_device_index = output_device_index
        self.volume = max(0.0, min(1.0, volume_percent / 100.0))

        self._pa = pyaudio.PyAudio()
        self._stream = self._pa.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=self.rate,
            output=True,
            output_device_index=self.output_device_index,
        )

    # ---------- 内部工具 ----------
    def _apply_volume(self, audio_np):
        if self.volume == 1.0:
            return audio_np
        scaled = audio_np.astype(np.float32) * self.volume
        scaled = np.clip(scaled, -32768, 32767)
        return scaled.astype(np.int16)

    def _write_pcm(self, pcm_bytes):
        if not pcm_bytes:
            return
        audio_np = np.frombuffer(pcm_bytes, dtype=np.int16)
        audio_np = self._apply_volume(audio_np)
        self._stream.write(audio_np.tobytes())

    def _call(self, text, stream):
        """调用 DashScope;language_type 对老模型可能不支持,失败则去掉重试。"""
        kwargs = dict(model=self.model, api_key=self.api_key,
                      text=text, voice=self.voice, stream=stream)
        try:
            return dashscope.MultiModalConversation.call(
                language_type=self.language_type, **kwargs)
        except TypeError:
            return dashscope.MultiModalConversation.call(**kwargs)

    # ---------- 对外接口 ----------
    def say(self, text):
        if text is None:
            return
        text = str(text).strip()
        if not text:
            return
        print(f"[TTS] 播报: {text}")
        try:
            if self._say_stream(text):
                return
        except Exception as e:
            print(f"[TTS] 流式播放失败,转非流式: {e}")
        try:
            self._say_nonstream(text)
        except Exception as e:
            print(f"[TTS] 播放失败: {e}")

    def _say_stream(self, text):
        """流式播放;成功返回 True。"""
        responses = self._call(text, stream=True)
        got_audio = False
        for chunk in responses:
            output = getattr(chunk, "output", None)
            if output is None:
                continue
            audio = getattr(output, "audio", None)
            if audio is not None and getattr(audio, "data", None):
                pcm = base64.b64decode(audio.data)
                self._write_pcm(pcm)
                got_audio = True
            if getattr(output, "finish_reason", None) == "stop":
                break
        return got_audio

    def _say_nonstream(self, text):
        """非流式:取 url 下载 wav 后播放。"""
        import requests  # 延迟导入,避免无网环境 import 失败影响其它功能
        response = self._call(text, stream=False)
        url = response.output.audio.url
        wav_bytes = requests.get(url, timeout=30).content

        tmp_path = None
        try:
            with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as tmp:
                tmp.write(wav_bytes)
                tmp_path = tmp.name
            with wave.open(tmp_path, 'rb') as wf:
                data = wf.readframes(wf.getnframes())
            self._write_pcm(data)
        finally:
            if tmp_path and os.path.exists(tmp_path):
                os.remove(tmp_path)

    def close(self):
        try:
            if self._stream is not None:
                self._stream.stop_stream()
                self._stream.close()
        except Exception:
            pass
        try:
            if self._pa is not None:
                self._pa.terminate()
        except Exception:
            pass
        self._stream = None
        self._pa = None


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--api_key", required=True)
    ap.add_argument("--voice", default="Serena")
    ap.add_argument("--model", default="qwen-tts")
    ap.add_argument("--text", default="你好,我是巡逻机器人,语音合成测试。")
    args = ap.parse_args()

    player = TTSPlayer(api_key=args.api_key, voice=args.voice, model=args.model)
    player.say(args.text)
    player.close()
