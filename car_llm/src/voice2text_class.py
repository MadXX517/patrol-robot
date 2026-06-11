# voice2text_class.py
# 明文重写版(替换原 pyarmor 加密模块),适配 Python 3.10。
# 实时语音转文字:DashScope paraformer-realtime-v2 + pyaudio 麦克风。
#
# 契约(由 llm_main.py 反推):
#   RealTimeASR(api_key, result_queue, model, max_sentence_silence)
#     .start()                       # 初始化,启动后台读麦线程(默认未聆听)
#     .audio_loop(llm_mic_running)   # 主循环反复调用;True=聆听,False=暂停(非阻塞)
#     .stop()                        # 停止会话与线程,释放麦克风
#   识别到的整句文本 push 进 result_queue
#
# 设计:后台线程负责会话生命周期与读麦送帧;audio_loop 只切换 enable 标志,
#       保证主线程不被阻塞。TTS 播报期间主程序会把门控置 False,避免录到自身声音。

import threading
import time

import pyaudio
import dashscope
from dashscope.audio.asr import (Recognition,
                                 RecognitionCallback,
                                 RecognitionResult)

# 麦克风录音参数(paraformer 推荐)
_SAMPLE_RATE = 16000
_CHANNELS = 1
_BLOCK_SIZE = 3200  # 约 200ms / 帧


class _ASRCallback(RecognitionCallback):
    def __init__(self, owner):
        self._owner = owner

    def on_open(self) -> None:
        print("[ASR] 识别会话已打开")

    def on_close(self) -> None:
        print("[ASR] 识别会话已关闭")

    def on_error(self, result) -> None:
        msg = getattr(result, "message", result)
        print(f"[ASR] 识别错误: {msg}")
        self._owner._mark_error()

    def on_event(self, result: RecognitionResult) -> None:
        try:
            sentence = result.get_sentence()
        except Exception:
            return
        if not sentence:
            return
        # sentence 可能是 dict 或 list
        if isinstance(sentence, list):
            sentence = sentence[-1] if sentence else None
        if not isinstance(sentence, dict):
            return
        text = sentence.get("text", "")
        try:
            ended = RecognitionResult.is_sentence_end(sentence)
        except Exception:
            ended = bool(sentence.get("sentence_end", False))
        if ended and text:
            self._owner._on_final_sentence(text)


class RealTimeASR:
    def __init__(self, api_key=None, result_queue=None, model='paraformer-realtime-v2',
                 max_sentence_silence=800, input_device_index=None,
                 vocabulary_id=None, language_hints=None):
        if not api_key:
            raise ValueError("RealTimeASR 需要 DashScope API Key")
        if result_queue is None:
            raise ValueError("RealTimeASR 需要 result_queue")
        dashscope.api_key = api_key
        self.api_key = api_key
        self.result_queue = result_queue
        self.model = model
        self.max_sentence_silence = int(max_sentence_silence)
        self.input_device_index = input_device_index
        self.language_hints = language_hints or ['zh', 'en']

        # 可选热词:失败则静默降级为无热词
        if vocabulary_id is None:
            vocabulary_id = self._try_load_vocabulary()
        self.vocabulary_id = vocabulary_id

        self._pa = None
        self._mic = None
        self._recognition = None
        self._callback = _ASRCallback(self)

        self._enabled = threading.Event()   # 是否聆听
        self._stop = threading.Event()      # 是否退出
        self._error = threading.Event()     # 会话出错需重建
        self._session_active = False
        self._worker = None

    # ---------- 可选热词 ----------
    def _try_load_vocabulary(self):
        try:
            import vocabulary_manager
            return vocabulary_manager.get_vocabulary_id(self.api_key)
        except Exception as e:
            print(f"[ASR] 未启用热词(降级): {e}")
            return None

    # ---------- 内部回调钩子 ----------
    def _mark_error(self):
        self._error.set()

    def _on_final_sentence(self, text):
        if self._enabled.is_set():
            print(f"[ASR] 整句: {text}")
            self.result_queue.put(text)

    # ---------- 会话管理 ----------
    def _build_recognition(self):
        kwargs = dict(model=self.model, format='pcm', sample_rate=_SAMPLE_RATE,
                      callback=self._callback)
        # 逐项加入可选参数,SDK 版本不支持时回退
        optional = dict(language_hints=self.language_hints,
                        max_sentence_silence=self.max_sentence_silence)
        if self.vocabulary_id:
            optional['vocabulary_id'] = self.vocabulary_id
        for k, v in optional.items():
            kwargs[k] = v
        try:
            return Recognition(**kwargs)
        except TypeError:
            # 去掉可选参数重试
            base = dict(model=self.model, format='pcm', sample_rate=_SAMPLE_RATE,
                        callback=self._callback)
            return Recognition(**base)

    def _open_session(self):
        self._error.clear()
        self._recognition = self._build_recognition()
        self._recognition.start()
        self._mic = self._pa.open(format=pyaudio.paInt16, channels=_CHANNELS,
                                  rate=_SAMPLE_RATE, input=True,
                                  frames_per_buffer=_BLOCK_SIZE,
                                  input_device_index=self.input_device_index)
        self._session_active = True
        print("[ASR] 开始聆听")

    def _close_session(self):
        if self._mic is not None:
            try:
                self._mic.stop_stream()
                self._mic.close()
            except Exception:
                pass
            self._mic = None
        if self._recognition is not None:
            try:
                self._recognition.stop()
            except Exception:
                pass
            self._recognition = None
        self._session_active = False
        print("[ASR] 暂停聆听")

    def _run(self):
        while not self._stop.is_set():
            try:
                if self._enabled.is_set():
                    if not self._session_active or self._error.is_set():
                        if self._session_active:
                            self._close_session()
                        self._open_session()
                    # 读麦送帧
                    data = self._mic.read(_BLOCK_SIZE, exception_on_overflow=False)
                    self._recognition.send_audio_frame(data)
                else:
                    if self._session_active:
                        self._close_session()
                    time.sleep(0.02)
            except Exception as e:
                print(f"[ASR] 线程异常,重建会话: {e}")
                self._close_session()
                time.sleep(0.3)

    # ---------- 对外接口 ----------
    def start(self):
        if self._pa is None:
            self._pa = pyaudio.PyAudio()
        self._stop.clear()
        if self._worker is None or not self._worker.is_alive():
            self._worker = threading.Thread(target=self._run, daemon=True)
            self._worker.start()
        print("[ASR] 已启动(等待聆听门控)")

    def audio_loop(self, llm_mic_running):
        """主线程反复调用;仅切换聆听标志,非阻塞。"""
        if llm_mic_running:
            self._enabled.set()
        else:
            self._enabled.clear()

    def stop(self):
        self._stop.set()
        self._enabled.clear()
        if self._worker is not None:
            self._worker.join(timeout=3)
            self._worker = None
        self._close_session()
        if self._pa is not None:
            try:
                self._pa.terminate()
            except Exception:
                pass
            self._pa = None
        print("[ASR] 已停止")


if __name__ == "__main__":
    import argparse
    from multiprocessing import Queue
    ap = argparse.ArgumentParser()
    ap.add_argument("--api_key", required=True)
    ap.add_argument("--model", default="paraformer-realtime-v2")
    args = ap.parse_args()

    q = Queue()
    asr = RealTimeASR(api_key=args.api_key, result_queue=q, model=args.model)
    asr.start()
    print("开始聆听 10 秒,请说话...")
    t0 = time.time()
    try:
        while time.time() - t0 < 10:
            asr.audio_loop(True)
            while not q.empty():
                print("识别结果:", q.get())
            time.sleep(0.05)
    finally:
        asr.stop()
