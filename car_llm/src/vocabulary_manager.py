# vocabulary_manager.py
# 明文重写版(替换原 pyarmor 加密模块),适配 Python 3.10。
# 可选:为 paraformer 实时识别注册热词(提升专有名词识别率)。
#
# 说明:
#   - 明文代码本不直接调用本模块;voice2text_class 会 best-effort 调用 get_vocabulary_id。
#   - 任何异常都静默降级返回 None,ASR 无热词仍可运行。
#   - 在服务端创建的热词表会持久存在,故按"内容哈希"本地缓存 vocabulary_id,避免每次重建。
#
# 热词来源:config/my_vocabulary.json,格式即 DashScope vocabulary 参数:
#   [{"text": "小甲", "weight": 4, "lang": "zh"}, ...]

import os
import json
import hashlib

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_CONFIG_DIR = os.path.join(os.path.dirname(_THIS_DIR), "config")
_VOCAB_JSON = os.path.join(_CONFIG_DIR, "my_vocabulary.json")
_CACHE_JSON = os.path.join(_CONFIG_DIR, ".vocabulary_id.json")

_TARGET_MODEL = "paraformer-realtime-v2"
_PREFIX = "carllm"  # 仅小写字母/数字,<=10 字符


def _load_vocabulary():
    with open(_VOCAB_JSON, "r", encoding="utf-8") as f:
        return json.load(f)


def _content_hash(vocab, target_model):
    blob = json.dumps(vocab, ensure_ascii=False, sort_keys=True) + "|" + target_model
    return hashlib.md5(blob.encode("utf-8")).hexdigest()


def _read_cache():
    if os.path.exists(_CACHE_JSON):
        try:
            with open(_CACHE_JSON, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception:
            return {}
    return {}


def _write_cache(data):
    try:
        with open(_CACHE_JSON, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
    except Exception as e:
        print(f"[Vocab] 缓存写入失败(忽略): {e}")


def get_vocabulary_id(api_key):
    """返回可用的 vocabulary_id;失败或无热词时返回 None。"""
    if not os.path.exists(_VOCAB_JSON):
        return None
    try:
        vocab = _load_vocabulary()
        if not vocab:
            return None
        digest = _content_hash(vocab, _TARGET_MODEL)

        cache = _read_cache()
        if cache.get("hash") == digest and cache.get("vocabulary_id"):
            return cache["vocabulary_id"]

        import dashscope  # 延迟导入
        from dashscope.audio.asr import VocabularyService
        dashscope.api_key = api_key

        service = VocabularyService()
        vocabulary_id = service.create_vocabulary(
            target_model=_TARGET_MODEL,
            prefix=_PREFIX,
            vocabulary=vocab,
        )
        if vocabulary_id:
            _write_cache({"hash": digest, "vocabulary_id": vocabulary_id})
            print(f"[Vocab] 已创建热词表: {vocabulary_id}")
            return vocabulary_id
        return None
    except Exception as e:
        print(f"[Vocab] 热词注册失败(降级为无热词): {e}")
        return None


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--api_key", required=True)
    args = ap.parse_args()
    print("vocabulary_id =", get_vocabulary_id(args.api_key))
