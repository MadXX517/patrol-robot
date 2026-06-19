# language_tools.py
# 明文重写版(替换原 pyarmor 加密模块),适配 Python 3.10。
# 仅提供 chat_model_class.py 实际调用的接口:remove_subsentence_if_included
#
# 用途:
#   - 去掉唤醒应答语(如"在")所在的子句
#   - 屏蔽/剔除含助手名("小甲")的子句,防止 TTS 自我激活
#
# 契约(由调用方反推):
#   remove_subsentence_if_included(keyword, text)              -> 删除含 keyword 的子句
#   remove_subsentence_if_included(keyword, text, replacement) -> 用 replacement 替换含 keyword 的子句

import re

# 子句分隔标点(中英文),用作切分点;切分时保留标点本身
_DELIMITERS = "，。！？；、,.!?;\n"
_SPLIT_PATTERN = re.compile("([" + re.escape(_DELIMITERS) + "])")


def _split_keep_delimiters(text):
    """把文本按标点切成 [子句, 标点, 子句, 标点, ...],标点保留。"""
    if not text:
        return []
    parts = _SPLIT_PATTERN.split(text)
    return [p for p in parts if p != ""]


def remove_subsentence_if_included(keyword, text, replacement=""):
    """
    若某个子句中包含 keyword:
      - replacement 为空 -> 删除该子句(连同其后紧跟的一个标点)
      - replacement 非空 -> 把该子句整体替换为 replacement(保留其后标点)

    keyword 为空或 text 为空时原样返回。
    """
    if not keyword or not text:
        return text if text is not None else ""

    pieces = _split_keep_delimiters(text)
    result = []
    i = 0
    while i < len(pieces):
        piece = pieces[i]
        is_delim = (len(piece) == 1 and piece in _DELIMITERS)

        if not is_delim and keyword in piece:
            # 命中含关键词的子句
            # 紧跟其后的标点(若有)
            trailing = ""
            if i + 1 < len(pieces) and len(pieces[i + 1]) == 1 and pieces[i + 1] in _DELIMITERS:
                trailing = pieces[i + 1]
                i += 1  # 跳过该标点

            if replacement:
                result.append(replacement + trailing)
            # replacement 为空:整段(含其后标点)丢弃
        else:
            result.append(piece)
        i += 1

    cleaned = "".join(result).strip()
    return cleaned


if __name__ == "__main__":
    # 简单自测
    assert remove_subsentence_if_included("在", "在，前进0.5米") == "前进0.5米"
    assert remove_subsentence_if_included("小甲", "你好小甲，抓取红色物块") == "抓取红色物块"
    print(remove_subsentence_if_included("小甲", "我是小甲，很高兴见到你", "(已屏蔽)"))
    print("language_tools self-test ok")
