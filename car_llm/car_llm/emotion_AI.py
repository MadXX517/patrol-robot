#!/usr/bin/env python3
import os

from openai import OpenAI

# 初始化客户端(API Key 从环境变量读取,请提前 export DASHSCOPE_API_KEY=...)
client = OpenAI(
    api_key=os.environ.get("DASHSCOPE_API_KEY"),
    base_url="https://dashscope.aliyuncs.com/compatible-mode/v1"
)

def main():
    print("=====  AI  情感分析小助手已启动 =====")
    print("输入内容并回车即可对话（Ctrl+C 退出）")
    print("=================================\n")

    history = [
        {"role": "system", "content":"请严格按照以下步骤思考（思考过程不要输出）："
        "1. 先完整理解字面意思\n"
        "2. 再判断是否有反讽、夸张、阴阳、矛盾等"
        "3. 列出所有情感线索（关键词、语气、表情符号等）"
        "4. 综合给出真实情感极性"
        "5. 打出0.0-1.0强度"}
    ]

    try:
        while True:
            user_input = input("\033[92m你:\033[0m ").strip()
            if not user_input:
                continue

            history.append({"role": "user", "content": user_input})

            print("\033[94m助手:\033[0m ", end="", flush=True)

            # 调用 API（流式输出）
            completion = client.chat.completions.create(
                model="qwen-plus",
                messages=history,
                stream=True,
                stream_options={"include_usage": False}
            )

            assistant_reply = ""

            for chunk in completion:
                delta = chunk.choices[0].delta
                if delta is not None and delta.content:
                    print(delta.content, end="", flush=True)
                    assistant_reply += delta.content

            print("\n")  # 换行
            history.append({"role": "assistant", "content": assistant_reply})

    except KeyboardInterrupt:
        print("\n退出助手，再见！")

if __name__ == "__main__":
    main()
