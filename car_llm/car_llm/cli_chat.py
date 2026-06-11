#!/usr/bin/env python3
from openai import OpenAI

# 初始化客户端
client = OpenAI(
    api_key="sk-a07b9af862154c89a34741c80db78f7a",  # 请提前 export DASHSCOPE_API_KEY=...
    base_url="https://dashscope.aliyuncs.com/compatible-mode/v1"
)

def main():
    print("=====  AI  终端交互助手已启动 =====")
    print("输入内容并回车即可对话（Ctrl+C 退出）")
    print("=================================\n")

    history = [
        {"role": "system", "content":"你是一个AI管家，"}
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
