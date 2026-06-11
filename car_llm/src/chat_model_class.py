from openai import OpenAI
import json,os
import time
from language_tools import remove_subsentence_if_included
import yaml

msg_receivce = "在"  # 默认提示语，可以根据需要修改
wake_up_words = "小星" # 唤醒词/助手名,用来屏蔽防止自我激活

class LLMCommandParser():
    def __init__(self, api_key=None,base_url="https://dashscope.aliyuncs.com/compatible-mode/v1", yaml_file=None,tts_queue=None, ros_control_queue=None, model="qwen-flash"):
        self.tts_queue=tts_queue
        self.ros_control_queue=ros_control_queue
        if not api_key:
            raise ValueError("请输入DashScope API Key")

        self.client = OpenAI(base_url=base_url,api_key=api_key)
        self.model = model
        if yaml_file is None:
            raise ValueError("无法读取yaml，请提供prompt配置文件的正确路径")
        current_dir = os.path.dirname(os.path.abspath(__file__))
        parent_dir = os.path.dirname(current_dir)
        file_path = os.path.join(parent_dir, "config", yaml_file)
        if os.path.exists(file_path):
            with open(file_path, "r", encoding="utf-8") as f:
                config = yaml.safe_load(f)
        
            self.control_type = config["control_type"]
            print("control_type:",self.control_type)
            self.system_prompt = config["system_prompt"]
            self.context_template = config["context_info"]
        else:
            raise FileNotFoundError(f"prompt配置文件未找到: {file_path}")

        self.buffer=""
        self.brace_count = 0
        self.in_json = False

    def parse_command(self, user_command, enable_tts=False):
        """
        输入用户自然语言命令，返回解析后的 JSON（作为字符串）
        """
        user_command=remove_subsentence_if_included(msg_receivce,user_command)
        user_command=remove_subsentence_if_included(wake_up_words,user_command)

        # # 🔧 获取当前系统时间（格式可调整）
        # current_time = datetime.now().strftime("%Y年%m月%d日 %H点%M分")

        # 注入环境上下文，给LLM使用，存储一些常用的物体、位置
        context_info = self.context_template

        dynamic_prompt = self.system_prompt + "\n" + "以下是机器状态信息，请结合这些信息理解用户指令：\n" + context_info

        response = self.client.chat.completions.create(
            model=self.model,
            messages=[
                {"role": "system", "content": dynamic_prompt},
                {"role": "user", "content": user_command}
            ],
            stream=True  # 启用流式输出
        )

        # time.sleep(2.0)

        # 重置缓冲区
        self.buffer = ""
        self.json_objects = []
        message_buffer = ""  # ✅ 在这里初始化，避免未定义错误

        # 流式处理响应
        for chunk in response:
            if hasattr(chunk, "choices") and chunk.choices:
                delta = chunk.choices[0].delta
                content = getattr(delta, "content", None)
                if content:
                    print(content, end='', flush=True)  # 打印原始内容
                    self.buffer += content

                    # ✅ 实时检测 message 字段
                    if message_buffer == "":
                        msg_start = self.buffer.find('"message"')
                        if msg_start != -1:
                            msg_quote_start = self.buffer.find('"', msg_start + 9)
                            msg_quote_end = self.buffer.find('"', msg_quote_start + 1)
                            if msg_quote_start != -1 and msg_quote_end != -1:
                                message_text = self.buffer[msg_quote_start + 1:msg_quote_end]

                                # 只在新message出现时播报
                                if message_text and message_text != message_buffer:
                                    message_buffer = message_text
                                    message = remove_subsentence_if_included(wake_up_words, message_text, "(已屏蔽)")
                                    print(f"\n📢 实时提示信息: {message}")
                                    if enable_tts and self.tts_queue is not None:
                                        self.tts_queue.put(message)

                                # 从缓冲中移除已处理的 message 段
                                self.buffer = self.buffer[msg_quote_end + 1:]

                    self._extract_json_objects()
                    
                    # 实时提取并处理 JSON 对象
                    if self.json_objects:
                        if "step" in self.json_objects[0]:
                            if self.ros_control_queue is not None:
                                self.ros_control_queue.put(self.json_objects[0])
                            print("\n👉 动作加入队列:", self.json_objects[0]["step"])
                            self.json_objects.pop(0)  # 移除已处理的对象
                            
    def _extract_json_objects(self):
        """从缓冲区提取完整的 step JSON 对象"""
        while True:
            start_idx = self.buffer.find('{')
            if start_idx == -1:
                break

            brace_count = 0
            end_idx = -1
            for i in range(start_idx, len(self.buffer)):
                if self.buffer[i] == '{':
                    brace_count += 1
                elif self.buffer[i] == '}':
                    brace_count -= 1
                    if brace_count == 0:
                        end_idx = i
                        break

            if end_idx == -1:
                break

            json_str = self.buffer[start_idx:end_idx + 1]
            try:
                json_obj = json.loads(json_str)
                # ✅ 只处理 step
                if "step" in json_obj:
                    self.json_objects.append(json_obj)
            except json.JSONDecodeError:
                pass

            # 从缓冲中移除已处理内容
            self.buffer = self.buffer[end_idx + 1:]


# 示例使用.
if __name__ == '__main__':
    parser = LLMCommandParser(api_key="sk-a07b9af862154c89a34741c80db78f7a",model="qwen-flash") # max比turbo慢三倍，turbo不再更新，建议使用flash
    # parser = LLMCommandParser(api_key="048d77d4-f25c-4255-90d8-299cef2c2fb3",base_url="https://ark.cn-beijing.volces.com/api/v3",model="doubao-1-5-lite-32k-250115")

    # user_input = "你好小甲，下午好，将红色物块搬运到绿色物块上方，然后机械臂移动到位置(0.2,0.2,0.1)处，夹爪开合一次，第一个关节向左旋转30度。"
    # user_input = "你好小甲，抓取红色的物块"
    # user_input = "你好小甲，第一个关节向右转30度，第二个关节向前转20度，然后夹爪打开，夹爪关闭，最后恢复原位"
    # user_input = "你好小甲，将红色物块搬到绿框上,把绿色物块放到蓝框上，把蓝色物块放在红框上"
    # user_input = "你好小甲，将红色物块搬到蓝色框上，从蓝色框搬到绿色框，再从绿色框搬到红色框"
    # user_input = "你好小甲，抓取红色物块，不放下"
    # user_input = "你好小甲，放置到绿色框上"
    # user_input = "你好小甲，抓取红色物块，不放下,第一关节向左10度，第二关节向后10度，然后恢复原位".
    # user_input = "你好小甲，恢复原位"
    # user_input = "你好小甲，依次归类颜色"

    # user_input = "你好小甲，自我介绍一下"
    # user_input = "你好小甲，请摇摇头，再点点头,最后比个耶"
    # user_input = "你好小甲，依次归类绿色，蓝色，红色"
    # user_input = "依次抓取有害垃圾、可回收垃圾、其他垃圾、厨余垃圾"

    # user_input = "导航到动物园，告诉我动物园里面有什么动物，然后导航到篮球场，看看篮球场里面有什么，最后回到出发区"
    # user_input = "将红色物块放置到蓝色框中，然后将绿色物块放置到红色框中，最后将蓝色物块放置到绿色框中"
    # user_input = "向前10厘米，抓取长方形物块，放到蓝色的框中，小车原地转一圈"
    # user_input = "抓取红色物块，导航到家，然后放下红色物块"

    # user_input = "帮我看看有什么水果"
    # user_input = "先抓取绿色的方块，然后放置到红色的框中"
    # user_input = "先抓取绿色的方块，然后放置到红色的框中"
    # user_input = "到动物园看看有什么动物，去水下科研中心跟研究人员见面，去篮球场里看看有人在打篮球吗，最后回到出发区"
    user_input = "追踪小鹿玩具"

    print("start")
    start=time.time()
    result_json = parser.parse_command(user_input, enable_tts=False)
    print("used:",time.time()-start)

    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("程序已终止。")
