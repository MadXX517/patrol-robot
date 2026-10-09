#!/usr/bin/env python3
# voice_main.py — car_voice 语音助手主程序(多进程编排)。
# 复用 car_llm 的成熟组件:
#   text2voice_class.TTSPlayer       文字转语音(qwen-tts)
#   voice2text_class.RealTimeASR     实时语音转文字(paraformer)
#   chat_model_class.LLMCommandParser 自然语言 -> {message, step} JSON
# 把 ros_control(机械臂执行器)换成 car_voice.web_control(POST dashboard HTTP API)。
#
# 进程结构(沿用 llm_main.py 思路,去掉 aibox 串口/机械臂):
#   主进程         : ASR 门控 + 唤醒词逻辑 + TTS 播放 + 开关(tts/asr)切换
#   parse 进程     : LLMCommandParser.parse_command(把 message 投 tts_queue, step 投 ros_queue)
#   web_control 进程: 消费 step -> dashboard;订阅 /voice/announce 状态播报、/voice/control 开关
#
# 开关语义(对应 web 两个按钮):
#   语音播报(tts_on): 控制是否真正出声(状态播报 + 命令回执)。关 -> 静音。
#   语音触发(asr_on): 控制麦克风是否聆听。关 -> 不听不识别。
# 音频:一律走 pulse 默认设备(input/output_device_index=None),见 elf2-audio-pulse-default。

import argparse
import os
import sys
import time
from multiprocessing import Process, Queue

# 把 car_llm 安装目录的 src/ 加入 import 路径(复用其类)
def _add_car_llm_src():
    try:
        from ament_index_python.packages import get_package_share_directory
        src = os.path.join(get_package_share_directory('car_llm'), 'src')
        if os.path.isdir(src) and src not in sys.path:
            sys.path.insert(0, src)
    except Exception as exc:
        print('[voice] 无法定位 car_llm/src: %s' % exc)

_add_car_llm_src()


def parse_command_process(api_key, base_url, yaml_file, model,
                          command_queue, tts_queue, ros_control_queue):
    """子进程:LLM 解析自然语言 -> message(投 tts) + step(投 ros_control_queue)。"""
    import chat_model_class
    parser = chat_model_class.LLMCommandParser(
        api_key=api_key, base_url=base_url, yaml_file=yaml_file,
        tts_queue=tts_queue, ros_control_queue=ros_control_queue, model=model)
    try:
        while True:
            text = command_queue.get()
            if text is None:
                break
            try:
                parser.parse_command(text, enable_tts=True)
            except Exception as exc:
                print('[voice] 解析出错: %s' % exc)
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass


def web_control_process(ros_control_queue, tts_queue, control_queue, dashboard_url):
    """子进程:跑 web_control ROS 节点,把 step POST 给 dashboard,并桥接播报/开关。"""
    import rclpy
    from car_voice.web_control import WebControl
    rclpy.init(args=None)
    node = WebControl(ros_control_queue=ros_control_queue, tts_queue=tts_queue,
                      control_queue=control_queue, dashboard_url=dashboard_url)
    try:
        while rclpy.ok():
            node.loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _truthy(v):
    return str(v).strip().lower() in ('1', 'true', 'yes', 'on')


def build_args():
    ap = argparse.ArgumentParser(description='car_voice 语音助手')
    # API Key 默认从环境变量 DASHSCOPE_API_KEY 读取,也可用 --api_key 显式传入
    ap.add_argument('--api_key', default=os.environ.get('DASHSCOPE_API_KEY', ''))
    ap.add_argument('--base_url', default='https://dashscope.aliyuncs.com/compatible-mode/v1')
    ap.add_argument('--yaml_file', default='chat_prompt_patrol.yaml')
    ap.add_argument('--llm_model', default='qwen-flash')
    ap.add_argument('--tts_voice', default='Serena')
    ap.add_argument('--tts_model', default='qwen3-tts-flash')
    ap.add_argument('--asr_model', default='paraformer-realtime-v2')
    ap.add_argument('--sound_volume', type=int, default=90)
    ap.add_argument('--max_sentence_silence', type=int, default=800)
    ap.add_argument('--wake_word', default='小星')
    ap.add_argument('--dashboard_url', default='http://127.0.0.1:8000')
    # 初始开关:语音播报 / 语音触发(运行中可经 /voice/control 实时切换)
    ap.add_argument('--enable_tts', default='true')
    ap.add_argument('--enable_asr', default='true')
    # 忽略 ROS 注入的 --ros-args 等未知参数
    args, _ = ap.parse_known_args()
    return args


def wake_variants_of(wake_word):
    if not wake_word:
        return []
    variants = [wake_word]
    if wake_word == '小星':
        # paraformer 常把"小星(xīng)"听成同音/近音词,一并接受以免漏唤醒。
        # 误触代价低(裸唤醒只回"在,请说"),漏唤醒代价高,故从宽。
        variants += ['小星星', '晓星', '小兴', '小新', '小心', '小型',
                     '小欣', '小醒', '消息', '晓兴']
    # 去重保序
    seen, out = set(), []
    for v in variants:
        if v and v not in seen:
            seen.add(v)
            out.append(v)
    return out


def main():
    args = build_args()
    import voice2text_class
    import text2voice_class

    wake_word = (args.wake_word or '').strip()
    wake_variants = wake_variants_of(wake_word)
    # 剥离唤醒词时按长度降序,先去长的(如"小星星")再去短的(如"小星"),避免残留
    wake_variants_long_first = sorted(wake_variants, key=len, reverse=True)
    tts_on = _truthy(args.enable_tts)
    asr_on = _truthy(args.enable_asr)

    # 开机自查 dashboard 权威开关,消除"进程默认全开 vs dashboard 单开"竞态
    try:
        import requests
        r = requests.get(args.dashboard_url.rstrip('/') + '/api/status', timeout=4)
        if r.status_code == 200:
            st = r.json()
            if 'tts_on' in st:
                tts_on = bool(st['tts_on'])
            if 'asr_on' in st:
                asr_on = bool(st['asr_on'])
            print('[voice] 已自查 dashboard 开关: 播报=%s 触发=%s' % (tts_on, asr_on))
    except Exception as exc:
        print('[voice] 自查 dashboard 失败(用启动默认): %s' % exc)

    def strip_wake(s):
        out = s
        for w in wake_variants_long_first:
            out = out.replace(w, '')
        return out.strip('，。！？、；,.!?; \t')

    # 队列
    asr_result_queue = Queue()
    command_queue = Queue()
    tts_queue = Queue()
    ros_control_queue = Queue()
    control_queue = Queue()   # web_control 把 /voice/control 信号转回来

    # ASR / TTS(走 pulse 默认设备,index=None)
    asr = voice2text_class.RealTimeASR(
        api_key=args.api_key, result_queue=asr_result_queue,
        model=args.asr_model, max_sentence_silence=args.max_sentence_silence,
        input_device_index=None)
    asr.start()
    tts_player = text2voice_class.TTSPlayer(
        api_key=args.api_key, voice=args.tts_voice, volume_percent=args.sound_volume,
        rate=24000, model=args.tts_model, output_device_index=None)

    # 子进程:LLM 解析 + web_control
    parse_proc = Process(target=parse_command_process, args=(
        args.api_key, args.base_url, args.yaml_file, args.llm_model,
        command_queue, tts_queue, ros_control_queue))
    parse_proc.start()
    web_proc = Process(target=web_control_process, args=(
        ros_control_queue, tts_queue, control_queue, args.dashboard_url))
    web_proc.start()

    print('[voice] 启动完成 | 播报=%s 触发=%s 唤醒词=%s' % (tts_on, asr_on, wake_word or '无'))
    if tts_on:
        tts_queue.put('系统就绪，我是巡逻机器人小星。')

    try:
        while True:
            # 1) 实时开关(dashboard 经 /voice/control -> web_control -> control_queue)
            while not control_queue.empty():
                cmd = control_queue.get()
                if cmd == 'tts_on':
                    tts_on = True
                elif cmd == 'tts_off':
                    tts_on = False
                elif cmd == 'asr_on':
                    asr_on = True
                elif cmd == 'asr_off':
                    asr_on = False
                elif cmd.startswith('voice:'):
                    name = cmd.split(':', 1)[1].strip()
                    if name:
                        tts_player.voice = name      # 下一句发声即用新音色
                        print('[voice] 音色切换: %s' % name)
                elif cmd.startswith('volume:'):
                    try:
                        vol = max(0, min(100, int(cmd.split(':', 1)[1])))
                        tts_player.volume = vol / 100.0
                        print('[voice] 音量切换: %d%%' % vol)
                    except ValueError:
                        pass
                print('[voice] 开关更新: 播报=%s 触发=%s' % (tts_on, asr_on))

            # 2) 播报队列(关播报时丢弃,保持静音)
            if not tts_queue.empty():
                text = tts_queue.get()
                if tts_on:
                    asr.audio_loop(False)            # 播报时关麦,防回授
                    tts_player.say(text)
                    time.sleep(0.6)                  # 余音衰减
                    while not asr_result_queue.empty():
                        asr_result_queue.get()       # 清掉播报期间误采

            # 3) 麦克风门控:仅在 开触发 且 不在播报 时聆听
            asr.audio_loop(asr_on)

            # 4) 处理识别结果(唤醒词过滤)
            while not asr_result_queue.empty():
                text = asr_result_queue.get()
                if not asr_on:
                    continue
                norm = text.replace(' ', '')
                print('[voice] ASR: %s' % text)
                if wake_word and not any(w in norm for w in wake_variants):
                    continue                          # 无唤醒词,忽略(防误触+防回声)
                remainder = strip_wake(norm) if wake_word else text
                if remainder:
                    command_queue.put(remainder)      # 交 LLM 解析
                else:
                    tts_queue.put('在，请说')          # 仅唤醒词 -> 应答

            time.sleep(0.05)
    except KeyboardInterrupt:
        print('[voice] 退出中...')
    finally:
        asr.stop()
        tts_player.close()
        command_queue.put(None)
        parse_proc.join(timeout=5)
        if parse_proc.is_alive():
            parse_proc.terminate()
        if web_proc.is_alive():
            web_proc.terminate()


if __name__ == '__main__':
    main()
