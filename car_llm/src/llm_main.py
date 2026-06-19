# llm大模型 主线程 运行
from multiprocessing import Process, Queue
import voice2text_class,chat_model_class,text2voice_class
import time
import aibox_tty
import sys
import argparse

# 是否接入机器人控制(机械臂/底盘)。第一阶段默认关闭,只做语音对话。
# 由命令行 --enable_ros_control 覆盖;在 __main__ 中按需 import ros_control / rclpy。
# (Linux fork 启动的子进程会继承此处设置的全局变量)
ros_control_enable = False
ros_control = None
rclpy = None

# Run as script

def parse_command_process(api_key, base_url, yaml_file, model, command_queue, tts_queue, ros_control_queue):
    """在独立进程中运行的命令解析函数"""
    # 在子进程中创建LLM实例
    chat_parser = chat_model_class.LLMCommandParser(api_key=api_key,base_url=base_url,yaml_file=yaml_file,tts_queue=tts_queue, ros_control_queue=ros_control_queue,model=model)
    
    try:
        while True:
            try:
                # 从队列获取需要解析的文本
                text = command_queue.get()
                # 如果收到None，说明要退出进程
                if text is None:
                    break
                print(f"子进程解析命令: {text}")
                chat_parser.parse_command(text, enable_tts=True)
                time.sleep(0.01)
            except KeyboardInterrupt:
                break  # 捕获中断，跳出循环后清理
    except Exception as e:
        print(f"解析进程出错: {e}")

def ttyUSB_process(port,baudrate,data_stamp_queue,send_queue):
    """串口通信进程"""
    try:
        # 串口初始化
        serial_comm=aibox_tty.CrossPlatformSerial(port=port,baudrate=baudrate)

        # 打开串口
        if serial_comm.open():
            try:
                serial_comm.send_data("llm_mode")
                
                # 循环读取数据
                print("开始监听串口数据，按Ctrl+C退出...")
                data_stamp_queue.put([time.time(),True])
                while True:
                    try:
                        data = serial_comm.read_data()
                        if data:
                            data_stamp_queue.put([time.time(),data])
                        elif data is None:
                            if serial_comm.attempt_times > 0:
                                print("连接设备成功")
                                serial_comm.send_data("llm_mode")
                                serial_comm.attempt_times = 0
                        elif data==False:
                            serial_comm.reconnect()
                            time.sleep(3.0)

                        if send_queue.qsize()>0:
                            data=send_queue.get()
                            if data is None:
                                break
                            serial_comm.send_data(data)
                        time.sleep(0.01)
                    except KeyboardInterrupt:
                        break  # 捕获中断，跳出循环后清理
            finally:
                # 关闭串口
                serial_comm.close()
        else:
            
            print("无法打开串口，程序退出")
    except Exception as e:
        print(f"串口进程出错: {e}")
    finally:
        data_stamp_queue.put([time.time(),False])

def ros_control_process(ros_control_queue,tts_queue):
    if ros_control_enable:
        try:
            rclpy.init(args=None)
            ros_controller = ros_control.ros_control(ros_control_queue=ros_control_queue,tts_queue=tts_queue)
            while rclpy.ok():
                try:
                    ros_controller.loop()
                    time.sleep(0.05)
                except KeyboardInterrupt:
                    break  # 捕获中断，跳出循环                
        except Exception as e:
            print(f"ROS控制进程出错: {e}")
        finally:
            rclpy.shutdown()
    else:
        print("rclpy未加载，此时无法控制机器人")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='LLM Main Program')
    parser.add_argument('--api_key', type=str, default="sk-a07b9af862154c89a34741c80db78f7a")
    parser.add_argument('--base_url', type=str, default="https://dashscope.aliyuncs.com/compatible-mode/v1")
    parser.add_argument('--yaml_file', type=str, default="chat_prompt_car.yaml")
    parser.add_argument('--aibox_serial_port', type=str, default='/dev/aibox')
    parser.add_argument('--baudrate', type=int, default=115200)
    parser.add_argument('--sound_volume', type=int, default=85)
    parser.add_argument('--llm_model', type=str, default='qwen-flash')
    parser.add_argument('--tts_voice', type=str, default='Neil')
    parser.add_argument('--tts_model', type=str, default='qwen3-tts-flash')
    parser.add_argument('--asr_model', type=str, default='fun-asr-realtime')
    parser.add_argument('--max_sentence_silence', type=int, default=800)
    # 是否接入机器人控制(机械臂/底盘);第一阶段默认关闭,仅语音对话
    # 取值型(便于 launch 传参):true/false、1/0、yes/no、on/off
    parser.add_argument('--enable_ros_control', type=str, default='false')
    # 麦克风触发方式:
    #   continuous = 持续聆听(常开麦,播报时自动暂停),适用于无按钮的 aibox(默认)
    #   button     = 按 aibox 按钮"按住说话"(需 aibox 通过串口发 llm_mic_start/end)
    parser.add_argument('--mic_trigger', type=str, default='continuous',
                        choices=['continuous', 'button'])
    # 唤醒词:仅在听到该词后才进入对话;留空("")则不需要唤醒词,听到即应答。
    parser.add_argument('--wake_word', type=str, default='小星')
    # 唤醒后无新指令的保持时长(秒),超时自动回到休眠
    parser.add_argument('--awake_timeout', type=float, default=20.0)

    args = parser.parse_args()

    # 运行期决定是否启用机器人控制,并按需导入(子进程经 fork 继承)
    ros_control_enable = str(args.enable_ros_control).strip().lower() in ('1', 'true', 'yes', 'on')
    if ros_control_enable:
        import ros_control
        import rclpy
        print("机器人控制:已启用")
    else:
        print("机器人控制:已关闭(仅语音对话)")

    # 使用解析的参数
    parser_api_key = args.api_key
    parser_base_url = args.base_url
    parser_yaml_file = args.yaml_file
    # aibox 串口独立于控制开关:即便关闭机器人控制,仍保留 aibox 按钮"按住说话"握手
    parser_serial_port = args.aibox_serial_port
    print("aibox_serial_port:",parser_serial_port,"type:",type(parser_serial_port))
    parser_baudrate=args.baudrate
    parser_sound_volume=args.sound_volume
    parser_llm_model = args.llm_model
    parser_tts_voice = args.tts_voice
    parser_tts_model = args.tts_model
    parser_asr_model = args.asr_model
    parser_max_sentence_silence = args.max_sentence_silence
    parser_mic_trigger = args.mic_trigger
    print("麦克风触发方式:", parser_mic_trigger)

    # 唤醒词与同音变体(为容错 ASR 识别偏差;小星->小星星/晓星)
    wake_word = (args.wake_word or "").strip()
    awake_timeout = args.awake_timeout
    wake_variants = []
    if wake_word:
        wake_variants = [wake_word]
        if wake_word == '小星':
            wake_variants += ['小星星', '晓星', '小兴']
    print("唤醒词:", wake_word if wake_word else "(无,直接对话)")

    # 通过queue传递接收到的消息，voice2text -> chat       voice2text -> text2voice       chat -> text2voice      chat -> arm_controller
    # continuous(无按钮):开机即常开麦;button:等待 aibox 按钮触发
    llm_mic_running = (parser_mic_trigger == 'continuous')
    # --- 主程序入口 ---
    asr_result_queue=Queue()
    command_queue=Queue()
    tts_result_queue=Queue()
    ttyUSB_data_stamp_queue=Queue()
    ttyUSB_send_queue=Queue()
    ros_control_queue=Queue()
    
    # --- 启动语音模块串口进程 ---
    ttyUSB_process_handle=Process(target=ttyUSB_process,args=(parser_serial_port,parser_baudrate,ttyUSB_data_stamp_queue,ttyUSB_send_queue))
    ttyUSB_process_handle.start()

    while True:
        try:
            if ttyUSB_data_stamp_queue.qsize() > 0:
                data_stamp=ttyUSB_data_stamp_queue.get()
                if data_stamp[1] == True:
                    break
                elif data_stamp[1] == False:
                    sys.exit()
            time.sleep(1.0)
        except KeyboardInterrupt:
            sys.exit()
            print("退出程序")
    
    # --- 主程序中启动语音转文字ASR 文字转语音TTS ---
    asr=voice2text_class.RealTimeASR(api_key=parser_api_key,result_queue=asr_result_queue,model=parser_asr_model,max_sentence_silence=parser_max_sentence_silence)
    asr.start()

    tts_player = text2voice_class.TTSPlayer(api_key=parser_api_key,voice=parser_tts_voice,volume_percent=parser_sound_volume,rate=24000,model=parser_tts_model)

    # --- 启动动作执行进程(仅在启用机器人控制时) ---
    ros_control_process_handle = None
    if ros_control_enable:
        ros_control_process_handle = Process(target=ros_control_process,args=(ros_control_queue,tts_result_queue))
        ros_control_process_handle.start()
        parser_ros_control_queue = ros_control_queue
    else:
        # 关闭控制:不向 ros_control_queue 投递,避免无人消费而堆积
        parser_ros_control_queue = None

    # --- 创建并启动解析命令的进程 ---
    parser_process_handle = Process(target=parse_command_process,args=(parser_api_key,parser_base_url,parser_yaml_file,parser_llm_model,command_queue,tts_result_queue,parser_ros_control_queue))
    parser_process_handle.start()

    # asr_result_queue.put("小车前进0.27米，抓取橙色的圆柱体，然后放置到蓝色的框中")
    # asr_result_queue.put("抓取红色的方块，然后放置到蓝色的框中")
    # asr_result_queue.put("抓取橙色的圆柱体，然后放置到蓝色的框中")
    # asr_result_queue.put("追踪前方狮子")
    # asr_result_queue.put("向左平移10厘米，向后移动15厘米，原地旋转180度")
    # asr_result_queue.put("向前移动45厘米，向左平移30厘米，原地旋转180度，机械臂第一关节向右旋转45度")
    # asr_result_queue.put("向前移动20厘米，向左平移20厘米，原地旋转350度，云台向右旋转45度，最后云台复位")
    # asr_result_queue.put("自我介绍一下")
    # asr_result_queue.put("抓取有害垃圾标志方块,放置到灰色的框中")
    # asr_result_queue.put("去篮球场里看看谁在打篮球，去水下科研中心跟研究人员见面问好，到动物园看看有什么动物，最后回到出发区")


    def _strip_wake(s):
        """从识别文本中去掉唤醒词及其前后标点,返回剩余指令。"""
        out = s
        for w in wake_variants:
            out = out.replace(w, '')
        return out.strip('，。！？、；,.!?; \t')

    # 逐句唤醒:每条指令都需带唤醒词(机器人自身回复不含唤醒词,从根本上杜绝回授自循环)
    try:
        while True:
            if tts_result_queue.qsize()>0:
                tts_result=tts_result_queue.get()
                # 播报前暂停麦克风,避免把自己的 TTS 当成输入(防自激励)
                asr.audio_loop(False)
                tts_player.say(tts_result)
                # 关键:say() 返回后扬声器缓冲/房间余音仍在响,先静默缓冲让其衰减,
                # 期间麦保持暂停,再清空残留,避免把自己的声音/回声当成新指令(防回授死循环)
                time.sleep(0.8)
                if parser_mic_trigger == 'continuous':
                    while not asr_result_queue.empty():
                        asr_result_queue.get()
                    llm_mic_running = True

            if ttyUSB_data_stamp_queue.qsize()>0:
                data_stamp=ttyUSB_data_stamp_queue.get()
                if data_stamp[1] == 'llm_mic_start':
                    llm_mic_running=True

            asr.audio_loop(llm_mic_running) # 运行在主线程中

            # 检查是否有ASR结果
            while not asr_result_queue.empty():
                if parser_mic_trigger == 'button':
                    llm_mic_running=False # 按钮模式:识别到结果后暂停,等下次按键
                    ttyUSB_send_queue.put('llm_mic_end')

                text = asr_result_queue.get()
                print("ASR Result:", text)
                norm = text.replace(' ', '')

                if not wake_word:
                    command_queue.put(text)  # 无唤醒词:直接处理
                    continue

                # 逐句唤醒:必须含唤醒词,否则忽略(过滤环境语音 + 机器人自身回声)
                if not any(w in norm for w in wake_variants):
                    print('\033[1;33m[忽略] 未含唤醒词「%s」\033[0m' % wake_word)
                    continue

                remainder = _strip_wake(norm)
                if remainder:
                    print('\033[1;32m[唤醒] 指令: %s\033[0m' % remainder)
                    command_queue.put(remainder)   # "小星,讲个笑话" -> 执行剩余指令
                else:
                    print('\033[1;32m[唤醒] 应答\033[0m')
                    tts_result_queue.put('在,请说')  # 仅唤醒词 -> 应答

            # 短暂休眠，减少CPU占用
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("收到退出信号，正在清理资源...")
    finally:
        # 停止ASR
        asr.stop()
        # 停止TTS
        tts_player.close()
        
        # 发送退出信号给解析进程并等待其结束
        command_queue.put(None)
        ttyUSB_send_queue.put(None)

        ttyUSB_process_handle.join(timeout=5)
        parser_process_handle.join(timeout=5)
        
        # 如果进程仍未结束，强制终止
        if ttyUSB_process_handle.is_alive():
            ttyUSB_process_handle.terminate()

        if parser_process_handle.is_alive():
            parser_process_handle.terminate()

