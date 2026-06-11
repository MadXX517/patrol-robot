# carLLM

#### 介绍
多模态大模型

#### 参考
API调用：阿里系大模型 通义千问

​	大语言模型LLM：qwen-plus

https://help.aliyun.com/zh/model-studio/use-qwen-by-calling-api?spm=a2c4g.11186623.help-menu-2400256.d_2_1_0.429c215dARkkKU

​	语音合成模型：qwen-tts

https://help.aliyun.com/zh/model-studio/qwen-tts-api?spm=a2c4g.11186623.help-menu-2400256.d_2_5_5.c81b54e0eZttO5

​	实时语音转文字：paraformer-realtime-v2

https://help.aliyun.com/zh/model-studio/paraformer-real-time-speech-recognition-python-sdk?spm=a2c4g.11186623.0.0.7ae1108aISrg3N

#### 架构：

Nothing To Say.

#### 安装环境

**Python >= 3.8** 

Jetson Nano 4GB
Ubuntu20.04 
ROS2 Foxy

必备python库安装命令：

```
pip install pyaudio dashscope numpy openai scikit-learn pyyaml argparse pyserial
```

参考版本：
Python==3.8.10

pyaudio==0.2.14
dashscope=1.23.4
openai==1.84.0
pyserial==3.4

#### 使用说明
一.在树莓派jetson等设备上运行ros2
1.car_llm文件夹放入工作空间的src文件夹中
2.colcon build编译源码
3.ros2 launch car_llm car_llm.launch.py

二.如何在非ROS2环境上运行，如windows，mac等支持python3的系统
1.cd car_llm/src
2.将ros_control_enable改为False （如果需要接入ros2控制，改为True）
3.python llm_main.py (python3 llm_main.py)

#### 联系方式
感谢