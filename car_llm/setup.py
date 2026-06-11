from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'car_llm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # llm_main.py 经 launch 以 python3 直接运行;安装 src/ 与 config/ 到 share,
        # 保持 src/ 与 config/ 同级,使 chat_model_class 能按 ../config 找到 yaml。
        (os.path.join('share', package_name, 'src'), glob('src/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yeahbot',
    maintainer_email='yeahbot@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'app_hand_gesture_arm = car_app.app_hand_gesture_arm:main',
            'color_rect_pick = car_llm.color_rect_pick:main',
            'garbage_class = car_llm.garbage_class:main',
            'navi_color_pick = car_llm.navi_color_pick:main',
            'navi_color_pick_signal = car_llm.navi_color_pick_signal:main',
            'cli_chat = car_llm.cli_chat:main',
            'tts_test = car_llm.tts_test:main',
            'emotion_AI = car_llm.emotion_AI:main',
        ],
    },
)
