from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'car_voice'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yeahbot',
    maintainer_email='yeahbot@todo.todo',
    description='语音播报 + 语音触发,复用 car_llm 的 TTS/ASR/LLM,命令转 dashboard HTTP 调用',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'voice_main = car_voice.voice_main:main',
        ],
    },
)
