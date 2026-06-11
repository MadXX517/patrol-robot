from glob import glob
import os

from setuptools import find_packages, setup


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
    description='Voice command bridge for the patrol robot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'voice_frontend = car_voice.voice_frontend:main',
            'voice_executor = car_voice.voice_executor:main',
        ],
    },
)
