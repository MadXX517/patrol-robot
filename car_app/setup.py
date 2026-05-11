from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'car_app'

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
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'app_hand_gesture_arm = car_app.app_hand_gesture_arm:main',
            'app_object_tracking = car_app.app_object_tracking:main',
            'app_color_pick = car_app.app_color_pick:main',
            'app_color_pick_d = car_app.app_color_pick_d:main',
            'app_tag_stackup = car_app.app_tag_stackup:main',
            'app_tag_stackup_d = car_app.app_tag_stackup_d:main',
        ],
    },
)
