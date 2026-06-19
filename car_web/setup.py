from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'car_web'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'web'), glob('web/*.html') + glob('web/*.js') + glob('web/*.css')),
        (os.path.join('share', package_name, 'web', 'vendor'), glob('web/vendor/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yeahbot',
    maintainer_email='yeahbot@todo.todo',
    description='Browser dashboard for robot remote control and patrol event operations',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'dashboard_server = car_web.dashboard_server:main',
        ],
    },
)
