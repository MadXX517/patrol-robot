from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'car_patrol'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={
        package_name: ['weights/*'],
    },
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='MadXX517',
    maintainer_email='cockmagiccn@gmail.com',
    description='Patrol assistant: person-follow (lock & keep distance), gimbal tracking; later gesture/cruise.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'person_follow = car_patrol.person_follow:main',
        ],
    },
)
