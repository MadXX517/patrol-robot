from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'car_report'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'data'), ['data/.gitignore']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yeahbot',
    maintainer_email='yeahbot@todo.todo',
    description='Visual event recording and GLM report generation for the patrol robot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'event_recorder = car_report.event_recorder:main',
            'report_generator = car_report.report_generator:main',
            'report_service = car_report.report_service:main',
        ],
    },
)
