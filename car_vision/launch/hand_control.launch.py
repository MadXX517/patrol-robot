from launch import LaunchDescription
from launch.actions import TimerAction, RegisterEventHandler, EmitEvent
from launch.actions import ExecuteProcess
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node

def generate_launch_description():

    # Call main() explicitly to avoid both stale console_script metadata and __main__ guard issues.
    hand_gesture_node = ExecuteProcess(
        cmd=[
            'python3', '-c', 'from car_vision.hand_gesture import main; main()',
            '--ros-args', '-p', 'machine_type:=Mec'
        ],
        output='screen'
    )

    return LaunchDescription([
        hand_gesture_node
    ])