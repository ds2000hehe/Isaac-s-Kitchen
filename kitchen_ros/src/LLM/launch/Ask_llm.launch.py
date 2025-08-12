from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os

def generate_launch_description():

    Model = ExecuteProcess(
            cmd=['python3', '/home/david/kitchen_ros/src/LLM/scripts/ask_with_image_gui.py'],
            output='screen'
        )
    img_sending  = ExecuteProcess(
            cmd=['python3', '/home/david/kitchen_ros/src/LLM/scripts/img_send.py'],
            output='screen'
        )

    
    return LaunchDescription([
        Model,
        img_sending
    ])
