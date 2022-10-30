
from tkinter.font import names
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package="turtlesim", executable="turtlesim_node",namespace="lyx",name="t1")
    ])
