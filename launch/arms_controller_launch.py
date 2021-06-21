import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('arms_controller'),
        'config',
        'params.yaml'
        )
        
    node=Node(
        package = 'arms_controller',
        name = 'arms_controller_node',
        executable = 'arms_controller',
        parameters = [config]
    )
    ld.add_action(node)
    return ld
