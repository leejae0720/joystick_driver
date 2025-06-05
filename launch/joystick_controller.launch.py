from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
  config_path = os.path.join(
    get_package_share_directory('joystick_controller'),
    'config',
    'params.yaml'
  )

  load_nodes = GroupAction(
    actions=[
      Node(
        package='joystick_controller',
        executable='joystick_controller_node',
        name='joystick_controller_node',
        output='screen',
        parameters=[config_path],
      ),
    ]
  )

  ld = LaunchDescription()
  ld.add_action(load_nodes)
  return ld
