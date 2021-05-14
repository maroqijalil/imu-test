import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
  webots = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(get_package_share_directory('webots_ros2_core'), 'launch', 'robot_launch.py')
    ),
    launch_arguments=[
      ('package', 'my_webots_driver'),
      ('executable', 'driver'),
      ('world', path_to_webots_world_file),
    ]
  )

  return LaunchDescription([
    webots
  ])