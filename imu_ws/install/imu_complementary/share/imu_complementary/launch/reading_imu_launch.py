from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
  reading_imu = Node(
    package='imu_complementary',
    executable='imu_main',
    output='screen',
    remappings=[
      ('imu/data_raw', 'imu')
    ]
  )

  return LaunchDescription([
    reading_imu
  ])