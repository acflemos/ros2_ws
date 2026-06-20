# linefollow_X3.launch.py — Seguimento de linha colorida no X3 (bringup completo)
# ==================================================================================
# Lança: bringup X3 + RPLIDAR A1 + nó de seguimento de linha + joy_node.
#
# Pré-requisito: sllidar_ros2 instalado (ros-humble-sllidar-ros2).
# Nota: LaunchDescription duplicado no import — não causa erro mas é código morto.
#
# Uso:
#   ros2 launch yahboomcar_linefollow linefollow_X3.launch.py
#
# Para alterar LiDAR (ex: YDLIDAR X4), substituir sllidar_launch.py por ydlidar_ros2_driver.

from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Nó principal de seguimento de linha (câmera + HSV + PID + LiDAR avoidance)
    linefollow_node = Node(
        package='yahboomcar_linefollow',
        executable='follow_line_a1_X3',
    )

    # Driver do RPLIDAR A1 → publica /scan
    lidar_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('sllidar_ros2'), 'launch'),
            '/sllidar_launch.py'])
    )

    # Bringup completo do X3 (driver Arduino + IMU + EKF + robot_state_publisher)
    bringup_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('yahboomcar_bringup'), 'launch'),
            '/yahboomcar_bringup_X3_launch.py'])
    )

    # Driver do joystick → /joy → yahboom_joy_X3 (lançado pelo bringup) → /JoyState
    Joy_node = Node(
        package='joy',
        executable='joy_node',
    )

    return LaunchDescription([linefollow_node, lidar_node, bringup_node, Joy_node])
