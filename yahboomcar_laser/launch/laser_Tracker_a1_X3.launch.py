# laser_Tracker_a1_X3.launch.py
# ================================
# Seguidor de obstáculos completo para X3 com RPLIDAR A1.
# Mantém o robô a ResponseDist (padrão 0.55m) do objeto mais próximo usando PID.
#
# Nós lançados:
#   yahboomcar_bringup_X3_launch.py → driver X3 + bringup completo
#   sllidar_ros2/sllidar_launch.py  → driver RPLIDAR A1 (/scan)
#   laser_Tracker_a1_X3            → PID de seguimento (subscreve /scan, publica /cmd_vel)
#
# Ativar seguimento:
#   ros2 param set /laser_Tracker_a1 Switch false  # false = seguimento ativo
#   ros2 param set /laser_Tracker_a1 ResponseDist 0.55

from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    laser_Avoidance_node = Node(
        package='yahboomcar_laser',
        executable='laser_Tracker_a1_X3',
    )
    lidar_node = IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('sllidar_ros2'), 'launch'),
'/sllidar_launch.py'])
)
    bringup_node = IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('yahboomcar_bringup'), 'launch'),
'/yahboomcar_bringup_X3_launch.py'])
)
    
    launch_description = LaunchDescription([laser_Avoidance_node,lidar_node,bringup_node]) 
    return launch_description
