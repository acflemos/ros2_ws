# laser_Warning_a1_X3.launch.py
# ================================
# Aviso de obstáculos completo para X3 com RPLIDAR A1.
# Ativa buzzer quando obstáculo está dentro de ResponseDist e gira para enfrentar o obstáculo.
# NÃO faz desvio ativo — apenas avisa sonoramente e orienta o robô.
#
# Nós lançados:
#   yahboomcar_bringup_X3_launch.py → driver X3 + bringup completo
#   sllidar_ros2/sllidar_launch.py  → driver RPLIDAR A1 (/scan)
#   laser_Warning_a1_X3            → aviso + rotação (subscreve /scan, publica /cmd_vel + /Buzzer)

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
        executable='laser_Warning_a1_X3',
    )
    lidar_node = IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('sllidar_ros2'), 'launch'),
'/sllidar_launch.py'])
)
    bringup_node = IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('yahboomcar_bringup'), 'launch'),
'/yahboomcar_bringup_X3_launch.py'])
)
    
    launch_description = LaunchDescription([laser_Avoidance_node,lidar_node,bringup_node]) 
    return launch_description
