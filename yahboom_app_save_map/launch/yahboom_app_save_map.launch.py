# yahboom_app_save_map.launch.py — Lança o servidor de salvamento de mapa via app
# =================================================================================
# Inicia o nó 'server' (yahboom_app_save_map.py) que aguarda pedidos do app mobile.
# Pré-requisito: Nav2 + SLAM em execução para que map_saver_cli encontre o mapa.
#
# Uso:
#   ros2 launch yahboom_app_save_map yahboom_app_save_map.launch.py
from launch import LaunchDescription
#from launch.actions
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    
    yahboom_app_save_map_node = Node(
        package='yahboom_app_save_map',
        executable="server",
        output="screen"
    )

    return LaunchDescription(
        [
            yahboom_app_save_map_node,
        ]
    )
