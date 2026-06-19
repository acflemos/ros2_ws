# map_gmapping_4ros_s2_launch.py
# ================================
# SLAM com gmapping para LiDARs YDLIDAR X4 (4ROS) e RPLIDAR S2.
# Ambos geram mais de 1440 pontos por frame — acima do limite do gmapping.
# Por isso inclui o nó scan_filter que reduz a 1/2 (publica /downsampled_scan).
#
# Cadeia de nós:
#   laser_bringup_launch → bringup do hardware + driver do LiDAR (/scan)
#   scan_filter          → /scan → /downsampled_scan (mantém 1 de cada 2 pontos)
#   slam_gmapping        → consome /downsampled_scan e /tf, produz /map + TF map→odom
#
# O slam_gmapping é remapeado para ler /downsampled_scan em vez de /scan.
#
# Relevância para robodog2:
#   Não reutilizar — gmapping está deprecado no ROS2 Humble.
#   Preferir slam_toolbox (suporta qualquer densidade de pontos sem filtro).

from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    laser_bringup_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('yahboomcar_nav'), 'launch'),
        '/laser_bringup_launch.py'])
    )
    # 4ROS e s2 produzem >1440 pts/frame — filtrar para gmapping aceitar
    scan_filter_node = Node(
        package='yahboomcar_nav',
        executable='scan_filter',
    )

    slam_gmapping_node = Node(
            package='slam_gmapping', 
            executable='slam_gmapping', 
            output='screen', 
            parameters=[os.path.join(get_package_share_directory("slam_gmapping"), "params", "slam_gmapping.yaml")],
            remappings = [("/scan","/downsampled_scan")],
    )

    return LaunchDescription([
        laser_bringup_launch,
        scan_filter_node, 
        slam_gmapping_node
    ])
