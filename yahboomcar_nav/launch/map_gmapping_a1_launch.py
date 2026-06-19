# map_gmapping_a1_launch.py
# ==========================
# SLAM com gmapping para o LiDAR RPLIDAR A1 (Slamtec, ≤1440 pts/frame).
#
# Cadeia de nós:
#   laser_bringup_launch → bringup do hardware + driver do A1 (/scan)
#   slam_gmapping        → consome /scan e /tf, produz /map + TF map→odom
#
# Nota: sem scan_filter — o A1 já produz ≤1440 pontos por frame,
# dentro do limite aceite pelo gmapping.
#
# Relevância para robodog2:
#   Não reutilizar diretamente — gmapping está deprecado no ROS2 Humble.
#   Usar slam_toolbox em seu lugar. Este launch serve como referência de topologia.

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

    slam_gmapping_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('slam_gmapping'), 'launch'),
         '/slam_gmapping.launch.py'])
    )

    return LaunchDescription([laser_bringup_launch, slam_gmapping_launch])
