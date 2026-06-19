# map_cartographer_launch.py
# ===========================
# Bringup completo de SLAM com Cartographer: hardware + LiDAR + SLAM.
# Equivalente ao map_gmapping_a1_launch.py mas usando Cartographer em vez de gmapping.
#
# Cadeia de nós:
#   laser_bringup_launch → bringup do hardware (X1/X3/R2) + driver LiDAR (/scan)
#   cartographer_launch  → cartographer_node + occupancy_grid_node → /map
#
# Não inclui RViz2 — executar display_map_launch.py separadamente para visualizar.
# Não inclui scan_filter — Cartographer não tem limite de pontos por frame.
#
# Relevância para robodog2:
#   Padrão recomendado de SLAM para o robodog2 (mais robusto que gmapping).
#   Em simulação: passar use_sim_time:=true para cartographer_launch.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    package_launch_path = os.path.join(get_package_share_directory('yahboomcar_nav'), 'launch')

    laser_bringup_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [package_launch_path,'/laser_bringup_launch.py'])
    )

    cartographer_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [package_launch_path, '/cartographer_launch.py'])
    )

    return LaunchDescription([laser_bringup_launch, cartographer_launch])