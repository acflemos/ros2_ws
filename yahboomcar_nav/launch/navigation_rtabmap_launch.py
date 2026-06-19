# navigation_rtabmap_launch.py
# =============================
# Navegação autónoma completa com localização por RTAB-Map + Nav2.
# Combina: bringup de hardware + localização RTAB-Map + Nav2.
#
# Cadeia de nós (3 sub-launches em paralelo):
#   laser_bringup_launch       → hardware + LiDAR (/scan)
#   rtabmap_localization_launch → RTAB-Map em modo localização (usa mapa existente)
#   rtabmap_nav_launch         → Nav2 bringup com rtabmap_nav_params.yaml
#
# Pré-requisito: ter executado map_rtabmap_launch.py e salvo o mapa RTAB-Map (.db).
# A localização RTAB-Map substitui o AMCL — não usa ficheiro .yaml de mapa 2D.
#
# Relevância para robodog2:
#   Relevante se o robodog2 tiver câmera de profundidade.
#   RTAB-Map localização é mais robusta que AMCL em ambientes com loop closure.

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

    rtabmap_localization_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [package_launch_path, '/rtabmap_localization_launch.py'])
    )

    rtabmap_nav_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [package_launch_path, '/rtabmap_nav_launch.py'])
    )

    return LaunchDescription([
        laser_bringup_launch, 
        rtabmap_localization_launch,
        rtabmap_nav_launch
    ])