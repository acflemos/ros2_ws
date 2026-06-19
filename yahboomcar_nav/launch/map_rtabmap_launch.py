# map_rtabmap_launch.py
# ======================
# SLAM 3D/2D com RTAB-Map usando câmera RGB-D + LiDAR 2D.
# Requer câmera de profundidade (Intel RealSense ou similar) além do LiDAR.
#
# Cadeia de nós:
#   laser_bringup_launch → bringup do hardware + driver LiDAR (/scan)
#   rtabmap_sync_launch  → rgbd_sync + rtabmap SLAM (cria base de dados .db)
#
# Tópicos esperados:
#   /camera/color/image_raw      (RGB)
#   /camera/color/camera_info    (calibração)
#   /camera/depth/image_raw      (profundidade)
#   /scan                        (LiDAR 2D)
#   /odom                        (odometria)
#
# Relevância para robodog2:
#   Relevante se o robodog2 tiver câmera de profundidade (Astra ou RealSense).
#   RTAB-Map é mais robusto em ambientes 3D e gera mapas reutilizáveis.
#   Requer pacotes: ros-humble-rtabmap-ros, rtabmap_slam, rtabmap_sync.

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

    rtabmap_sync_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [package_launch_path, '/rtabmap_sync_launch.py'])
    )

    return LaunchDescription([laser_bringup_launch, rtabmap_sync_launch])