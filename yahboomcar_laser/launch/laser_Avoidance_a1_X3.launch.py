# laser_Avoidance_a1_X3.launch.py
# ==================================
# Desvio de obstáculos completo para X3 com RPLIDAR A1.
# Inicia em paralelo: bringup de hardware X3 + driver RPLIDAR A1 + nó de desvio.
#
# Nós lançados:
#   yahboomcar_bringup_X3_launch.py → driver X3 + base_node + IMU filter + EKF
#   sllidar_ros2/sllidar_launch.py  → driver RPLIDAR A1 (publica /scan)
#   laser_Avoidance_a1_X3          → lógica de desvio (subscreve /scan, publica /cmd_vel)
#
# Nota: hardcoded para RPLIDAR A1 (sllidar_launch.py). Para usar YDLIDAR X4,
# substituir por ydlidar_ros2_driver e executar laser_Avoidance_4ROS.py manualmente.
#
# Parâmetros ajustáveis em runtime:
#   ros2 param set /laser_Avoidance_a1 Switch true
#   ros2 param set /laser_Avoidance_a1 ResponseDist 0.4

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
        executable='laser_Avoidance_a1_X3',
    )
    lidar_node = IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('sllidar_ros2'), 'launch'),
'/sllidar_launch.py'])
)
    bringup_node = IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('yahboomcar_bringup'), 'launch'),
'/yahboomcar_bringup_X3_launch.py'])
)
    
    launch_description = LaunchDescription([laser_Avoidance_node,lidar_node,bringup_node]) 
    return launch_description
