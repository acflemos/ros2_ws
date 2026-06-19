# map_gmapping_launch.py
# ======================
# Dispatcher de SLAM com gmapping: seleciona o sub-launch correto pelo tipo de LiDAR.
#
# Uso: ros2 launch yahboomcar_nav map_gmapping_launch.py
# Pré-requisito: export RPLIDAR_TYPE=4ROS  # ou a1, s2
#
# Lógica:
#   RPLIDAR_TYPE=4ROS → map_gmapping_4ros_s2_launch.py  (inclui scan_filter — 4ROS tem >1440 pts/frame)
#   RPLIDAR_TYPE=s2   → map_gmapping_4ros_s2_launch.py  (igual ao 4ROS — s2 também tem >1440 pts/frame)
#   RPLIDAR_TYPE=a1   → map_gmapping_a1_launch.py        (sem scan_filter — A1 tem ≤1440 pts/frame)
#
# gmapping só aceita frames com ≤1440 pontos — por isso o scan_filter é necessário para 4ROS e s2.
#
# Relevância para robodog2:
#   Em simulação, substituir por slam_toolbox (padrão Nav2 Humble) — gmapping está deprecado no ROS2.
#   Este launch serve apenas como referência de como o SLAM era feito no hardware real.

from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.conditions import LaunchConfigurationEquals
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    RPLIDAR_TYPE = os.getenv('RPLIDAR_TYPE')
    rplidar_type_arg = DeclareLaunchArgument(name='rplidar_type', default_value=RPLIDAR_TYPE, 
                                              choices=['a1','s2','4ROS'],
                                              description='The type of robot')

    gmapping_4ros_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('yahboomcar_nav'), 'launch'),
        '/map_gmapping_4ros_s2_launch.py']),
        condition=LaunchConfigurationEquals('rplidar_type', '4ROS')
    )
    gmapping_s2_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('yahboomcar_nav'), 'launch'),
        '/map_gmapping_4ros_s2_launch.py']),
        condition=LaunchConfigurationEquals('rplidar_type', 's2')
    )
    gmapping_a1_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('yahboomcar_nav'), 'launch'),
        '/map_gmapping_a1_launch.py']),
        condition=LaunchConfigurationEquals('rplidar_type', 'a1')
    )

    return LaunchDescription([
        rplidar_type_arg,
        gmapping_4ros_launch, 
        gmapping_s2_launch, 
        gmapping_a1_launch
    ])
