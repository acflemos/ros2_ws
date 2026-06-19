# cartographer_launch.py
# =======================
# SLAM 2D com Google Cartographer.
# Não inclui bringup de hardware — usar após map_cartographer_launch.py,
# ou executar laser_bringup_launch.py separadamente antes.
#
# Argumentos:
#   configuration_directory — pasta com o ficheiro Lua (padrão: params/)
#   configuration_basename  — ficheiro de configuração Lua (padrão: lds_2d.lua)
#   use_sim_time            — false por padrão; true para Gazebo
#   resolution              — resolução do OccupancyGrid em metros (padrão: 0.05)
#   publish_period_sec      — período de publicação do mapa (padrão: 1.0s)
#
# Nós lançados:
#   cartographer_node    — SLAM principal; subscreve /scan e /odom via TF
#   occupancy_grid_node  — converte a representação interna do Cartographer em /map
#
# Configuração em params/lds_2d.lua:
#   use_imu_data = false, use_odometry = true, max_range = 8m, 2D mode.
#
# Relevância para robodog2:
#   Cartographer é uma alternativa de alta qualidade ao slam_toolbox.
#   Pode ser reutilizado diretamente em simulação passando use_sim_time:=true.
#   Instalar: sudo apt install ros-humble-cartographer-ros

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    package_path = get_package_share_directory('yahboomcar_nav')
    configuration_directory = LaunchConfiguration('configuration_directory', default=os.path.join(
                                                  package_path, 'params'))
    configuration_basename = LaunchConfiguration('configuration_basename', default='lds_2d.lua')

    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration(
        'publish_period_sec', default='1.0')

    return LaunchDescription([
        DeclareLaunchArgument(
            'configuration_directory',
            default_value=configuration_directory,
            description='Full path to config file to load'),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value=configuration_basename,
            description='Name of lua file for cartographer'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-configuration_directory', configuration_directory,
                       '-configuration_basename', configuration_basename]),

        DeclareLaunchArgument(
            'resolution',
            default_value=resolution,
            description='Resolution of a grid cell in the published occupancy grid'),

        DeclareLaunchArgument(
            'publish_period_sec',
            default_value=publish_period_sec,
            description='OccupancyGrid publishing period'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/occupancy_grid_launch.py']),
            launch_arguments={'use_sim_time': use_sim_time, 'resolution': resolution,
                              'publish_period_sec': publish_period_sec}.items(),
        ),
    ])
