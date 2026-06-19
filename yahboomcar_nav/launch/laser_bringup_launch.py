# laser_bringup_launch.py
# ========================
# Launch unificado: bringup de hardware + driver do LiDAR, selecionados por variáveis de ambiente.
# Uso: ros2 launch yahboomcar_nav laser_bringup_launch.py
#
# Pré-requisito: definir variáveis de ambiente antes de lançar:
#   export ROBOT_TYPE=x3       # ou x1, r2
#   export RPLIDAR_TYPE=4ROS   # ou a1, s2
#
# O que este launch faz:
#   1. Lê ROBOT_TYPE  → inicia o bringup correspondente (X1, X3 ou R2)
#   2. Lê RPLIDAR_TYPE → inicia o driver do LiDAR correspondente
#   3. Publica TF estático base_link → laser (posição física do LiDAR no X3)
#
# LiDARs suportados:
#   a1   → RPLIDAR A1  (Slamtec, pacote sllidar_ros2, /sllidar_launch.py)
#   s2   → RPLIDAR S2  (Slamtec, pacote sllidar_ros2, /sllidar_s2_launch.py)
#   4ROS → YDLIDAR X4  (YDLIDAR, pacote ydlidar_ros2_driver, /ydlidar_raw_launch.py)
#
# TF estático publicado:
#   base_link → laser  (x=0.0435m, y=5.258E-05m, z=0.11m, yaw=π)
#   Nota: frame "laser" (não "laser_link") — verificar consistência com o URDF.
#   O URDF do X3 usa "laser_link" mas o TF aqui publica "laser" — possível inconsistência.
#
# Relevância para robodog2:
#   Em simulação, o driver do LiDAR e o bringup são substituídos pelo Gazebo.
#   Este launch é a referência para entender o TF do LiDAR (posição e rotação).
#   O rpy="3.14 0 0" (π em roll) indica que o LiDAR está montado "de cabeça para baixo"
#   ou que as leituras são espelhadas — verificar com o hardware real.

from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Lê tipo de robô e LiDAR das variáveis de ambiente
    ROBOT_TYPE = os.getenv('ROBOT_TYPE')
    RPLIDAR_TYPE = os.getenv('RPLIDAR_TYPE')
    print("\n-------- robot_type = {}, rplidar_type = {} --------\n".format(ROBOT_TYPE, RPLIDAR_TYPE))

    robot_type_arg = DeclareLaunchArgument(
        name='robot_type', default_value=ROBOT_TYPE,
        choices=['x1', 'x3', 'r2'],
        description='The type of robot'
    )
    rplidar_type_arg = DeclareLaunchArgument(
        name='rplidar_type', default_value=RPLIDAR_TYPE,
        choices=['a1', 's2', '4ROS'],
        description='The type of lidar'
    )

    # Bringup condicionado ao tipo de robô (apenas um é iniciado)
    bringup_x1_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('yahboomcar_bringup'), 'launch'),
            '/yahboomcar_bringup_X1_launch.py'
        ]),
        condition=LaunchConfigurationEquals('robot_type', 'x1')
    )
    bringup_x3_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('yahboomcar_bringup'), 'launch'),
            '/yahboomcar_bringup_X3_launch.py'
        ]),
        condition=LaunchConfigurationEquals('robot_type', 'x3')
    )
    bringup_r2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('yahboomcar_bringup'), 'launch'),
            '/yahboomcar_bringup_R2_launch.py'
        ]),
        condition=LaunchConfigurationEquals('robot_type', 'r2')
    )

    # Driver do LiDAR condicionado ao tipo (apenas um é iniciado)
    lidar_a1_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('sllidar_ros2'), 'launch'),
            '/sllidar_launch.py'
        ]),
        condition=LaunchConfigurationEquals('rplidar_type', 'a1')
    )
    lidar_s2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('sllidar_ros2'), 'launch'),
            '/sllidar_s2_launch.py'
        ]),
        condition=LaunchConfigurationEquals('rplidar_type', 's2')
    )
    lidar_4ROS_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ydlidar_ros2_driver'), 'launch'),
            '/ydlidar_raw_launch.py'
        ]),
        condition=LaunchConfigurationEquals('rplidar_type', '4ROS')
    )

    # TF estático: posição física do LiDAR no chassis X3
    # x=4.35cm frente, z=11cm acima, yaw=π (rotação 180°)
    # CORRIGIDO: frame destino alterado de "laser" para "laser_link" (consistente com URDF do X3)
    tf_base_link_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.0435', '5.258E-05', '0.11', '3.14', '0', '0', 'base_link', 'laser_link']
    )

    return LaunchDescription([
        robot_type_arg,
        bringup_x1_launch,
        bringup_x3_launch,
        bringup_r2_launch,
        rplidar_type_arg,
        lidar_a1_launch,
        lidar_s2_launch,
        lidar_4ROS_launch,
        tf_base_link_to_laser,
    ])
