# colorTracker_X3.launch.py — Seguimento de objeto por cor HSV no X3 (ROS2)
#
# Lança: bringup do X3 + nó colorHSV (deteção de cor).
# O nó colorTracker (seguimento com PID) está comentado — ativar quando necessário.
#
# NOTA: o driver da câmera Astra está comentado (astra_node) — o colorHSV
# abre a câmera diretamente via VideoCapture(0), sem tópico ROS2.
#
# Uso:
#   ros2 launch yahboomcar_astra colorTracker_X3.launch.py
#
# Para ativar o seguimento (PID + profundidade), descomentar colorTracker_node.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Bringup completo do X3 (driver + IMU + EKF + joystick)
    driver_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('yahboomcar_bringup'), 'launch'),
            '/yahboomcar_bringup_X3_launch.py'])
    )

    # Deteção de cor HSV — abre câmera via VideoCapture(0)
    # Publica /Current_point (Position: pixel_x, pixel_y, raio)
    colorIdentify_node = Node(
        package='yahboomcar_astra',
        executable='colorHSV',        # corrigido: era node_executable (API deprecada)
        name='coloridentify',          # corrigido: era node_name (API deprecada)
    )

    # Seguimento com PID + profundidade (desativado por padrão)
    # Ativar descomentar abaixo e adicionar colorTracker_node ao LaunchDescription
    # colorTracker_node = Node(
    #     package='yahboomcar_astra',
    #     executable='colorTracker',
    #     name='colortracker',
    # )

    return LaunchDescription([
        driver_node,
        colorIdentify_node,
        # colorTracker_node,
    ])
