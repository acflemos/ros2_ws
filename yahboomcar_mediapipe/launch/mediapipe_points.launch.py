# mediapipe_points.launch.py — Launch file para visualização de landmarks no RViz2
# =================================================================================
# Lança dois nós em conjunto:
#   1. pub_point (pacote yahboomcar_point) — publica landmarks MediaPipe como
#      marcadores ROS2 para visualização 3D.
#   2. rviz2 — abre o RViz2 com configuração pré-definida para exibir os pontos.
#
# AVISO: o pacote referenciado é `yahboomcar_point` (não `yahboomcar_mediapipe`).
#   O nó pub_point deve estar instalado separadamente.
# AVISO: caminho do .rviz hardcoded para `/home/nx-ros2/yahboomcar_ws/...` —
#   não funciona fora do ambiente Yahboom original; adaptar para o workspace local.
# NOTA: as linhas de get_package_share_path e default_rviz_config_path estão
#   comentadas — a configuração alternativa portável está inativa.
#
# Para usar localmente: substituir o caminho do .rviz pelo caminho do workspace
#   atual ou descomentar as linhas de get_package_share_path.
# Relevância para robodog2: ponto de entrada para depuração visual dos landmarks
#   publicados pelos nós MediaPipe; útil durante desenvolvimento na Jetson Nano.
from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    #package_path = get_package_share_path('yahboomcar_mediapipe')
    #default_rviz_config_path = package_path / 'rviz/mediapipe_points.rviz'

    pub_point_node = Node(
        package='yahboomcar_point',
        executable='pub_point',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', '/home/nx-ros2/yahboomcar_ws/src/yahboomcar_mediapipe/rviz/mediapipe_points.rviz'],
    )

    return LaunchDescription([
        pub_point_node,
        rviz_node
    ])
