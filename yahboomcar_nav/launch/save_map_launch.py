# save_map_launch.py
# ===================
# Salva o mapa atual (publicado em /map) para disco usando map_saver_cli (nav2_map_server).
# Deve ser executado enquanto o SLAM estiver ativo e o mapa estiver sendo publicado.
#
# Cria dois ficheiros:
#   yahboomcar.yaml — metadados do mapa (resolução, origem, caminho da imagem)
#   yahboomcar.pgm  — imagem do mapa (pixels: 0=livre, 100=ocupado, -1=desconhecido)
#
# BUG — caminho hacky para salvar no source tree:
#   O caminho de destino é calculado como:
#     <install>/share/yahboomcar_nav/../../../../src/yahboomcar_nav/maps/
#   Este truque sobe 4 níveis do install tree para chegar ao src/.
#   Só funciona se o workspace estiver montado no caminho padrão ~/ros2_ws.
#   Em outros ambientes (Docker, CI, robodog2) o caminho será errado.
#   Solução: passar map_path explicitamente:
#     ros2 launch yahboomcar_nav save_map_launch.py map_path:=/caminho/para/mapa/nome_sem_extensao
#
# Argumentos:
#   map_path — caminho base do mapa sem extensão (padrão: ver BUG acima)

from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    package_share_path = str(get_package_share_path('yahboomcar_nav'))
    # BUG: sobe 4 níveis do install tree para tentar escrever em src/ — frágil e não portável
    package_path = os.path.abspath(os.path.join(
        package_share_path, "../../../../src/yahboomcar_nav"))
    map_name = "yahboomcar"
    default_map_path = os.path.join(package_path, 'maps', map_name)

    map_arg = DeclareLaunchArgument(name='map_path', default_value=str(default_map_path),
                                    description='The path of the map')

    map_saver_node = Node(
        package='nav2_map_server',
        executable='map_saver_cli',
        arguments=[
            '-f', LaunchConfiguration('map_path'), '--ros-args', '-p', 'save_map_timeout:=10000'],
    )

    return LaunchDescription([
        map_arg,
        map_saver_node
    ])
