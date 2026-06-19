# navigation_teb_launch.py
# =========================
# Navegação autónoma Nav2 com planeador local TEB (Timed Elastic Band).
# Idêntico ao navigation_dwa_launch.py, mas usa teb_nav_params.yaml que
# configura teb_local_planner::TebLocalPlannerROS em vez de DWB.
#
# TEB vs DWB para o X3:
#   TEB lida melhor com obstáculos dinâmicos e trajetórias curvas.
#   DWB é mais simples e rápido, mais próximo do comportamento original.
#   Nenhum dos dois está configurado para holonómica — ver bugs abaixo.
#
# ATENÇÃO — BUG crítico em teb_nav_params.yaml (mesmos do DWB):
#   robot_model_type: "differential" no AMCL — deve ser "omni" para mecanum.
#   TEB usa footprint_model.type: circular (ok) mas sem configuração de
#   velocidade lateral — potencial holonómico do X3 não é aproveitado.
#
# BUG adicional em teb_nav_params.yaml:
#   'ontroller_frequency: 20.0' — typo: falta a letra 'c' no início da chave.
#   O parâmetro é ignorado silenciosamente pelo Nav2.
#
# Relevância para robodog2:
#   Corrigir os bugs de AMCL e velocidade Y antes de usar com robodog2.
#   TEB requer instalação: sudo apt install ros-humble-teb-local-planner

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_path = get_package_share_directory('yahboomcar_nav')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_yaml_path = LaunchConfiguration(
        'map', default=os.path.join(package_path, 'maps', 'yahboomcar.yaml'))
    nav2_param_path = LaunchConfiguration('params_file', default=os.path.join(
        package_path, 'params', 'teb_nav_params.yaml'))

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value=use_sim_time,
                              description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('map', default_value=map_yaml_path,
                              description='Full path to map file to load'),
        DeclareLaunchArgument('params_file', default_value=nav2_param_path,
                              description='Full path to param file to load'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_bringup_dir, '/launch', '/bringup_launch.py']),
            launch_arguments={
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path}.items(),
        ),
    ])
