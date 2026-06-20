# yahboomcar_bringup_R2_launch.py — Bringup completo do ROSMASTER R2 (hardware real, ROS2)
#
# Finalidade: inicia todos os nós necessários para operar o robô R2 (Ackermann) físico.
#             NÃO é para simulação — usa o driver de hardware Ackman_driver_R2.
#
# Nós lançados (mesma estrutura que X3, com drivers do R2):
#   1. robot_state_publisher  — publica TF a partir do URDF (yahboomcar_R2.urdf.xacro)
#   2. joint_state_publisher  — publica joint_states
#   3. Ackman_driver_R2       — driver de hardware: lê IMU/encoders, publica /vel_raw com
#                               vy×1000 (ângulo de esterçamento em millirad), /imu/data_raw
#   4. base_node_R2           — nó de cinemática Ackermann: calcula /odom a partir de /vel_raw
#   5. imu_filter_madgwick    — fusão IMU → /imu/data com orientação
#   6. ekf (robot_localization) — fusão /odom + /imu/data → /odometry/filtered
#   7. yahboom_joy_R2         — controlo por joystick para o R2
#
# Diferença vs X3 launch:
#   - URDF: yahboomcar_R2.urdf.xacro (Ackermann com steer joints)
#   - driver: Ackman_driver_R2 (car_type=5)
#   - base_node: base_node_R2 (cinemática Ackermann)
#   - joystick: yahboom_joy_R2 (mapeamento diferente para esterçamento)
#   - Mesmo EKF launch (ekf_x1_x3_launch.py) — reutilizado
#
# Argumentos:
#   gui, model, rvizconfig, pub_odom_tf — idênticos ao X3 launch
#
# Relevância para robodog2: baixa — robodog2 usa X3 mecanum.

from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import os
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

print("---------------------robot_type = r2---------------------")
def generate_launch_description():
    urdf_tutorial_path = get_package_share_path('yahboomcar_description')
    default_model_path = urdf_tutorial_path / 'urdf/yahboomcar_R2.urdf.xacro'  # R2: xacro (não URDF direto)
    default_rviz_config_path = urdf_tutorial_path / 'rviz/yahboomcar.rviz'

    gui_arg = DeclareLaunchArgument(name='gui', default_value='false', choices=['true', 'false'],
                                    description='Flag to enable joint_state_publisher_gui')
    model_arg = DeclareLaunchArgument(name='model', default_value=str(default_model_path),
                                      description='Absolute path to robot urdf file')
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
                                     description='Absolute path to rviz config file')
    # pub_odom_tf=false → EKF publica o TF odom→base_footprint
    pub_odom_tf_arg = DeclareLaunchArgument('pub_odom_tf', default_value='false',
                                            description='Whether to publish the tf from the original odom to the base_footprint')

    # processa o URDF/xacro em tempo de execução
    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    # caminho para o YAML do filtro Madgwick — idêntico ao X3
    imu_filter_config = os.path.join(
        get_package_share_directory('yahboomcar_bringup'),
        'param',
        'imu_filter_param.yaml'
    )

    # driver R2: publica /vel_raw com vy×1000 (ângulo de esterçamento em millirad)
    driver_node = Node(
        package='yahboomcar_bringup',
        executable='Ackman_driver_R2',
    )

    # base_node R2: cinemática Ackermann (diferente do X3 mecanum)
    base_node = Node(
        package='yahboomcar_base_node',
        executable='base_node_R2',
        # 当使用ekf融合时，该tf有ekf发布
        # Quando EKF está ativo, desativar pub_odom_tf para evitar conflito de TF
        parameters=[{'pub_odom_tf': LaunchConfiguration('pub_odom_tf')}]
    )

    # imu_filter_madgwick: idêntico ao X3 — mesma cadeia IMU para ambos os modelos
    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        parameters=[imu_filter_config]
    )

    # EKF: reutiliza o mesmo launch do X3/X1 — configuração EKF é independente do tipo de robô
    ekf_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('robot_localization'), 'launch'),
            '/ekf_x1_x3_launch.py'])
    )

    # joystick R2: mapeamento específico para esterçamento Ackermann
    yahboom_joy_node = Node(
        package='yahboomcar_ctrl',
        executable='yahboom_joy_R2',
    )

    return LaunchDescription([
        gui_arg,
        model_arg,
        rviz_arg,
        pub_odom_tf_arg,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        #rviz_node,   # descomentar para abrir RViz2 no bringup
        driver_node,
        base_node,
        imu_filter_node,
        ekf_node,
        yahboom_joy_node
    ])
