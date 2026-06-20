# yahboomcar_bringup_X1_launch.py — Bringup completo do ROSMASTER X1 (hardware real, ROS2)
#
# Finalidade: inicia todos os nós necessários para operar o robô X1 físico.
#             Estrutura idêntica ao X3 launch com drivers do X1.
#
# Nós lançados:
#   1. robot_state_publisher  — publica TF a partir do URDF (yahboomcar_X1.urdf)
#   2. joint_state_publisher  — publica joint_states
#   3. Mcnamu_driver_x1       — driver de hardware X1 (car_type=4)
#                               NOTA: publica /vel_raw com vy×1000 (como R2, não como X3)
#   4. base_node_x1           — nó de cinemática X1 (parâmetros mecanum diferentes do X3)
#   5. imu_filter_madgwick    — fusão IMU → /imu/data com orientação
#   6. ekf (robot_localization) — fusão /odom + /imu/data
#   7. yahboom_joy_X3         — NOTA: usa o joystick do X3 (não há yahboom_joy_X1)
#
# Diferenças vs X3 launch:
#   - URDF: yahboomcar_X1.urdf (pacote yahboomcar_description_x1)
#   - driver: Mcnamu_driver_x1 (car_type=4)
#   - base_node: base_node_x1 (constantes de encoder diferentes)
#   - Joystick: reutiliza yahboom_joy_X3 (sem versão X1 específica)
#
# Relevância para robodog2: baixa — robodog2 usa X3.

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

print("---------------------robot_type = x1---------------------")
def generate_launch_description():
    # X1 tem pacote de descrição separado (yahboomcar_description_x1)
    urdf_tutorial_path = get_package_share_path('yahboomcar_description_x1')
    default_model_path = urdf_tutorial_path / 'urdf/yahboomcar_X1.urdf'
    default_rviz_config_path = urdf_tutorial_path / 'rviz/yahboomcar.rviz'

    gui_arg = DeclareLaunchArgument(name='gui', default_value='false', choices=['true', 'false'],
                                    description='Flag to enable joint_state_publisher_gui')
    model_arg = DeclareLaunchArgument(name='model', default_value=str(default_model_path),
                                      description='Absolute path to robot urdf file')
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
                                     description='Absolute path to rviz config file')
    pub_odom_tf_arg = DeclareLaunchArgument('pub_odom_tf', default_value='false',
                                            description='Whether to publish the tf from the original odom to the base_footprint')

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

    # 找到参数文件的完整路径 — localiza o ficheiro de parâmetros do filtro IMU
    imu_filter_config = os.path.join(
        get_package_share_directory('yahboomcar_bringup'),
        'param',
        'imu_filter_param.yaml'
    )

    # driver X1: car_type=4, /vel_raw.linear.y = vy×1000 (escala millirad, como R2)
    driver_node = Node(
        package='yahboomcar_bringup',
        executable='Mcnamu_driver_x1',
    )

    # base_node X1: constantes de encoder diferentes do X3 (rodas menores)
    base_node = Node(
        package='yahboomcar_base_node',
        executable='base_node_x1',
        # 当使用ekf融合时，该tf有ekf发布
        # Quando EKF está ativo, pub_odom_tf=false para evitar conflito
        parameters=[{'pub_odom_tf': LaunchConfiguration('pub_odom_tf')}]
    )

    # imu_filter_madgwick: idêntico ao X3/R2
    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        parameters=[imu_filter_config]
    )

    # EKF: reutiliza o mesmo launch do X3 — configuração não depende do tipo de robô
    ekf_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('robot_localization'), 'launch'),
            '/ekf_x1_x3_launch.py'])
    )

    # NOTA: X1 reutiliza o joystick do X3 — sem versão específica para X1
    yahboom_joy_node = Node(
        package='yahboomcar_ctrl',
        executable='yahboom_joy_X3',
    )

    return LaunchDescription([
        gui_arg,
        model_arg,
        rviz_arg,
        pub_odom_tf_arg,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        # rviz_node   # descomentar para abrir RViz2 no bringup
        driver_node,
        base_node,
        imu_filter_node,
        ekf_node,
        yahboom_joy_node
    ])
