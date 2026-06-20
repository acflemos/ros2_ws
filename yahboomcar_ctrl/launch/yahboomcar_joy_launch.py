# yahboomcar_joy_launch.py
# Inicia apenas o nó joy_node (driver do joystick físico → tópico /joy).
#
# ATENÇÃO: este launch file está INCOMPLETO — não lança yahboom_joy_X3 nem yahboom_joy_R2.
# Para controlo completo por joystick, é preciso lançar manualmente em separado:
#   ros2 run yahboomcar_ctrl yahboom_joy_X3
#
# Para uso no robodog2, criar um launch file completo que inicie ambos:
#   1. joy_node         → lê o joystick físico → publica /joy
#   2. yahboom_joy_X3   → converte /joy → /cmd_vel + /JoyState + /RGBLight + /Buzzer

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # joy_node: driver genérico ROS2 para joystick USB/Bluetooth
    # Publica sensor_msgs/Joy em /joy a cada evento de botão/eixo
    node1 = Node(
        package='joy',
        executable='joy_node',
    )
    return LaunchDescription([node1])
