# keyboart_ctrl_launch-Copy1.py  (nome com typo — "keyboart" em vez de "keyboard")
# Inicia o controlo por teclado do ROSMaster X3/R2.
#
# Uso:
#   ros2 launch yahboomcar_ctrl keyboart_ctrl_launch-Copy1.py
#
# O nó yahboom_keyboard lê input raw do terminal e publica /cmd_vel.
# Requer terminal interativo — não funciona em background ou via SSH sem TTY.

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='yahboomcar_ctrl',
            executable='yahboom_keyboard',
        ),
    ])
