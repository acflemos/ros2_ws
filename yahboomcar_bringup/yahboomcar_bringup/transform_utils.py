#!/usr/bin/env python
# encoding: utf-8
#
# transform_utils.py — Utilitários de conversão de orientação (TF/quaternion)
# ===========================================================================
# Funções auxiliares portadas do turtlebot_node (ROS1) para converter quaternions
# em ângulos e normalizar ângulos. Usadas pelos scripts de calibração e patrulha.
#
# Dependência: PyKDL (KDL para Python)
#   Instalar: sudo apt install ros-humble-python-orocos-kdl
#
# Funções exportadas:
#   quat_to_angle(quat) → float   — extrai o yaw (rotação em torno de Z) de um quaternion
#   normalize_angle(angle) → float — normaliza ângulo para [-π, π]
#
# Nota para o robodog2:
#   Estas funções duplicam o que existe no pacote tf_transformations (ROS2).
#   Para código novo no rbd2_bringup, preferir a versão moderna:
#     from tf_transformations import euler_from_quaternion
#     (r, p, y) = euler_from_quaternion([q.x, q.y, q.z, q.w])
#   A versão PyKDL é mantida aqui por compatibilidade com o código Yahboom existente.

import PyKDL
from math import pi


def quat_to_angle(quat):
    """Converte quaternion para ângulo yaw (rotação em torno de Z) em radianos.

    Args:
        quat: objeto com atributos .x, .y, .z, .w (geometry_msgs/Quaternion)

    Returns:
        float: ângulo yaw em radianos no intervalo [-π, π]
    """
    rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
    return rot.GetRPY()[2]  # [0]=roll, [1]=pitch, [2]=yaw


def normalize_angle(angle):
    """Normaliza ângulo para o intervalo [-π, π] para evitar saltos de 2π.

    Args:
        angle: ângulo em radianos (qualquer valor)

    Returns:
        float: ângulo normalizado no intervalo [-π, π]
    """
    res = angle
    while res > pi:
        res -= 2.0 * pi
    while res < -pi:
        res += 2.0 * pi
    return res
