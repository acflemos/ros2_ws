#!/usr/bin/env python
# coding:utf-8
# common.py
# ==========
# Utilitários partilhados por todos os nós do pacote yahboomcar_laser.
# Importado via `from yahboomcar_laser.common import *` nos outros módulos.
#
# Exporta:
#   SinglePID — controlador PID simples (sem integrador anti-windup)
#   Bool, Twist — importações relexadas para os módulos que fazem `import *`
#
# SinglePID.pid_compute(target, current) → float
#   result = Kp*error + Ki*integral + Kd*derivative
#   Acumula integral indefinidamente (sem limites) — cuidado com windup.

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class SinglePID:
    def __init__(self, P=0.1, I=0.0, D=0.1):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        print("init_pid: ", P, I, D)
        self.pid_reset()

    def Set_pid(self, P, I, D):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        print("set_pid: ", P, I, D)
        self.pid_reset()

    def pid_compute(self, target, current):
        self.error = target - current
        self.intergral += self.error
        self.derivative = self.error - self.prevError
        result = self.Kp * self.error + self.Ki * self.intergral + self.Kd * self.derivative
        self.prevError = self.error
        return result

    def pid_reset(self):
        self.error = 0
        self.intergral = 0
        self.derivative = 0
        self.prevError = 0
