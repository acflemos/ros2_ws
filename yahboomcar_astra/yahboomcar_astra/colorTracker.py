#!/usr/bin/env python
# encoding: utf-8
# Nó de seguimento de objeto por cor com PID + profundidade Astra.
# Subscreve /Current_point (posição 2D) e image_raw (depth) para calcular
# distância real e publicar comandos de velocidade para o robô seguir o objeto.

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from yahboomcar_msgs.msg import Position

import os
import time
import math
from yahboomcar_astra.astra_common import *
from cv_bridge import CvBridge

print("import done")


class color_Tracker(Node):
    def __init__(self, name):
        super().__init__(name)
        self.pub_cmdVel = self.create_publisher(Twist, '/cmd_vel', 10)
        # image_raw: imagem de profundidade da Astra (encoding 32FC1 = float32 metros)
        self.sub_depth    = self.create_subscription(Image, "image_raw", self.depth_img_Callback, 1)
        self.sub_JoyState = self.create_subscription(Bool, '/JoyState', self.JoyStateCallback, 1)
        self.sub_position = self.create_subscription(Position, "/Current_point", self.positionCallback, 1)

        self.bridge = CvBridge()
        self.Center_x = 0
        self.Center_y = 0
        self.Center_r = 0          # raio do objeto detetado (0 = nenhum objeto)
        self.Center_prevx = 0
        self.Center_prevr = 0
        self.prev_time = 0
        self.prev_dist = 0
        self.prev_angular = 0
        self.Joy_active = False
        self.Robot_Run = False
        self.dist = []
        self.encoding = ['16UC1', '32FC1']

        # Ganhos PID — serão sobrescritos por declare_param()
        self.linear_PID  = (3.0, 0.0, 1.0)
        self.angular_PID = (0.5, 0.0, 2.0)
        self.scale = 1000
        self.PID_init()
        self.declare_param()
        print("init done")

    def declare_param(self):
        self.declare_parameter("linear_Kp",  3.0)
        self.declare_parameter("linear_Ki",  0.0)
        self.declare_parameter("linear_Kd",  1.0)
        self.declare_parameter("angular_Kp", 0.5)
        self.declare_parameter("angular_Ki", 0.0)
        self.declare_parameter("angular_Kd", 2.0)
        self.declare_parameter("scale",      1000)
        self.declare_parameter("minDistance", 1.0)  # distância alvo em metros

        self.linear_Kp  = self.get_parameter('linear_Kp').get_parameter_value().double_value
        self.linear_Ki  = self.get_parameter('linear_Ki').get_parameter_value().double_value
        self.linear_Kd  = self.get_parameter('linear_Kd').get_parameter_value().double_value
        self.angular_Kp = self.get_parameter('angular_Kp').get_parameter_value().double_value
        self.angular_Ki = self.get_parameter('angular_Ki').get_parameter_value().double_value
        self.angular_Kd = self.get_parameter('angular_Kd').get_parameter_value().double_value
        self.scale       = self.get_parameter('scale').get_parameter_value().integer_value
        self.minDistance = self.get_parameter('minDistance').get_parameter_value().double_value
        # minDist em metros (igual a minDistance — 32FC1 já está em metros)
        self.minDist = self.minDistance

    def get_param(self):
        self.linear_Kp  = self.get_parameter('linear_Kp').get_parameter_value().double_value
        self.linear_Ki  = self.get_parameter('linear_Ki').get_parameter_value().double_value
        self.linear_Kd  = self.get_parameter('linear_Kd').get_parameter_value().double_value
        self.angular_Kp = self.get_parameter('angular_Kp').get_parameter_value().double_value
        self.angular_Ki = self.get_parameter('angular_Ki').get_parameter_value().double_value
        self.angular_Kd = self.get_parameter('angular_Kd').get_parameter_value().double_value
        self.scale       = self.get_parameter('scale').get_parameter_value().integer_value
        self.minDistance = self.get_parameter('minDistance').get_parameter_value().double_value
        self.linear_PID  = (self.linear_Kp,  self.linear_Ki,  self.linear_Kd)
        self.angular_PID = (self.angular_Kp, self.angular_Ki, self.angular_Kd)
        self.minDist = self.minDistance  # metros (32FC1)

    def PID_init(self):
        # Ganhos divididos por 1000/100 para escalar a saída à gama [-1, 1] típica do Twist
        self.linear_pid  = simplePID(self.linear_PID[0]  / 1000.0,
                                     self.linear_PID[1]  / 1000.0,
                                     self.linear_PID[2]  / 1000.0)
        self.angular_pid = simplePID(self.angular_PID[0] / 100.0,
                                     self.angular_PID[1] / 100.0,
                                     self.angular_PID[2] / 100.0)

    def depth_img_Callback(self, msg):
        if not isinstance(msg, Image):
            return
        # 32FC1: profundidade em metros (float32, 1 canal)
        depthFrame = self.bridge.imgmsg_to_cv2(msg, desired_encoding=self.encoding[1])
        self.action = cv.waitKey(1)

        if self.Center_r != 0:
            # Detetar objeto parado: se posição não muda em 5s, considerar perdido
            now_time = time.time()
            if now_time - self.prev_time > 5:
                if self.Center_prevx == self.Center_x and self.Center_prevr == self.Center_r:
                    self.Center_r = 0
                self.prev_time = now_time

            # Amostrar 5 pontos ao redor do centro do objeto para medir profundidade
            distance = [0.0] * 5
            cy, cx = int(self.Center_y), int(self.Center_x)
            if 3 < cy < 477 and 3 < cx < 637:
                distance[0] = depthFrame[cy - 3][cx - 3]
                distance[1] = depthFrame[cy + 3][cx - 3]
                distance[2] = depthFrame[cy - 3][cx + 3]
                distance[3] = depthFrame[cy + 3][cx + 3]
                distance[4] = depthFrame[cy][cx]

                distance_sum = 0.0
                num_valid = 0
                for d in distance:
                    # CORRIGIDO: imagem 32FC1 está em metros — range válido 0.04m a 80m
                    # (código original comparava com 40 < d < 80000, válido para mm mas não metros)
                    if 0.04 < d < 80.0:
                        distance_sum += d
                        num_valid += 1

                # CORRIGIDO: inicializar acumulador a 0 (original: 1000.0 inflacionava distância em 1m)
                if num_valid == 0:
                    distance_ = self.minDist
                else:
                    distance_ = distance_sum / num_valid

                self.execute(self.Center_x, distance_)
                self.Center_prevx = self.Center_x
                self.Center_prevr = self.Center_r
        else:
            if self.Robot_Run:
                self.pub_cmdVel.publish(Twist())
                self.Robot_Run = False

        if self.action == ord('q') or self.action == 113:
            self.cleanup()

    def JoyStateCallback(self, msg):
        if not isinstance(msg, Bool):
            return
        self.Joy_active = msg.data
        self.pub_cmdVel.publish(Twist())  # parar ao ativar joystick

    def positionCallback(self, msg):
        if not isinstance(msg, Position):
            return
        # Position.anglex/angley aqui são pixel X/Y (reutilização semântica do tipo)
        self.Center_x = msg.anglex
        self.Center_y = msg.angley
        self.Center_r = msg.distance  # raio do círculo envolvente

    def cleanup(self):
        self.pub_cmdVel.publish(Twist())
        print("Shutting down this node.")
        cv.destroyAllWindows()

    def execute(self, point_x, dist):
        self.get_param()
        # Filtros anti-spike: ignorar saltos súbitos (>0.3m ou >300px) como leituras inválidas
        if abs(self.prev_dist - dist) > 0.3:
            self.prev_dist = dist
            return
        if abs(self.prev_angular - point_x) > 300:
            self.prev_angular = point_x
            return
        if self.Joy_active:
            return

        # PID linear: aproximar até minDist metros
        linear_x  = self.linear_pid.compute(dist, self.minDist)
        # PID angular: centrar objeto em x=320 (metade de 640px)
        angular_z = self.angular_pid.compute(320, point_x)

        # Zona morta: parar movimento se já está próximo do alvo
        if abs(dist - self.minDist) < 0.03:      # ±3cm
            linear_x = 0
        if abs(point_x - 320.0) < 30:            # ±30px
            angular_z = 0

        twist = Twist()
        twist.angular.z = max(-2.0, min(2.0, angular_z))
        twist.linear.x  = max(-1.0, min(1.0, linear_x))
        self.pub_cmdVel.publish(twist)
        self.Robot_Run = True


def main():
    rclpy.init()
    color_tracker = color_Tracker("ColorTracker")
    print("start it")
    rclpy.spin(color_tracker)
