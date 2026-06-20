#!/usr/bin/env python
# encoding: utf-8
#
# calibrate_angular_X3.py — Calibração da odometria angular do X3
# ================================================================
# Nó de calibração que roda o robô um ângulo conhecido e mede o erro via TF.
# Permite ajustar o fator de escala da odometria angular do base_node_X3.
#
# Procedimento de calibração:
#   1. Iniciar o bringup X3: ros2 launch yahboomcar_bringup yahboomcar_bringup_X3_launch.py
#   2. Iniciar este nó: ros2 run yahboomcar_bringup calibrate_angular_X3
#   3. Marcar a orientação inicial do robô (ex: fita no chão)
#   4. Ativar o teste:  ros2 param set /calibrate_angular start_test true
#   5. Observar se o robô volta à orientação inicial após 360°
#   6. Ajustar e repetir:
#      ros2 param set /calibrate_angular odom_angular_scale_correction 0.80
#      (padrão: 0.75 — aumentar se robô fica aquém de 360°; diminuir se ultrapassa)
#
# Parâmetros ROS2 (configuráveis em runtime):
#   start_test                   (bool,  false) — inicia/para o teste
#   test_angle                   (float, 360.0) — ângulo alvo em graus
#   speed                        (float, 2.0)   — velocidade angular durante o teste (rad/s)
#   tolerance                    (float, 1.5)   — tolerância de paragem (rad)
#   odom_angular_scale_correction (float, 0.75) — fator de correção angular
#   odom_frame                   (str, "odom")
#   base_frame                   (str, "base_footprint")
#
# Funcionamento:
#   - Acumula rotação incremental a cada tick via TF (delta de yaw)
#   - Aplica odom_angular_scale_correction ao delta acumulado
#   - Para quando abs(ângulo_corrigido - ângulo_alvo) < tolerância
#   - Inverte direção (self.reverse) a cada teste para compensar drift sistemático
#
# Diferença vs calibrate_angular_R2.py:
#   - X3: odom_angular_scale_correction padrão = 0.75 (cinemática mecanum)
#   - R2: odom_angular_scale_correction padrão = 0.65 (Ackermann)
#   - X3: sem movimento linear durante rotação (gira no lugar)
#   - R2: usa move_cmd.linear.x = 0.2 durante rotação (não holonômico)
#
# Publicadores:
#   /cmd_vel (geometry_msgs/Twist) — comandos de velocidade para calibração
#
# TF necessário: odom → base_footprint
#
# Relevância para robodog2:
#   Usar para calibrar o rbd2_base_node antes de ativar o SLAM/Nav2 no hardware real.
#   Valor calibrado de odom_angular_scale_correction vai para o YAML do base_node.

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point,Quaternion
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from math import copysign, sqrt, pow,radians
import time
from rclpy.duration import Duration
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

import PyKDL
from math import pi

class Calibrateangular(Node):
    def __init__(self,name):
        super().__init__(name)
        #create a publisher
        self.cmd_vel = self.create_publisher(Twist,"/cmd_vel",5)
        #declare_parameter
        self.declare_parameter('rate',20.0)
        self.rate = self.get_parameter('rate').get_parameter_value().double_value

        self.declare_parameter('test_angle',360.0)
        self.test_angle = self.get_parameter('test_angle').get_parameter_value().double_value
        self.test_angle = radians(self.test_angle)  # converte graus → rad

        self.declare_parameter('speed',2.0)
        self.speed = self.get_parameter('speed').get_parameter_value().double_value

        # tolerância em rad — 1.5 rad é largo (~86°); pode reduzir para maior precisão
        self.declare_parameter('tolerance',1.5)
        self.tolerance = self.get_parameter('tolerance').get_parameter_value().double_value

        # fator de escala angular — padrão 0.75 para X3 (R2 usa 0.65)
        self.declare_parameter('odom_angular_scale_correction',0.75)
        self.odom_angular_scale_correction = self.get_parameter('odom_angular_scale_correction').get_parameter_value().double_value

        self.declare_parameter('start_test',False)

        # direction: +1.0 = anti-horário; -1.0 = horário
        self.declare_parameter('direction',1.0)
        self.direction = self.get_parameter('direction').get_parameter_value().double_value

        self.declare_parameter('base_frame','base_footprint')
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value

        self.declare_parameter('odom_frame','odom')
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value

        #init the tf listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        #self.tf_listener.waitForTransform(self.odom_frame, self.base_frame, rclpy.time.Time(), timeout = Duration(seconds = 60.0))
        #time.sleep(2)
        self.position = Point()
        self.x_start = self.position.x
        self.y_start = self.position.y
        self.first_angle = 0

        print ("finish init work")
        now = rclpy.time.Time()
        #trans = self.tf_buffer.lookup_transform(self.odom_frame,self.base_frame,now,timeout=Duration(seconds = 10.0))
        # self.reverse alterna +1/-1 a cada teste para compensar drift sistemático
        self.reverse = 1
        self.turn_angle = 0   # acumulador de rotação total
        self.delta_angle  = 0
        # timer a 100Hz para medição precisa do delta angular
        self.timer = self.create_timer(0.01, self.on_timer)
        self.odom_angle = self.get_odom_angle()
        self.last_angle = self.odom_angle
        self.test_angle *= self.reverse
        self.error = 0

    def on_timer(self):
        # relê parâmetros para permitir ajuste em runtime
        self.start_test = self.get_parameter('start_test').get_parameter_value().bool_value
        self.odom_angular_scale_correction = self.get_parameter('odom_angular_scale_correction').get_parameter_value().double_value
        self.test_angle = self.get_parameter('test_angle').get_parameter_value().double_value
        self.test_angle = radians(self.test_angle) #角度转成弧度 — converte graus para radianos
        self.speed = self.get_parameter('speed').get_parameter_value().double_value
        move_cmd = Twist()
        self.test_angle *= self.reverse  # aplica direção (horário/anti-horário)
        self.error = self.test_angle - self.turn_angle
        if self.start_test:
            self.error = self.test_angle - self.turn_angle
            if self.start_test and (abs(self.error) > self.tolerance or self.error==0) :
                #move_cmd.linear.x = 0.2   # X3: sem linear (gira no lugar) vs R2 (usa 0.2)
                move_cmd.angular.z = copysign(self.speed, self.error)
                #print("angular: ",move_cmd.angular.z)
                self.cmd_vel.publish(move_cmd)
                self.odom_angle = self.get_odom_angle()
                # delta incremental com fator de correção aplicado
                self.delta_angle = self.odom_angular_scale_correction * self.normalize_angle(self.odom_angle - self.first_angle)
                #print("delta_angle: ",self.delta_angle)
                self.turn_angle += self.delta_angle
                print("turn_angle: ",self.turn_angle)
                self.error = self.test_angle - self.turn_angle
                print("error: ",self.error)
                self.first_angle = self.odom_angle

                #print("first_angle: ",self.first_angle)
            else:
                # ângulo atingido: para o robô e inverte direção para próximo teste
                #self.error = 0.0
                self.turn_angle = 0.0
                self.cmd_vel.publish(Twist())
                self.start_test  = rclpy.parameter.Parameter('start_test',rclpy.Parameter.Type.BOOL,False)
                all_new_parameters = [self.start_test]
                self.set_parameters(all_new_parameters)
                self.reverse = -self.reverse  # alterna direção a cada teste
                self.first_angle = 0

        else:
            #self.error = 0.0
            self.cmd_vel.publish(Twist())
            self.start_test  = rclpy.parameter.Parameter('start_test',rclpy.Parameter.Type.BOOL,False)
            all_new_parameters = [self.start_test]
            self.set_parameters(all_new_parameters)


    def get_odom_angle(self):
        """Lê o ângulo yaw atual do robô a partir do TF odom→base_footprint."""
        try:
            now = rclpy.time.Time()
            rot = self.tf_buffer.lookup_transform(self.odom_frame,self.base_frame,now)
            #print("oring_rot: ",rot.transform.rotation)
            # PyKDL extrai yaw do quaternion
            cacl_rot = PyKDL.Rotation.Quaternion(rot.transform.rotation.x, rot.transform.rotation.y, rot.transform.rotation.z, rot.transform.rotation.w)
            #print("cacl_rot: ",cacl_rot)
            angle_rot = cacl_rot.GetRPY()[2]  # [2] = yaw
            #print("angle_rot: ",angle_rot)




        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().info('transform not ready')
            return

        return angle_rot

    def normalize_angle(self,angle):
        """Normaliza ângulo para o intervalo [-π, π] para evitar saltos de 2π."""
        res = angle
        #print("res: ",res)
        while res > pi:
            res -= 2.0 * pi
        while res < -pi:
            res += 2.0 * pi
        return res



def main():
    rclpy.init()
    class_calibrateangular = Calibrateangular("calibrate_angular")
    rclpy.spin(class_calibrateangular)
