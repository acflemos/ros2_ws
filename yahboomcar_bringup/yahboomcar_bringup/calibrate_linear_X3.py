#!/usr/bin/env python
# encoding: utf-8
#
# calibrate_linear_X3.py — Calibração da odometria linear do X3
# ==============================================================
# Nó de calibração que move o robô uma distância conhecida e mede o erro via TF.
# Permite ajustar o fator de escala da odometria linear do base_node_X3.
#
# Procedimento de calibração:
#   1. Iniciar o bringup X3: ros2 launch yahboomcar_bringup yahboomcar_bringup_X3_launch.py
#   2. Iniciar este nó: ros2 run yahboomcar_bringup calibrate_linear_X3
#   3. Marcar fisicamente 1 metro no chão à frente do robô
#   4. Ativar o teste:  ros2 param set /calibrate_linear start_test true
#   5. Observar onde o robô para vs o metro marcado
#   6. Ajustar e repetir:
#      ros2 param set /calibrate_linear odom_linear_scale_correction 1.05
#      (aumentar se robô fica aquém; diminuir se ultrapassa)
#
# Parâmetros ROS2 (configuráveis em runtime):
#   start_test                  (bool,  false) — inicia/para o teste
#   test_distance               (float, 1.0)   — distância alvo em metros
#   speed                       (float, 0.5)   — velocidade durante o teste (m/s)
#   tolerance                   (float, 0.03)  — tolerância de paragem (m)
#   odom_linear_scale_correction (float, 1.0)  — fator de correção (ajustar até erro → 0)
#   direction                   (bool,  true)  — true=eixo X (frente), false=eixo Y (lateral)
#   odom_frame                  (str, "odom")
#   base_frame                  (str, "base_footprint")
#
# Funcionamento:
#   - Quando start_test=false: regista posição de referência continuamente
#   - Quando start_test=true: mede distância Euclidiana via TF, aplica fator de escala,
#     move o robô na direção correta, para quando abs(distância_corrigida - alvo) < tolerância
#
# Publicadores:
#   /cmd_vel (geometry_msgs/Twist) — comandos de velocidade para calibração
#
# TF necessário: odom → base_footprint (publicado pelo base_node + EKF)
#
# Relevância para robodog2:
#   Usar antes de ativar Nav2 no hardware real para calibrar o odom_linear_scale_correction
#   do rbd2_base_node. Especialmente importante para movimento lateral (direction=false)
#   que testa a cinemática mecanum. O valor calibrado vai para o ficheiro YAML do base_node.

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from math import copysign, sqrt, pow
#import time
from rclpy.duration import Duration
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import PyKDL
from math import pi
from yahboomcar_bringup.transform_utils import *

class CalibrateLinear(Node):
    def __init__(self,name):
        super().__init__(name)
        #create a publisher
        self.cmd_vel = self.create_publisher(Twist,"/cmd_vel",5)
        #declare_parameter
        self.declare_parameter('rate',20.0)
        self.rate = self.get_parameter('rate').get_parameter_value().double_value

        self.declare_parameter('test_distance',1.0)
        self.test_distance = self.get_parameter('test_distance').get_parameter_value().double_value

        self.declare_parameter('speed',0.5)
        self.speed = self.get_parameter('speed').get_parameter_value().double_value

        self.declare_parameter('tolerance',0.03)
        self.tolerance = self.get_parameter('tolerance').get_parameter_value().double_value

        self.declare_parameter('odom_linear_scale_correction',1.0)
        self.odom_linear_scale_correction = self.get_parameter('odom_linear_scale_correction').get_parameter_value().double_value

        self.declare_parameter('start_test',False)

        # direction=true → move em X (frente/trás); false → move em Y (lateral, mecanum)
        self.declare_parameter('direction',True)
        self.direction = self.get_parameter('direction').get_parameter_value().bool_value

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

        print ("finish init work")
        now = rclpy.time.Time()
        #trans = self.tf_buffer.lookup_transform(self.odom_frame,self.base_frame,now,timeout=Duration(seconds = 10.0))
        # timer a 20Hz (intervalo 50ms) para controlo suave
        self.timer = self.create_timer(0.05, self.on_timer)


    def on_timer(self):
        move_cmd = Twist()
        #self.get_param()
        # relê parâmetros a cada tick para permitir ajuste em runtime
        self.start_test = self.get_parameter('start_test').get_parameter_value().bool_value
        self.odom_linear_scale_correction = self.get_parameter('odom_linear_scale_correction').get_parameter_value().double_value
        self.direction = self.get_parameter('direction').get_parameter_value().bool_value
        self.test_distance = self.get_parameter('test_distance').get_parameter_value().double_value
        self.tolerance = self.get_parameter('tolerance').get_parameter_value().double_value
        self.speed = self.get_parameter('speed').get_parameter_value().double_value
        if self.start_test:
            '''trans = self.tf_buffer.lookup_transform(
                        self.odom_frame,
                        self.base_frame,
                        now,
                        )'''
            #self.position.x = trans.transform.translation.x
            #self.position.y = trans.transform.translation.y
            self.position.x = self.get_position().transform.translation.x
            self.position.y = self.get_position().transform.translation.y
            print("self.position.x: ",self.position.x)
            print("self.position.y: ",self.position.y)
            # distância Euclidiana percorrida desde o início do teste
            distance = sqrt(pow((self.position.x - self.x_start), 2) +
                                pow((self.position.y - self.y_start), 2))
            # aplica fator de correção da odometria
            distance *= self.odom_linear_scale_correction
            print("distance: ",distance)
            error = distance - self.test_distance
            print("error: ",error)
            #start = time()
            if not self.start_test or abs(error) < self.tolerance:
                # para o robô e desativa o teste
                self.start_test  = rclpy.parameter.Parameter('start_test',rclpy.Parameter.Type.BOOL,False)
                all_new_parameters = [self.start_test]
                self.set_parameters(all_new_parameters)

                print("done")
            else:
                if self.direction:
                    print("x")
                    # copysign: move na direção correta (frente se ainda não chegou)
                    move_cmd.linear.x = copysign(self.speed, -1 * error)
                else:
                    # movement lateral mecanum (Y) para calibrar cinemática holonômica
                    move_cmd.linear.y = copysign(self.speed, -1 * error)
                    print("y")
            self.cmd_vel.publish(move_cmd)
            #end = time()
        else:
            # modo de espera: regista posição inicial continuamente até start_test=true
            #self.position.x = trans.transform.translation.x
            #self.position.y = trans.transform.translation.y
            self.x_start = self.get_position().transform.translation.x
            self.y_start = self.get_position().transform.translation.y
            print("self.x_start: ",self.x_start)
            print("self.y_start:",self.y_start)
            self.cmd_vel.publish(Twist())
        #self.cmd_vel.publish(Twist() )

    def get_param(self):
        #self.start_test = self.get_parameter('start_test').get_parameter_value().bool_value
        self.rate = self.get_parameter('rate').get_parameter_value().double_value
        self.test_distance = self.get_parameter('test_distance').get_parameter_value().double_value
        self.direction = self.get_parameter('direction').get_parameter_value().bool_value
        self.tolerance = self.get_parameter('tolerance').get_parameter_value().double_value
        self.speed = self.get_parameter('speed').get_parameter_value().double_value


    def get_position(self):
        """Lê a posição XY atual do robô a partir do TF odom→base_footprint."""
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(self.odom_frame,self.base_frame,now)
            return trans
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().info('transform not ready')
            raise
            return

def main():
    rclpy.init()
    class_calibratelinear = CalibrateLinear("calibrate_linear")
    rclpy.spin(class_calibratelinear)
