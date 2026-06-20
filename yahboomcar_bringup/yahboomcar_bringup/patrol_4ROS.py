#!/usr/bin/env python
# encoding: utf-8
#
# patrol_4ROS.py — Patrulha autónoma simples para o X3 (sem JoyState)
# ====================================================================
# Versão simplificada do patrol_a1_X3.py — sem subscrição ao /JoyState.
# O joystick não interrompe a patrulha nesta versão.
#
# Suporta: Square, Circle, LengthTest (sem Triangle).
#
# Diferenças vs patrol_a1_X3.py:
#   - Sem subscrição a /JoyState → joystick não pausa a patrulha
#   - Square: vira 180° (ida e volta), não 90° (quadrado real do a1_X3)
#   - LaserAngle monitoriza ângulos < LaserAngle° (frente absoluta)
#     vs a1_X3 que monitoriza >(180-LaserAngle)° (frente relativa ao X3)
#   - Velocidade linear padrão: 0.2 m/s (vs 0.5 m/s no a1_X3)
#   - Parâmetros Linear/Angular/RotationTolerance/RotationScaling NÃO são
#     configuráveis em runtime (valores fixos em código)
#   - Spin com Square: velocidade angular fixa = 2 rad/s (não usa parâmetro Angular)
#
# Parâmetros ROS2 (subconjunto do patrol_a1_X3):
#   Switch, Command, Set_loop, ResponseDist, LaserAngle, odom_frame, base_frame, circle_adjust
#
# Relevância para robodog2: baixa — preferir patrol_a1_X3 ou Nav2.
# Esta versão é mais simples mas menos configurável.

#for patrol
#math
import math
from math import radians, copysign, sqrt, pow
from math import pi
import numpy as np
#rclpy
import rclpy
from rclpy.node import Node
#tf
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
#msg
from geometry_msgs.msg import Twist, Point,Quaternion
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
#others
import PyKDL
from time import sleep

print("import finish")

class YahboomCarPatrol(Node):
    def __init__(self,name):
        super().__init__(name)
        self.moving = True
        self.Joy_active = False  # mantido por compatibilidade, mas nunca atualizado (sem /JoyState)
        self.command_src = "finish"
        self.warning = 1
        self.SetLoop = False
        self.Linear = 0.2   # mais lento que patrol_a1_X3 (0.5)
        self.Angular = 1.0
        self.Length = 1.0 #1.0
        self.Angle = 360.0
        self.LineScaling = 1.1
        self.RotationScaling = 0.75  # valores fixos — não configuráveis em runtime
        self.LineTolerance = 0.1
        self.RotationTolerance = 0.3
        #self.ResponseDist = 0.6
        #self.LaserAngle = 20
        #self.Command = "LengthTest"
        #self.Switch = False
        self.position = Point()
        self.x_start = self.position.x
        self.y_start = self.position.y
        self.error = 0.0
        self.distance = 0.0
        self.last_angle = 0.0
        self.delta_angle = 0.0
        self.turn_angle = 0.0
        self.warning = 1
        #create publisher
        self.pub_cmdVel = self.create_publisher(Twist,"/cmd_vel",5)
        #create subscriber
        # NOTA: sem subscrição ao /JoyState — esta é a diferença principal vs patrol_a1_X3
        self.sub_scan = self.create_subscription(LaserScan,"/scan",self.LaserScanCallback,1)
        #create TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer,self)
        #declare param
        self.declare_parameter('odom_frame',"odom")
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.declare_parameter('base_frame',"base_footprint")
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.declare_parameter("circle_adjust",2.0)
        self.circle_adjust = self.get_parameter("circle_adjust").get_parameter_value().double_value
        self.declare_parameter('Switch',False)
        self.Switch = self.get_parameter('Switch').get_parameter_value().bool_value
        self.declare_parameter('Command',"Square")
        self.Command = self.get_parameter('Command').get_parameter_value().string_value
        self.declare_parameter('Set_loop',False)
        self.Set_loop = self.get_parameter('Set_loop').get_parameter_value().bool_value
        self.declare_parameter('ResponseDist',0.6)
        self.ResponseDist = self.get_parameter('ResponseDist').get_parameter_value().double_value
        self.declare_parameter('LaserAngle',20.0)
        self.LaserAngle = self.get_parameter('LaserAngle').get_parameter_value().double_value
        # NOTA: Linear, Angular, RotationTolerance, RotationScaling NÃO são declarados como
        # parâmetros — os valores acima são fixos (não alteráveis via ros2 param set)

        # timer a 100Hz para baixa latência de controlo
        self.timer = self.create_timer(0.01,self.on_timer)
        self.index = 0


    def on_timer(self):
        #print("self.error: ",self.error)
        # relê apenas os parâmetros que foram declarados
        self.Switch = self.get_parameter('Switch').get_parameter_value().bool_value
        self.Command = self.get_parameter('Command').get_parameter_value().string_value
        self.Set_loop = self.get_parameter('Set_loop').get_parameter_value().bool_value
        self.ResponseDist = self.get_parameter('ResponseDist').get_parameter_value().double_value
        self.LaserAngle = self.get_parameter('LaserAngle').get_parameter_value().double_value
        index = 0

        if self.Switch==True:
            index = 0
            print("Switch True")
            if self.Command == "LengthTest":
                self.command_src = "LengthTest"
                print("LengthTest")
                advancing = self.advancing(self.Length)
                if advancing ==True:
                    self.Switch  = rclpy.parameter.Parameter('Switch',rclpy.Parameter.Type.BOOL,False)
                    all_new_parameters = [self.Switch]
                    self.set_parameters(all_new_parameters)
                    self.Command  = rclpy.parameter.Parameter('Command',rclpy.Parameter.Type.STRING,"finish")
                    all_new_parameters = [self.Command]
                    self.set_parameters(all_new_parameters)

            elif self.Command == "Circle":
                self.command_src = "Circle"
                spin = self.Spin(360)
                if spin == True:
                    print("spin done")
                    #self.Command = "finish"
                    self.Switch  = rclpy.parameter.Parameter('Switch',rclpy.Parameter.Type.BOOL,False)
                    all_new_parameters = [self.Switch]
                    self.set_parameters(all_new_parameters)
                    self.Command  = rclpy.parameter.Parameter('Command',rclpy.Parameter.Type.STRING,"finish")
                    all_new_parameters = [self.Command]
                    self.set_parameters(all_new_parameters)

            elif self.Command == "Square":
                self.command_src = "Square"
                square = self.Square()
                if square == True:
                    self.Command  = rclpy.parameter.Parameter('Command',rclpy.Parameter.Type.STRING,"finish")
                    all_new_parameters = [self.Command]
                    self.set_parameters(all_new_parameters)
        else:
            print("Switch False")
            if self.Command == "finish":
                print("finish")
                if self.Set_loop == True:
                    print("Continute")
                    self.Command  = rclpy.parameter.Parameter('Command',rclpy.Parameter.Type.STRING,self.command_src)
                    all_new_parameters = [self.Command]
                    self.set_parameters(all_new_parameters)
                    self.Switch  = rclpy.parameter.Parameter('Switch',rclpy.Parameter.Type.BOOL,True)
                    all_new_parameters = [self.Switch]
                    self.set_parameters(all_new_parameters)
                else:
                    print("Not loop")
                    self.Switch  = rclpy.parameter.Parameter('Switch',rclpy.Parameter.Type.BOOL,False)
                    all_new_parameters = [self.Switch]
                    self.set_parameters(all_new_parameters)



    def advancing(self,target_distance):
        """Avança em linha reta até target_distance (m), usando TF para medir distância."""
        self.position.x = self.get_position().transform.translation.x
        self.position.y = self.get_position().transform.translation.y
        move_cmd = Twist()
        self.distance = sqrt(pow((self.position.x - self.x_start), 2) +
                            pow((self.position.y - self.y_start), 2))
        self.distance *= self.LineScaling
        print("distance: ",self.distance)
        self.error = self.distance - target_distance
        move_cmd.linear.x = self.Linear
        if abs(self.error) < self.LineTolerance :
            print("stop")
            self.distance = 0.0
            self.pub_cmdVel.publish(Twist())
            self.x_start = self.position.x;
            self.y_start = self.position.y;
            self.Switch  = rclpy.parameter.Parameter('Switch',rclpy.Parameter.Type.BOOL,False)
            all_new_parameters = [self.Switch]
            self.set_parameters(all_new_parameters)
            return True
        else:
            if self.Joy_active or self.warning > 10:
                if self.moving == True:
                    self.pub_cmdVel.publish(Twist())
                    self.moving = False
                    print("obstacles")
            else:
                #print("Go")
                self.pub_cmdVel.publish(move_cmd)
            self.moving = True
            return False


    def Spin(self,angle):
        """Vira o robô pelo ângulo especificado (graus). Velocidade angular fixa = 2 rad/s."""
        self.target_angle = radians(angle)
        self.odom_angle = self.get_odom_angle()
        self.delta_angle = self.RotationScaling * self.normalize_angle(self.odom_angle - self.last_angle)
        self.turn_angle += self.delta_angle
        print("turn_angle: ",self.turn_angle)
        self.error = self.target_angle - self.turn_angle
        print("error: ",self.error)
        self.last_angle = self.odom_angle
        move_cmd = Twist()
        if abs(self.error) < self.RotationTolerance or self.Switch==False :
            self.pub_cmdVel.publish(Twist())
            self.turn_angle = 0.0
            '''self.Switch  = rclpy.parameter.Parameter('Switch',rclpy.Parameter.Type.BOOL,False)
            all_new_parameters = [self.Switch]
            self.set_parameters(all_new_parameters)'''
            return True
        if self.Joy_active or self.warning > 10:
            if self.moving == True:
                self.pub_cmdVel.publish(Twist())
                self.moving = False
                print("obstacles")
        else:
            if self.Command == "Square":
                # velocidade angular fixa = 2 rad/s (não usa parâmetro Angular)
                move_cmd.linear.x = 0.2
                move_cmd.angular.z = copysign(2, self.error)
            elif self.Command == "Circle":
                #length = self.Linear * self.circle_adjust / self.Length
                #print("length: ",length)
                move_cmd.linear.x = self.Linear
                move_cmd.angular.z = copysign(2, self.error)  # circular com angular fixo = 2
                #print("angular: ",move_cmd.angular.z)
                '''move_cmd.linear.x = 0.2
                move_cmd.angular.z = copysign(2, self.error)'''
            self.pub_cmdVel.publish(move_cmd)
        self.moving = True


    def Square(self):
        """Square: ida e volta (vira 180°) — não quadrado real (usa 180° como R2, não 90°)."""
        #if self.index in range(2):
        if self.index == 0:
            print("Length")
            step1 = self.advancing(self.Length)
            #sleep(0.5)
            if step1 == True:
                #self.distance = 0.0
                self.index = self.index + 1;
                self.Switch  = rclpy.parameter.Parameter('Switch',rclpy.Parameter.Type.BOOL,True)
                all_new_parameters = [self.Switch]
                self.set_parameters(all_new_parameters)
        elif self.index == 1:
            print("Spin")
            step2 = self.Spin(180)  # 180° (não 90° como patrol_a1_X3)
            #sleep(0.5)
            if step2 == True:
                self.index = self.index + 1;
                self.Switch  = rclpy.parameter.Parameter('Switch',rclpy.Parameter.Type.BOOL,True)
                all_new_parameters = [self.Switch]
                self.set_parameters(all_new_parameters)
        elif self.index == 2:
            print("Length")
            step3 = self.advancing(self.Length)
            #sleep(0.5)
            if step3 == True:
                self.index = self.index + 1;
                self.Switch  = rclpy.parameter.Parameter('Switch',rclpy.Parameter.Type.BOOL,True)
                all_new_parameters = [self.Switch]
                self.set_parameters(all_new_parameters)
        elif self.index == 3:
            print("Spin")
            step4 = self.Spin(180)
            #sleep(0.5)
            if step4 == True:
                self.index = self.index + 1;
                self.Switch  = rclpy.parameter.Parameter('Switch',rclpy.Parameter.Type.BOOL,True)
                all_new_parameters = [self.Switch]
                self.set_parameters(all_new_parameters)
        else:
            self.index = 0
            self.Switch  = rclpy.parameter.Parameter('Switch',rclpy.Parameter.Type.BOOL,False)
            all_new_parameters = [self.Switch]
            self.set_parameters(all_new_parameters)
            #self.Command == "finish"
            print("Done!")
            return True



    def get_odom_angle(self):
        """Lê o ângulo yaw atual do robô a partir do TF odom→base_footprint."""
        try:
            now = rclpy.time.Time()
            rot = self.tf_buffer.lookup_transform(self.odom_frame,self.base_frame,now)
            #print("oring_rot: ",rot.transform.rotation)
            cacl_rot = PyKDL.Rotation.Quaternion(rot.transform.rotation.x, rot.transform.rotation.y, rot.transform.rotation.z, rot.transform.rotation.w)
            #print("cacl_rot: ",cacl_rot)
            angle_rot = cacl_rot.GetRPY()[2]  # yaw




        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().info('transform not ready')
            return

        return angle_rot


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

    def normalize_angle(self,angle):
        """Normaliza ângulo para o intervalo [-π, π]."""
        res = angle
        #print("res: ",res)
        while res > pi:
            res -= 2.0 * pi
        while res < -pi:
            res += 2.0 * pi
        return res

    def LaserScanCallback(self,scan_data):
        """Conta leituras próximas frontais do laser. Se >10, robô pára por obstáculo.
        NOTA: esta versão usa abs(angle) < LaserAngle (frente absoluta),
        diferente do patrol_a1_X3 que usa > (180-LaserAngle)."""
        if self.ResponseDist == 0: return
        ranges = np.array(scan_data.ranges)
        sortedIndices = np.argsort(ranges)
        self.warning = 1
        for i in range(len(ranges)):
            angle = (scan_data.angle_min + scan_data.angle_increment * i) * RAD2DEG
            if abs(angle) < self.LaserAngle:  # frente absoluta (vs ±180° do a1_X3)
                if ranges[i] < self.ResponseDist: self.warning += 1

def main():
    rclpy.init()
    class_patrol = YahboomCarPatrol("YahboomCarPatrol")
    print("create done")
    rclpy.spin(class_patrol)
