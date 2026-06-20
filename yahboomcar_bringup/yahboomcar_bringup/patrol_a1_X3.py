#!/usr/bin/env python
# encoding: utf-8
#
# patrol_a1_X3.py — Patrulha autónoma geométrica para o X3 (com obstacle avoidance)
# ===================================================================================
# Nó de patrulha autónoma que executa sequências geométricas configuráveis via
# parâmetros ROS2 em runtime. Versão "a1": inclui subscrição ao /JoyState (joystick
# pode interromper) e obstacle avoidance via LaserScan.
#
# Localização: usa TF (odom → base_footprint) para medir distância e ângulo percorridos.
# Obstacle avoidance: conta leituras do /scan frontais (ângulos próximos de ±180°)
#   < ResponseDist; se warning > 10, para até obstáculo desaparecer.
#
# Subscritores:
#   /scan      (sensor_msgs/LaserScan) — deteção de obstáculos
#   /JoyState  (std_msgs/Bool)         — se True, joystick está ativo → pausa patrulha
#
# Publicadores:
#   /cmd_vel   (geometry_msgs/Twist)   — comandos de velocidade para o robô
#
# TF necessário:
#   odom → base_footprint (publicado pelo base_node + EKF ou pelo Gazebo)
#
# Comandos suportados (parâmetro 'Command'):
#   "Square"     — quadrado: avança Length, vira 90°, repete 4 lados (9 passos no total)
#   "Triangle"   — triângulo: avança Length, vira 120°, repete 3 lados
#   "Circle"     — círculo: avança com angular.z constante proporcional a Linear/Length
#   "LengthTest" — avança uma distância reta (teste de odometria linear)
#
# Parâmetros ROS2 (configuráveis em runtime com ros2 param set):
#   Switch            (bool, false)  — ativa/desativa execução
#   Command           (str, "Square")— comando atual
#   Set_loop          (bool, false)  — repetir em loop ao terminar
#   Linear            (float, 0.5)  — velocidade linear (m/s)
#   Angular           (float, 1.0)  — velocidade angular (rad/s)
#   Length            (float, 1.0)  — comprimento do segmento ou raio (m)
#   ResponseDist      (float, 0.6)  — distância de deteção de obstáculos (m); 0=desativado
#   LaserAngle        (float, 20.0) — ângulo de deteção frontal (graus, a partir de ±180°)
#   RotationTolerance (float, 0.3)  — tolerância angular de paragem (rad)
#   RotationScaling   (float, 1.0)  — fator de escala para compensar drift de odometria angular
#   odom_frame        (str, "odom") — frame de odometria
#   base_frame        (str, "base_footprint") — frame base do robô
#   circle_adjust     (float, 2.0)  — fator de ajuste do raio do círculo
#
# Diferença vs patrol_4ROS.py (versão sem JoyState):
#   patrol_a1_X3: subscreve /JoyState → para se joystick ativo
#   patrol_4ROS:  sem /JoyState (apenas obstacle avoidance por laser)
#   patrol_a1_X3: Square faz 4 lados de 90° (correto geometricamente)
#   patrol_4ROS:  Square faz 2 lados de 180° (ida e volta)
#   patrol_a1_X3: LaserAngle monitoriza ângulos >(180-LaserAngle)° (frente do robô X3)
#   patrol_4ROS:  LaserAngle monitoriza ângulos <LaserAngle° (frente absoluta)
#
# Relevância para robodog2: referência de patrulha autónoma simples sem Nav2.
#   O robodog2 usa Nav2 para navegação (mais robusto e com mapa), mas este nó
#   pode ser útil para testes de odometria ou como modo de fallback.
#   Para adaptar: substituir odom_frame/base_frame pelos frames do robodog2.

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

# fator de conversão rad→grau para o processamento do LaserScan
RAD2DEG = 180 / math.pi

class YahboomCarPatrol(Node):
    def __init__(self,name):
        super().__init__(name)
        # flag de movimento (para detetar transição em movimento→parado por obstáculo)
        self.moving = True
        self.Joy_active = False
        # memoriza o último comando executado para o loop
        self.command_src = "finish"
        self.warning = 1
        self.SetLoop = False
        self.Linear = 0.5
        self.Angular = 1.0
        self.Length = 1.0 #1.0
        self.Angle = 360.0
        # LineScaling: corrige drift de odometria linear (valor > 1 = odom subestima distância)
        self.LineScaling = 1.1
        self.RotationScaling = 1.0
        self.LineTolerance = 0.1
        self.RotationTolerance = 0.3
        #self.ResponseDist = 0.6
        #self.LaserAngle = 20
        self.warning = 1
        #self.Command = "LengthTest"
        #self.Switch = False
        self.position = Point()
        self.x_start = self.position.x
        self.y_start = self.position.y
        self.error = 0.0
        self.distance = 0.0
        self.last_angle = 0.0
        self.delta_angle = 0.0
        self.turn_angle = 0.0  # acumulador de rotação para o Spin()
        #create publisher
        self.pub_cmdVel = self.create_publisher(Twist,"/cmd_vel",5)
        #create subscriber
        self.sub_scan = self.create_subscription(LaserScan,"/scan",self.LaserScanCallback,1)
        # /JoyState: se joystick ativo, pausa a patrulha para controlo manual ter prioridade
        self.sub_joy = self.create_subscription(Bool,"/JoyState",self.JoyStateCallback,1)
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
        self.declare_parameter('Linear',0.5)
        self.Linear = self.get_parameter('Linear').get_parameter_value().double_value
        self.declare_parameter('Angular',1.0)
        self.Angular = self.get_parameter('Angular').get_parameter_value().double_value
        self.declare_parameter('Length',1.0)
        self.Length = self.get_parameter('Length').get_parameter_value().double_value
        self.declare_parameter('RotationTolerance',0.3)
        self.RotationTolerance = self.get_parameter('RotationTolerance').get_parameter_value().double_value
        self.declare_parameter('RotationScaling',1.0)
        self.RotationScaling = self.get_parameter('RotationScaling').get_parameter_value().double_value

        # timer a 100Hz para baixa latência de controlo
        self.timer = self.create_timer(0.01,self.on_timer)
        # índice do passo atual dentro de Square/Triangle (estado da máquina)
        self.index = 0


    def on_timer(self):
        #print("self.error: ",self.error)
        # relê parâmetros a cada tick — permite ajuste em tempo real via ros2 param set
        self.Switch = self.get_parameter('Switch').get_parameter_value().bool_value
        self.Command = self.get_parameter('Command').get_parameter_value().string_value
        self.Set_loop = self.get_parameter('Set_loop').get_parameter_value().bool_value
        self.ResponseDist = self.get_parameter('ResponseDist').get_parameter_value().double_value
        self.Linear = self.get_parameter('Linear').get_parameter_value().double_value
        self.Angular = self.get_parameter('Angular').get_parameter_value().double_value
        self.Length = self.get_parameter('Length').get_parameter_value().double_value
        self.LaserAngle = self.get_parameter('LaserAngle').get_parameter_value().double_value
        self.RotationTolerance = self.get_parameter('RotationTolerance').get_parameter_value().double_value
        self.RotationScaling = self.get_parameter('RotationScaling').get_parameter_value().double_value


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

            elif self.Command == "Triangle":
                self.command_src = "Triangle"
                triangle = self.Triangle()
                if triangle == True:
                    self.Command  = rclpy.parameter.Parameter('Command',rclpy.Parameter.Type.STRING,"finish")
                    all_new_parameters = [self.Command]
                    self.set_parameters(all_new_parameters)

        else:
            # Switch=False: para o robô e verifica se deve repetir (loop)
            self.pub_cmdVel.publish(Twist())
            print("Switch False")
            if self.Command == "finish":
                print("finish")
                if self.Set_loop == True:
                    print("Continute")
                    # reinicia o comando anterior e ativa Switch para recomeçar
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
        # distância Euclidiana percorrida desde o início do segmento
        self.distance = sqrt(pow((self.position.x - self.x_start), 2) +
                            pow((self.position.y - self.y_start), 2))
        # LineScaling corrige sub-estimativa da odometria (valor típico: 1.1)
        self.distance *= self.LineScaling
        print("distance: ",self.distance)
        self.error = self.distance - target_distance
        move_cmd.linear.x = self.Linear
        if abs(self.error) < self.LineTolerance :
            print("stop")
            self.distance = 0.0
            self.pub_cmdVel.publish(Twist())
            # atualiza ponto de referência para o próximo segmento
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
        """Roda no lugar o ângulo especificado (graus), usando TF para medir rotação."""
        self.target_angle = radians(angle)
        self.odom_angle = self.get_odom_angle()
        # delta incremental de rotação desde o último tick
        self.delta_angle = self.RotationScaling * self.normalize_angle(self.odom_angle - self.last_angle)
        self.turn_angle += self.delta_angle
        print("turn_angle: ",self.turn_angle)
        self.error = self.target_angle - self.turn_angle
        print("error: ",self.error)
        self.last_angle = self.odom_angle
        move_cmd = Twist()
        if abs(self.error) < self.RotationTolerance or self.Switch==False :
            self.pub_cmdVel.publish(Twist())
            self.turn_angle = 0.0  # reset acumulador para o próximo Spin
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
            if self.Command == "Square" or self.Command == "Triangle":
                # rotação no lugar (sem movimento linear)
                #move_cmd.linear.x = 0.2
                move_cmd.angular.z = copysign(self.Angular, self.error)
            elif self.Command == "Circle":
                # círculo: combina linear com angular proporcional ao raio pedido
                length = self.Linear * self.circle_adjust / self.Length
                #print("length: ",length)
                move_cmd.linear.x = self.Linear
                move_cmd.angular.z = copysign(length, self.error)
                #print("angular: ",move_cmd.angular.z)
                '''move_cmd.linear.x = 0.2
                move_cmd.angular.z = copysign(2, self.error)'''
            self.pub_cmdVel.publish(move_cmd)
        self.moving = True


    def Square(self):
        """Máquina de estados: quadrado com 4 lados de 90° cada (8 passos + final)."""
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
            step2 = self.Spin(90)
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
            step4 = self.Spin(90)
            #sleep(0.5)
            if step4 == True:
                self.index = self.index + 1;
                self.Switch  = rclpy.parameter.Parameter('Switch',rclpy.Parameter.Type.BOOL,True)
                all_new_parameters = [self.Switch]
                self.set_parameters(all_new_parameters)
        elif self.index == 4:
            print("Length")
            step5 = self.advancing(self.Length)
            #sleep(0.5)
            if step5 == True:
                self.index = self.index + 1;
                self.Switch  = rclpy.parameter.Parameter('Switch',rclpy.Parameter.Type.BOOL,True)
                all_new_parameters = [self.Switch]
                self.set_parameters(all_new_parameters)
        elif self.index == 5:
            print("Spin")
            step6 = self.Spin(90)
            #sleep(0.5)
            if step6 == True:
                self.index = self.index + 1;
                self.Switch  = rclpy.parameter.Parameter('Switch',rclpy.Parameter.Type.BOOL,True)
                all_new_parameters = [self.Switch]
                self.set_parameters(all_new_parameters)
        elif self.index == 6:
            print("Length")
            step7 = self.advancing(self.Length)
            #sleep(0.5)
            if step7 == True:
                self.index = self.index + 1;
                self.Switch  = rclpy.parameter.Parameter('Switch',rclpy.Parameter.Type.BOOL,True)
                all_new_parameters = [self.Switch]
                self.set_parameters(all_new_parameters)
        elif self.index == 7:
            print("Spin")
            step8 = self.Spin(90)
            #sleep(0.5)
            if step8 == True:
                self.index = self.index + 1;
                self.Switch  = rclpy.parameter.Parameter('Switch',rclpy.Parameter.Type.BOOL,True)
                all_new_parameters = [self.Switch]
                self.set_parameters(all_new_parameters)
        elif self.index == 8:
            # último lado (4 lados × 2 passos = 8, mas Square tem 9 steps — BUG ou intencional?)
            print("Length")
            step9 = self.advancing(self.Length)
            #sleep(0.5)
            if step9 == True:
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

    def Triangle(self):
        """Máquina de estados: triângulo com 3 lados de 120° cada."""
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
            step2 = self.Spin(120)
            #sleep(0.5)
            if step2 == True:
                self.index = self.index + 1;
                self.Switch  = rclpy.parameter.Parameter('Switch',rclpy.Parameter.Type.BOOL,True)
                all_new_parameters = [self.Switch]
                self.set_parameters(all_new_parameters)
        elif self.index == 2:
            print("Length")
            step1 = self.advancing(self.Length)
            #sleep(0.5)
            if step1 == True:
                #self.distance = 0.0
                self.index = self.index + 1;
                self.Switch  = rclpy.parameter.Parameter('Switch',rclpy.Parameter.Type.BOOL,True)
                all_new_parameters = [self.Switch]
                self.set_parameters(all_new_parameters)
        elif self.index == 3:
            print("Spin")
            step4 = self.Spin(120)
            #sleep(0.5)
            if step4 == True:
                self.index = self.index + 1;
                self.Switch  = rclpy.parameter.Parameter('Switch',rclpy.Parameter.Type.BOOL,True)
                all_new_parameters = [self.Switch]
                self.set_parameters(all_new_parameters)
        elif self.index == 4:
            print("Length")
            step5 = self.advancing(self.Length)
            #sleep(0.5)
            if step5 == True:
                #self.distance = 0.0
                self.index = self.index + 1;
                self.Switch  = rclpy.parameter.Parameter('Switch',rclpy.Parameter.Type.BOOL,True)
                all_new_parameters = [self.Switch]
                self.set_parameters(all_new_parameters)

        elif self.index == 5:
            print("Spin")
            step6 = self.Spin(120)
            #sleep(0.5)
            if step6 == True:
                self.index = self.index + 1;
                self.Switch  = rclpy.parameter.Parameter('Switch',rclpy.Parameter.Type.BOOL,True)
                all_new_parameters = [self.Switch]
                self.set_parameters(all_new_parameters)
        else:
            self.index = 0
            self.Switch  = rclpy.parameter.Parameter('Switch',rclpy.Parameter.Type.BOOL,False)
            all_new_parameters = [self.Switch]
            self.set_parameters(all_new_parameters)
            print("Done!")
            return True


    def get_odom_angle(self):
        """Lê o ângulo yaw atual do robô a partir do TF odom→base_footprint."""
        try:
            now = rclpy.time.Time()
            rot = self.tf_buffer.lookup_transform(self.odom_frame,self.base_frame,now)
            #print("oring_rot: ",rot.transform.rotation)
            # PyKDL converte quaternion para RPY (roll, pitch, yaw)
            cacl_rot = PyKDL.Rotation.Quaternion(rot.transform.rotation.x, rot.transform.rotation.y, rot.transform.rotation.z, rot.transform.rotation.w)
            #print("cacl_rot: ",cacl_rot)
            angle_rot = cacl_rot.GetRPY()[2]  # [2] = yaw (rotação em Z)




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
        """Normaliza ângulo para o intervalo [-π, π] para evitar saltos de 2π."""
        res = angle
        #print("res: ",res)
        while res > pi:
            res -= 2.0 * pi
        while res < -pi:
            res += 2.0 * pi
        return res

    def LaserScanCallback(self,scan_data):
        """Conta leituras próximas frontais do laser. Se >10, robô pára por obstáculo."""
        if self.ResponseDist == 0: return
        ranges = np.array(scan_data.ranges)
        sortedIndices = np.argsort(ranges)
        self.warning = 1
        for i in range(len(ranges)):
            # ângulo em graus — valores próximos de ±180° = frente do robô X3
            angle = (scan_data.angle_min + scan_data.angle_increment * i) * RAD2DEG
            if abs(angle) > (180 - self.LaserAngle):
                if ranges[i] < self.ResponseDist: self.warning += 1

    def JoyStateCallback(self, msg):
        """Atualiza flag Joy_active — quando True, pausa a patrulha."""
        if not isinstance(msg, Bool): return
        self.Joy_active = msg.data
        #print(msg.data)
        #if not self.Joy_active: self.pub_cmdVel.publish(Twist())

def main():
    rclpy.init()
    class_patrol = YahboomCarPatrol("YahboomCarPatrol")
    print("create done")
    rclpy.spin(class_patrol)
