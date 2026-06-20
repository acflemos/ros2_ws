#!/usr/bin/env python
# encoding: utf-8
#
# Mcnamu_driver_X3.py — Driver de hardware do ROSMASTER X3 (ROS2, mecanum 4WD)
# ==============================================================================
# Nó ROS2 que faz a ponte entre o hardware do Arduino (via Rosmaster_Lib) e o ROS2.
# Responsável por: ler sensores (IMU, encoders, bateria) e executar comandos de movimento.
#
# PEÇA CENTRAL para o robodog2:
#   Em hardware real, este nó é a única interface com o Arduino.
#   Em simulação (Gazebo Fortress), este nó é substituído pelos plugins:
#     - libgazebo_ros_diff_drive (ou mecanum) → equivalente ao /cmd_vel + /vel_raw
#     - libgazebo_ros_imu_sensor              → equivalente ao /imu/data_raw
#   O resto da cadeia (imu_filter_madgwick → EKF → Nav2) permanece idêntico.
#   Para o rbd2_bringup (hardware real), criar rbd2_driver baseado neste ficheiro,
#   substituindo a Rosmaster_Lib por qualquer biblioteca de comunicação Arduino equivalente.
#
# Subscritores:
#   /cmd_vel  (geometry_msgs/Twist) — recebe comandos de velocidade
#       linear.x:   velocidade longitudinal (m/s)
#       linear.y:   velocidade lateral (m/s) — movimento holonômico das mecanum
#       angular.z:  velocidade angular (rad/s)
#   /RGBLight (std_msgs/Int32)  — controla efeito de LEDs RGB (0..X)
#   /Buzzer   (std_msgs/Bool)   — liga/desliga buzzer do Arduino
#
# Publicadores:
#   /imu/data_raw (sensor_msgs/Imu)           — dados brutos acelerómetro + giroscópio (10Hz)
#                                               sem orientação calculada — o imu_filter_madgwick
#                                               adiciona a orientação via fusão de sensores
#   /imu/mag      (sensor_msgs/MagneticField)  — dados do magnetómetro (10Hz)
#                                               não usado pelo Madgwick (use_mag: false)
#   /vel_raw      (geometry_msgs/Twist)        — velocidade lida dos encoders do Arduino
#                                               entrada do base_node_X3 para calcular /odom
#   /joint_states (sensor_msgs/JointState)     — posição estimada das rodas (para TF/RViz)
#   /voltage      (std_msgs/Float32)           — tensão da bateria (V)
#   /edition      (std_msgs/Float32)           — versão do firmware do Arduino
#
# Parâmetros ROS2:
#   car_type       (default: 'X3')   — tipo de robô; determina modo no firmware
#   imu_link       (default: 'imu_link') — frame do IMU para os headers das mensagens
#   Prefix         (default: '')     — prefixo de namespace para nomes dos joints
#   xlinear_limit  (default: 1.0)    — limite máximo de velocidade linear x (m/s)
#   ylinear_limit  (default: 1.0)    — limite máximo de velocidade linear y (m/s)
#   angular_limit  (default: 5.0)    — limite máximo de velocidade angular (rad/s)
#
# Diferença X3 vs R2 (Ackman_driver_R2.py):
#   - X3: set_car_type(1); vy é velocidade lateral real (mecanum holonômica)
#   - R2: set_car_type(5); vy é convertido para ângulo de direção em milirradianos
#
# BUG CONHECIDO:
#   Os nomes em /joint_states são os do R2 (steer_joints), não do X3 (mecanum).
#   O X3 deveria publicar: front_right_joint, front_left_joint, back_right_joint, back_left_joint
#   Mas publica: back_right_joint, back_left_joint, front_left_steer_joint, etc.
#   Não afeta o hardware real (base_node ignora os nomes), mas pode causar
#   erros no robot_state_publisher se os nomes não baterem com o URDF.
#   Além disso, o X3 não publica state.position, apenas state.name — o URDF fica
#   com posição 0.0 para todas as rodas (sem animação de rotação).
#
# Hardware: Arduino com firmware Rosmaster via porta série USB.
#           A biblioteca Rosmaster_Lib abstrai o protocolo série proprietário Yahboom.

#public lib
import sys
import math
import random
import threading
from math import pi
from time import sleep
from Rosmaster_Lib import Rosmaster  # biblioteca proprietária Yahboom (não open-source)

#ros lib
import rclpy
from rclpy.node import Node
from std_msgs.msg import String,Float32,Int32,Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu,MagneticField, JointState
from rclpy.clock import Clock

# Mapeamento de tipo de robô para código de hardware (interpretado pelo firmware Arduino)
# Apenas X3 é relevante para o robodog2
car_type_dic={
    'R2':5,   # Ackermann — modo de direção diferente
    'X3':1,   # Mecanum 4WD — modo do robodog2
    'NONE':-1
}
class yahboomcar_driver(Node):
	def __init__(self, name):
		super().__init__(name)
		global car_type_dic
		self.RA2DE = 180 / pi  # fator de conversão rad→grau (não usado diretamente neste nó)
		self.car = Rosmaster()
		self.car.set_car_type(1)  # modo X3 mecanum no firmware Arduino
		#get parameter
		self.declare_parameter('car_type', 'X3')
		self.car_type = self.get_parameter('car_type').get_parameter_value().string_value
		print (self.car_type)
		self.declare_parameter('imu_link', 'imu_link')
		self.imu_link = self.get_parameter('imu_link').get_parameter_value().string_value
		print (self.imu_link)
		self.declare_parameter('Prefix', "")
		self.Prefix = self.get_parameter('Prefix').get_parameter_value().string_value
		print (self.Prefix)
		self.declare_parameter('xlinear_limit', 1.0)
		self.xlinear_limit = self.get_parameter('xlinear_limit').get_parameter_value().double_value
		print (self.xlinear_limit)
		self.declare_parameter('ylinear_limit', 1.0)
		self.ylinear_limit = self.get_parameter('ylinear_limit').get_parameter_value().double_value
		print (self.ylinear_limit)
		self.declare_parameter('angular_limit', 5.0)
		self.angular_limit = self.get_parameter('angular_limit').get_parameter_value().double_value
		print (self.angular_limit)

		#create subcriber
		self.sub_cmd_vel = self.create_subscription(Twist,"cmd_vel",self.cmd_vel_callback,1)
		self.sub_RGBLight = self.create_subscription(Int32,"RGBLight",self.RGBLightcallback,100)
		self.sub_BUzzer = self.create_subscription(Bool,"Buzzer",self.Buzzercallback,100)

		#create publisher
		self.EdiPublisher = self.create_publisher(Float32,"edition",100)
		self.volPublisher = self.create_publisher(Float32,"voltage",100)
		self.staPublisher = self.create_publisher(JointState,"joint_states",100)
		self.velPublisher = self.create_publisher(Twist,"vel_raw",50)
		self.imuPublisher = self.create_publisher(Imu,"/imu/data_raw",100)
		self.magPublisher = self.create_publisher(MagneticField,"/imu/mag",100)

		# timer a 10Hz para publicar todos os dados de sensores
		self.timer = self.create_timer(0.1, self.pub_data)

		#create and init variable
		self.edition = Float32()
		self.edition.data = 1.0
		# inicia thread de receção serial em background (leitura assíncrona do Arduino)
		self.car.create_receive_threading()

	#callback function
	def cmd_vel_callback(self,msg):
        # 小车运动控制，订阅者回调函数
        # Car motion control, subscriber callback function
		if not isinstance(msg, Twist): return
        # 下发线速度和角速度
        # Issue linear vel and angular vel
		vx = msg.linear.x*1.0
        #vy = msg.linear.y/1000.0*180.0/3.1416    #Radian system
		vy = msg.linear.y*1.0   # no X3: velocidade lateral real (não ângulo de direção)
		angular = msg.angular.z*1.0     # wait for chang
		# envia os três valores diretamente ao Arduino — firmware converte para RPM por roda
		self.car.set_car_motion(vx, vy, angular)
		'''print("cmd_vx: ",vx)
		print("cmd_vy: ",vy)
		print("cmd_angular: ",angular)'''
        #rospy.loginfo("nav_use_rot:{}".format(self.nav_use_rotvel))
        #print(self.nav_use_rotvel)

	def RGBLightcallback(self,msg):
        # 流水灯控制，服务端回调函数 RGBLight control
		if not isinstance(msg, Int32): return
		# print ("RGBLight: ", msg.data)
		# enviar 3x para garantir receção (protocolo série sem ACK)
		for i in range(3): self.car.set_colorful_effect(msg.data, 6, parm=1)

	def Buzzercallback(self,msg):
		if not isinstance(msg, Bool): return
		if msg.data:
			for i in range(3): self.car.set_beep(1)
		else:
			for i in range(3): self.car.set_beep(0)

	#pub data
	def pub_data(self):
		time_stamp = Clock().now()
		imu = Imu()
		twist = Twist()
		battery = Float32()
		edition = Float32()
		mag = MagneticField()
		state = JointState()
		state.header.stamp = time_stamp.to_msg()
		state.header.frame_id = "joint_states"
		# BUG: nomes dos joints são os do R2 (com steer_joints), não os do X3 (mecanum)
		# Para o robodog2, corrigir para: front_right_joint, front_left_joint, back_right_joint, back_left_joint
		if len(self.Prefix)==0:
			state.name = ["back_right_joint", "back_left_joint","front_left_steer_joint","front_left_wheel_joint",
							"front_right_steer_joint", "front_right_wheel_joint"]
		else:
			state.name = [self.Prefix+"back_right_joint",self.Prefix+ "back_left_joint",self.Prefix+"front_left_steer_joint",self.Prefix+"front_left_wheel_joint",
							self.Prefix+"front_right_steer_joint", self.Prefix+"front_right_wheel_joint"]

		#print ("mag: ",self.car.get_magnetometer_data())
		edition.data = self.car.get_version()*1.0
		battery.data = self.car.get_battery_voltage()*1.0
		ax, ay, az = self.car.get_accelerometer_data()  # m/s² — aceleração linear
		gx, gy, gz = self.car.get_gyroscope_data()      # rad/s — velocidade angular
		mx, my, mz = self.car.get_magnetometer_data()   # µT — campo magnético
		mx = mx * 1.0
		my = my * 1.0
		mz = mz * 1.0
		vx, vy, angular = self.car.get_motion_data()    # velocidade atual medida pelos encoders
		'''print("vx: ",vx)
		print("vy: ",vy)
		print("angular: ",angular)'''
		# 发布陀螺仪的数据
		# Publish gyroscope data
		imu.header.stamp = time_stamp.to_msg()
		imu.header.frame_id = self.imu_link
		imu.linear_acceleration.x = ax*1.0
		imu.linear_acceleration.y = ay*1.0
		imu.linear_acceleration.z = az*1.0
		imu.angular_velocity.x = gx*1.0
		imu.angular_velocity.y = gy*1.0
		imu.angular_velocity.z = gz*1.0
		# nota: imu.orientation NÃO é preenchido — será calculado pelo imu_filter_madgwick

		mag.header.stamp = time_stamp.to_msg()
		mag.header.frame_id = self.imu_link
		mag.magnetic_field.x = mx*1.0
		mag.magnetic_field.y = my*1.0
		mag.magnetic_field.z = mz*1.0

		# 将小车当前的线速度和角速度发布出去
		# Publish the current linear vel and angular vel of the car
		twist.linear.x = vx *1.0
		twist.linear.y = vy *1.0   # no X3: velocidade lateral em m/s (não ângulo como no R2)
		twist.angular.z = angular*1.0
		self.velPublisher.publish(twist)
		# print("ax: %.5f, ay: %.5f, az: %.5f" % (ax, ay, az))
		# print("gx: %.5f, gy: %.5f, gz: %.5f" % (gx, gy, gz))
		# print("mx: %.5f, my: %.5f, mz: %.5f" % (mx, my, mz))
		# rospy.loginfo("battery: {}".format(battery))
		# rospy.loginfo("vx: {}, vy: {}, angular: {}".format(twist.linear.x, twist.linear.y, twist.angular.z))
		self.imuPublisher.publish(imu)
		self.magPublisher.publish(mag)
		self.volPublisher.publish(battery)
		self.EdiPublisher.publish(edition)
		# nota: state (joint_states) não é publicado aqui — o joint_state_publisher do launch faz isso
		# Para animar as rodas no RViz2, seria necessário calcular posição angular das rodas



def main():
	rclpy.init()
	driver = yahboomcar_driver('driver_node')
	rclpy.spin(driver)

'''if __name__ == '__main__':
	main()'''



