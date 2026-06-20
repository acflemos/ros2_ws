#!/usr/bin/env python
# encoding: utf-8
#
# Ackman_driver_R2.py — Driver de hardware do ROSMASTER R2 (ROS2, Ackermann)
# ============================================================================
# Nó ROS2 equivalente ao Mcnamu_driver_X3.py mas para o modelo R2 (direção Ackermann).
# A diferença principal está na conversão de velocidade lateral para ângulo de direção.
#
# Subscritores / Publicadores: idênticos ao Mcnamu_driver_X3.
#
# Diferença crítica no cmd_vel_callback vs X3:
#   - R2: vy do /cmd_vel é tratado como ângulo de direção.
#         O firmware Arduino interpreta vy diretamente como ângulo (em radianos ou escala própria).
#         Comentário original sugeria /1000.0*180.0/3.1416 mas está comentado —
#         a versão ativa envia vy*1.0 igual ao X3. Isto é suspeito (possível bug).
#
# Diferença crítica no pub_data vs X3:
#   - R2: /vel_raw.linear.y = vy_encoder * 1000  (escala millirad para rad → base_node_R2)
#     X3: /vel_raw.linear.y = vy_encoder * 1.0   (metros por segundo, sem escala)
#   - R2 publica state.position com ângulo de esterçamento calculado
#   - Quando em movimento (vx ≠ 0 ou angular ≠ 0), posição das rodas de tração é
#     aleatória — simula rotação contínua para animação visual no RViz2
#
# Parâmetros ROS2 (diferenças vs X3):
#   car_type       (default: 'R2')   — ativa car_type=5 no firmware
#   angular_limit  (default: 1.0)    — menor que X3 (5.0): carro Ackermann tem raio mínimo
#   nav_use_rotvel (default: False)  — flag para usar vel angular de navegação (não implementado)
#
# Hardware: mesmo Arduino/Rosmaster_Lib, mas configurado como car_type=5 (R2/Ackermann).
#
# Relevância para robodog2: baixa — robodog2 usa X3 mecanum, não Ackermann.
# Útil como referência para entender as diferenças de protocolo com o Arduino.

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

# Mapeamento de tipo de robô para código de hardware (firmware Arduino)
car_type_dic={
    'R2':5,   # Ackermann — este ficheiro
    'X3':1,   # Mecanum 4WD
    'NONE':-1
}
class yahboomcar_driver(Node):
	def __init__(self, name):
		super().__init__(name)
		global car_type_dic
		self.RA2DE = 180 / pi
		self.car = Rosmaster()
		self.car.set_car_type(5)  # modo R2 Ackermann no firmware Arduino (vs 1 para X3)
		#get parameter
		self.declare_parameter('car_type', 'R2')
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
		self.declare_parameter('angular_limit', 1.0)  # mais baixo que X3 (5.0) — raio mínimo Ackermann
		self.angular_limit = self.get_parameter('angular_limit').get_parameter_value().double_value
		print (self.angular_limit)
		self.declare_parameter('nav_use_rotvel', False)  # parâmetro extra vs X3 (não implementado no código)
		self.nav_use_rotvel = self.get_parameter('nav_use_rotvel').get_parameter_value().bool_value
		print (self.nav_use_rotvel)

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
		vy = msg.linear.y*1.0  # SUSPEITO: R2 deveria converter vy para ângulo antes de enviar
		angular = msg.angular.z*1.0    # wait for change
		self.car.set_car_motion(vx, vy, angular)
		# self.car.set_car_motion(vx*1.8, vy, angular)
		# self.get_logger().info("cmd_vel_callback: vx = {}, vy = {}, angular = {},".format(vx, vy, angular))
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
		if len(self.Prefix)==0:
			state.name = ["back_right_joint", "back_left_joint","front_left_steer_joint","front_left_wheel_joint",
							"front_right_steer_joint", "front_right_wheel_joint"]
		else:
			state.name = [self.Prefix+"back_right_joint",self.Prefix+ "back_left_joint",self.Prefix+"front_left_steer_joint",self.Prefix+"front_left_wheel_joint",
							self.Prefix+"front_right_steer_joint", self.Prefix+"front_right_wheel_joint"]

		#print ("mag: ",self.car.get_magnetometer_data())
		edition.data = self.car.get_version()*1.0
		battery.data = self.car.get_battery_voltage()*1.0
		ax, ay, az = self.car.get_accelerometer_data()
		# self.get_logger().info("ax = {}, ay = {}, az = {} ".format(ax,ay,az))
		gx, gy, gz = self.car.get_gyroscope_data()
		# self.get_logger().info("gx = {}, gy = {}, gz = {} ".format(gx,gy,gz))
		mx, my, mz = self.car.get_magnetometer_data()
		# self.get_logger().info("mx = {}, my = {}, mz = {} ".format(mx,my,mz))
		mx = mx * 1.0
		my = my * 1.0
		mz = mz * 1.0
		vx, vy, angular = self.car.get_motion_data()
		# self.get_logger().info("get_motion_data: vx = {}, vy = {}, angular = {},".format(vx, vy, angular))

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

		mag.header.stamp = time_stamp.to_msg()
		mag.header.frame_id = self.imu_link
		mag.magnetic_field.x = mx*1.0
		mag.magnetic_field.y = my*1.0
		mag.magnetic_field.z = mz*1.0

		# 将小车当前的线速度和角速度发布出去
		# Publish the current linear vel and angular vel of the car
		twist.linear.x = vx*1.0    #velocity in axis
		# R2: vy é escalado ×1000 para o base_node_R2 converter para ângulo de esterçamento
		# Diferença crítica vs X3 (que publica vy*1.0 em m/s)
		twist.linear.y = vy*1000*1.0   #steer angle (em millirad)
		#twist.linear.y = vy   #steer angle
		#twist.angular.z = angular
		twist.angular.z = angular*1.0    #this is invalued
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

		# Converte ângulo de esterçamento de millirad para rad para publicar em joint_states
		# Fórmula original: vy*1000 (millirad) * pi/180 — mistura millirad com conversão grau→rad
		# O resultado é aproximadamente o ângulo de esterçamento em rad (com imprecisão numérica)
		steer_radis = vy*1000.0*3.1416/180.0
		state.position = [0.0, 0.0, steer_radis, 0.0, steer_radis, 0.0]
		if not vx == angular == 0:
			# animação de rotação das rodas para o RViz2 — posição aleatória simula movimento
			i = random.uniform(-3.14, 3.14)
			state.position = [i, i, steer_radis, i, steer_radis, i]
		self.staPublisher.publish(state)

def main():
	rclpy.init()
	driver = yahboomcar_driver('driver_node')
	rclpy.spin(driver)

'''if __name__ == '__main__':
	main()'''



