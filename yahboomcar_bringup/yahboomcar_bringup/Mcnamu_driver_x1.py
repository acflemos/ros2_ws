#!/usr/bin/env python
# encoding: utf-8
#
# Mcnamu_driver_x1.py — Driver de hardware do ROSMASTER X1 (ROS2, mecanum menor)
# ================================================================================
# Nó ROS2 equivalente ao Mcnamu_driver_X3.py mas para o modelo X1.
# O X1 é uma plataforma mecanum mais pequena e leve que o X3.
# Estrutura idêntica ao Mcnamu_driver_X3 com diferença fundamental:
#   set_car_type(4) → firmware Arduino em modo X1 (vs 1 para X3)
#
# Diferenças vs Mcnamu_driver_X3:
#   - car_type=4 (X1) em vez de 1 (X3) → cinemática diferente no Arduino
#   - angular_limit=1.0 (X3 usa 5.0) → X1 tem raio de rotação diferente
#   - nav_use_rotvel declarado (não existe no X3)
#   - /vel_raw publica com namespace absoluto '/vel_raw' (X3 usa 'vel_raw' relativo)
#   - print("mag:...) ativo em pub_data (X3 está comentado) — produz muito output em terminal
#   - /vel_raw.linear.y = vy*1000 (igual ao R2, não igual ao X3) — BUG ou diferença intencional?
#   - state.position publicado com ângulo de esterçamento (igual ao R2, não ao X3)
#
# Parâmetros ROS2:
#   car_type, imu_link, Prefix, xlinear_limit, ylinear_limit, angular_limit (=1.0), nav_use_rotvel
#
# Relevância para robodog2: baixa — robodog2 usa X3.
# Pode ser útil como referência para verificar diferenças de escala entre modelos.

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

# Mapeamento de tipo de robô para código de hardware
car_type_dic={
    'R2':5,    # Ackermann
    'X1':4,    # Mecanum pequeno — este ficheiro
    'X3':1,    # Mecanum grande
    'NONE':-1
}
class yahboomcar_driver(Node):
	def __init__(self, name):
		super().__init__(name)
		global car_type_dic
		self.RA2DE = 180 / pi
		self.car = Rosmaster()
		self.car.set_car_type(4)  # modo X1 no firmware Arduino (vs 1 para X3)
		#get parameter
		self.declare_parameter('car_type', 'X1')
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
		self.declare_parameter('angular_limit', 1.0)  # X1: limite angular menor que X3 (5.0)
		self.angular_limit = self.get_parameter('angular_limit').get_parameter_value().double_value
		print (self.angular_limit)
		self.declare_parameter('nav_use_rotvel', False)  # flag extra, não implementado
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
		# NOTA: namespace absoluto '/vel_raw' vs X3 que usa relativo 'vel_raw'
		self.velPublisher = self.create_publisher(Twist,"/vel_raw",50)
		# 这里直接发布imu_filter_madgwick功能包订阅的imu topic，之后直接输入到imu_filter_madgwick中进行imu的数据滤波
		# Publica diretamente no tópico subscrito pelo imu_filter_madgwick
		self.imuPublisher = self.create_publisher(Imu,"/imu/data_raw",100)
		self.magPublisher = self.create_publisher(MagneticField,"/imu/mag",100)

		# timer a 10Hz para publicar todos os dados de sensores
		self.timer = self.create_timer(0.1, self.pub_data)

		#create and init variable
		self.edition = Float32()
		self.edition.data = 1.0
		# inicia thread de receção serial em background
		self.car.create_receive_threading()

	#callback function
	def cmd_vel_callback(self,msg):
        # 小车运动控制，订阅者回调函数
        # Car motion control, subscriber callback function
		if not isinstance(msg, Twist): return
        # 下发线速度和角速度
        # Issue linear vel and angular vel
		vx = msg.linear.x
        #vy = msg.linear.y/1000.0*180.0/3.1416    #Radian system
		vy = msg.linear.y
		angular = msg.angular.z     # wait for change
		# self.get_logger().info("vx = {}, vy = {}, angular= {}".format(vx,vy,angular))
		self.car.set_car_motion(vx, vy, angular)
        #rospy.loginfo("nav_use_rot:{}".format(self.nav_use_rotvel))
        #print(self.nav_use_rotvel)

	def RGBLightcallback(self,msg):
        # 流水灯控制，服务端回调函数 RGBLight control
		if not isinstance(msg, Int32): return
		# print ("RGBLight: ", msg.data)
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

		# AVISO: print ativo aqui — gera muito output em terminal (comentar em produção)
		print ("mag: ",self.car.get_magnetometer_data())
		edition.data = self.car.get_version()
		battery.data = self.car.get_battery_voltage()
		ax, ay, az = self.car.get_accelerometer_data()
		gx, gy, gz = self.car.get_gyroscope_data()
		mx, my, mz = self.car.get_magnetometer_data()
		mx = mx * 1.0
		my = my * 1.0
		mz = mz * 1.0
		vx, vy, angular = self.car.get_motion_data()

		# 发布陀螺仪的数据
		# Publish gyroscope data
		imu.header.stamp = time_stamp.to_msg()
		imu.header.frame_id = self.imu_link
		imu.linear_acceleration.x = ax
		imu.linear_acceleration.y = ay
		imu.linear_acceleration.z = az
		imu.angular_velocity.x = gx
		imu.angular_velocity.y = gy
		imu.angular_velocity.z = gz

		mag.header.stamp = time_stamp.to_msg()
		mag.header.frame_id = self.imu_link
		mag.magnetic_field.x = mx
		mag.magnetic_field.y = my
		mag.magnetic_field.z = mz

		# 将小车当前的线速度和角速度发布出去
		# Publish the current linear vel and angular vel of the car
		twist.linear.x = vx    #velocity in axis
		# NOTA: X1 escala vy como o R2 (×1000), não como o X3 (×1.0)
		# Possível porque X1 usa base_node_x1 que espera a mesma escala do R2
		twist.linear.y = vy*1000   #steer angle (em millirad, como o R2)
		#twist.linear.y = vy   #steer angle
		#twist.angular.z = angular
		twist.angular.z = angular    #this is invalued
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

		# Converte ângulo de esterçamento (millirad) para posição dos joints
		steer_radis = vy*1000.0*3.1416/180.0
		state.position = [0.0, 0.0, steer_radis, 0.0, steer_radis, 0.0]
		if not vx == angular == 0:
			# animação de rotação das rodas para o RViz2
			i = random.uniform(-3.14, 3.14)
			state.position = [i, i, steer_radis, i, steer_radis, i]
		self.staPublisher.publish(state)

def main():
	rclpy.init()
	driver = yahboomcar_driver('driver_node')
	rclpy.spin(driver)

'''if __name__ == '__main__':
	main()'''



