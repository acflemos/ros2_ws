#!/usr/bin/env python
# encoding: utf-8
# pub_image.py — Pipe de redimensionamento e republicação de imagem
# =================================================================
# Nó ROS2 simples que subscreve imagem bruta, redimensiona para 640×480 e
# republica no tópico /image. Atua como adaptador de resolução no pipeline
# de visão, normalizando a resolução antes de nós de processamento posteriores.
#
# Subscreve: /image_raw  (sensor_msgs/Image, bgr8) — imagem original
# Publica:   /image      (sensor_msgs/Image, bgr8) — imagem 640×480
#
# Limitações: sem exibição local (apenas pub/sub); queue_size=500 pode consumir
#             memória excessiva em sistemas com restrição de RAM (ex.: Jetson Nano).
# Relevância para robodog2: intermediário útil se a câmara publicar resolução
#             variável; para câmara Astra preferir subscrever diretamente
#             /camera/color/image_raw que já publica 640×480.

import rclpy
from rclpy.node import Node
import cv2 as cv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image



class PubImage(Node):
	def __init__(self,name):
		super().__init__(name)
		self.bridge = CvBridge()
		self.sub_img = self.create_subscription(Image,'/image_raw',self.handleTopic,500)
		self.pub_img = self.create_publisher(Image,'/image',500)


	def handleTopic(self,msg):
		if not isinstance(msg, Image):
			return
		frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
		# 规范输入图像大小
		# Standardize the input image size
		frame = cv.resize(frame, (640, 480))
		# opencv mat ->  ros msg
		msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
		self.pub_img.publish(msg)


def main():
	rclpy.init()
	pub_image = PubImage('pub_image_node')
	rclpy.spin(pub_image)

