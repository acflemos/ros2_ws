#!/usr/bin/env python
# encoding: utf-8
# astra_color_point.py — Sincronização RGB + Depth com ApproximateTimeSynchronizer
# ==================================================================================
# Nó ROS2 que subscreve simultaneamente os tópicos de imagem RGB e profundidade da
# câmara Astra, sincronizando-os com tolerância de 0.5 s via message_filters, e
# exibe ambas as janelas com cv.imshow para depuração visual.
#
# Subscreve: /camera/color/image_raw  (sensor_msgs/Image, bgr8)    — imagem RGB
#            /camera/depth/image_raw  (sensor_msgs/Image, 32FC1)   — imagem de profundidade
#
# Limitações: apenas exibe imagens localmente (sem publicação); sem frame_id no sync.
#             Janela cv.imshow requer display gráfico (não funciona sem DISPLAY).
# Relevância para robodog2: ponto de partida para fusão RGB+Depth na Jetson Nano;
#             adaptar publicando PointCloud2 com cores para detecção de obstáculos 3D.

import rclpy
from rclpy.node import Node
import cv2 as cv
import message_filters
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class PointCloud(Node):
    def __init__(self,name):
        super().__init__(name)
        self.rgb_img = None
        self.depth_img = None
        self.bridge = CvBridge()
        # one callback that deals with depth and rgb at the same time
        
        self.im_sub = message_filters.Subscriber(self,Image,'/camera/color/image_raw')
        self.dep_sub = message_filters.Subscriber(self,Image,'/camera/depth/image_raw')
        self.timeSynchronizer = message_filters.ApproximateTimeSynchronizer([self.im_sub, self.dep_sub], 1, 0.5)
        self.timeSynchronizer.registerCallback(self.trackObject)

    def trackObject(self, image_data, depth_data):
        if not isinstance(image_data, Image): return
        if not isinstance(depth_data, Image): return
        # convert both images to numpy arrays
        frame = self.bridge.imgmsg_to_cv2(image_data, 'bgr8')
        depthFrame = self.bridge.imgmsg_to_cv2(depth_data, 'passthrough')  # "32FC1")
        cv.imshow("frame", frame)
        cv.imshow("depthFrame", depthFrame)
        cv.waitKey(10)

def main():
	rclpy.init()
	pointcloud = PointCloud('pub_point_cloud')
	print("start it")
	rclpy.spin(pointcloud)



