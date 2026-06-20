#!/usr/bin/env python
# encoding: utf-8
# laser_to_image.py — Conversão de LaserScan em imagem bird's-eye view
# =====================================================================
# Nó ROS2 que recebe dados do LIDAR (LaserScan), converte para PointCloud2 via
# LaserProjection local, e gera uma imagem top-down (1600×1200 px) onde cada
# ponto laser é desenhado com intensidade proporcional à coordenada Z.
# Escala: 80 px/m, origem no centro (500, 500). Publica a imagem resultante
# e exibe uma versão redimensionada (640×480) localmente.
#
# Subscreve: /scan        (sensor_msgs/LaserScan)  — dados do LIDAR 2D
# Publica:   /laserImage  (sensor_msgs/Image)       — imagem bird's-eye uint8
#
# Limitações: imagem muito grande (1600×1200) para uso em tempo real restrito;
#             LaserProjection é uma cópia local (laser_geometry.py) não o pacote ROS2.
#             Intensidade Z esperada entre -2 m e +2 m (LIDAR montado na horizontal).
# Relevância para robodog2: útil para visualizar cobertura do LIDAR em testes;
#             para navegação usar Nav2 costmap diretamente em vez desta imagem.

import rclpy
from rclpy.node import Node
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import LaserScan, Image
from .laser_geometry import LaserProjection
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2 as pc2 

class pt2brid_eye(Node):
    def __init__(self,name):
    	super().__init__(name)
    	self.bridge = CvBridge()
    	self.laserProj = LaserProjection()
    	self.laserSub = self.create_subscription(LaserScan,"/scan",self.laserCallback,100)  # 接收scan节点  Receiving scan Nodes
    	self.image_pub = self.create_publisher(Image,'/laserImage',1)

    def laserCallback(self, scan_data):
    	cloud_out = self.laserProj.projectLaser(scan_data)
    	lidar = pc2.read_points(cloud_out)
    	points = np.array(list(lidar))
    	img = self.pointcloud_to_laserImage(points)
    	self.image_pub.publish(self.bridge.cv2_to_imgmsg(img))
    	img = cv.resize(img, (640, 480))
    	cv.imshow("img", img)
    	cv.waitKey(10)

    def pointcloud_to_laserImage(self, points):  # 鸟瞰图生成  Aerial view generated
        x_points = points[:, 0]
        y_points = points[:, 1]
        z_points = points[:, 2]
        f_filt = np.logical_and((x_points > -50), (x_points < 50))
        s_filt = np.logical_and((y_points > -50), (y_points < 50))
        filter = np.logical_and(f_filt, s_filt)
        indices = np.argwhere(filter)
        x_points = x_points[indices]
        y_points = y_points[indices]
        z_points = z_points[indices]
        x_img = (-y_points * 80).astype(np.int32) + 500
        y_img = (-x_points * 80).astype(np.int32) + 500
        pixel_values = np.clip(z_points, -2, 2)
        pixel_values = ((pixel_values + 2) / 4.0) * 500
        img = np.zeros([1600, 1200], dtype=np.uint8)
        img[y_img, x_img] = pixel_values
        return img


def main():
    print("opencv: {}".format(cv.__version__))
    rclpy.init()
    pt2img = pt2brid_eye('laser_to_image')
    rclpy.spin(pt2img)
    
    
    
    
    
    
