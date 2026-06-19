#!/usr/bin/env python
# coding:utf-8
# scan_filter.py
# ===============
# Nó de downsampling de LaserScan para compatibilidade com gmapping.
#
# gmapping só aceita frames com ≤1440 pontos por varrimento 2D.
# Os LiDARs YDLIDAR X4 (4ROS) e RPLIDAR S2 geram >1440 pontos por frame.
# Este nó reduz a densidade em 50% (mantém 1 de cada 2 pontos) para cumprir esse limite.
#
# Subscreve: /scan           (LaserScan original do driver)
# Publica:   /downsampled_scan (LaserScan com metade dos pontos)
#
# self.multiple = 2 → mantém pontos nos índices 0, 2, 4, 6, ...
# angle_increment é duplicado para refletir o maior espaçamento angular.
#
# Nota: em ROS2 Humble, prefira slam_toolbox (sem limite de pontos).
# Este nó só é necessário se usar gmapping com LiDARs de alta densidade.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
from rclpy.clock import Clock


class scan_compression(Node):
    def __init__(self,name):
        super().__init__(name)
        self.multiple = 2  # fator de downsampling: mantém 1 de cada 2 pontos
        self.pub = self.create_publisher(LaserScan, "/downsampled_scan", 1000)
        self.laserSub = self.create_subscription(LaserScan,"/scan", self.laserCallback, 1000)

    def laserCallback(self, data):
        # self.get_logger().info("laserCallback")
        if not isinstance(data, LaserScan): return
        laser_scan = LaserScan()
        laser_scan.header.stamp = Clock().now().to_msg()
        laser_scan.header.frame_id = data.header.frame_id
        laser_scan.angle_increment = data.angle_increment * self.multiple
        laser_scan.time_increment = data.time_increment
        laser_scan.intensities = data.intensities
        laser_scan.scan_time = data.scan_time
        laser_scan.angle_min = data.angle_min
        laser_scan.angle_max = data.angle_max
        laser_scan.range_min = data.range_min
        laser_scan.range_max = data.range_max
        # self.get_logger().info("len(np.array(data.ranges)) = {}".format(len(np.array(data.ranges))))
        for i in range(len(np.array(data.ranges))):
            if i % self.multiple == 0: laser_scan.ranges.append(data.ranges[i])
        self.pub.publish(laser_scan)

def main():
    rclpy.init()
    scan_cp = scan_compression("scan_dilute")
    rclpy.spin(scan_cp)
    scan_cp.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
