# setup.py — yahboomcar_laser
# ============================
# Configuração do pacote ROS2 ament_python para aplicações de LiDAR do ROSMaster.
#
# Executáveis registados (entry_points):
#   laser_Avoidance_a1_X3   — desvio de obstáculos com RPLIDAR A1 (X3)
#   laser_Avoidance_a1_R2   — desvio de obstáculos com RPLIDAR A1 (R2)
#   laser_Avoidance_4ROS_R2 — desvio de obstáculos com YDLIDAR X4 (R2)
#   laser_Tracker_a1_X3     — seguidor de obstáculos com RPLIDAR A1 (X3)
#   laser_Tracker_a1_R2     — seguidor de obstáculos com RPLIDAR A1 (R2)
#   laser_Tracker_4ROS_R2   — seguidor de obstáculos com YDLIDAR X4 (R2) — versão corrigida
#   laser_Warning_a1_X3     — aviso sonoro + rotação com RPLIDAR A1 (X3)
#
# AUSENTES dos entry_points (ficheiros existem mas não registados):
#   laser_Avoidance_4ROS.py — desvio X3 com YDLIDAR X4 (sem entry point)
#   laser_Tracker_4ROS.py   — seguidor X3 com YDLIDAR X4 (sem entry point + BUG self.laserAngle)
#
# Launch files instalados: launch/*launch.py (Avoidance, Tracker, Warning — todos para a1 X3)

from setuptools import setup
import os
from glob import glob

package_name = 'yahboomcar_laser'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),glob(os.path.join('launch','*launch.py')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nx-ros2',
    maintainer_email='nx-ros2@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'laser_Avoidance_a1_X3 = yahboomcar_laser.laser_Avoidance_a1_X3:main',
            'laser_Avoidance_4ROS_R2 = yahboomcar_laser.laser_Avoidance_4ROS_R2:main',
            'laser_Avoidance_a1_R2 = yahboomcar_laser.laser_Avoidance_a1_R2:main',
            'laser_Tracker_a1_X3 = yahboomcar_laser.laser_Tracker_a1_X3:main',
            'laser_Tracker_4ROS_R2 = yahboomcar_laser.laser_Tracker_4ROS_R2:main',
            'laser_Tracker_a1_R2 = yahboomcar_laser.laser_Tracker_a1_R2:main',
            'laser_Warning_a1_X3 = yahboomcar_laser.laser_Warning_a1_X3:main',
        ],
    },
)
