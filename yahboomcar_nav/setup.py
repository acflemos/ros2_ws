# setup.py — yahboomcar_nav
# ==========================
# Configuração do pacote ROS2 ament_python para navegação e SLAM do ROSMaster.
#
# Executáveis registados:
#   scan_filter — downsampler de LaserScan (1/2 pontos) para compatibilidade com gmapping
#
# Recursos instalados:
#   launch/  — todos os *launch.py (gmapping, cartographer, Nav2 DWB/TEB/RTAB-Map)
#   rviz/    — configurações RViz2 (map.rviz, nav.rviz, rtabmap_*.rviz)
#   params/  — dwa_nav_params.yaml, teb_nav_params.yaml, rtabmap_nav_params.yaml, lds_2d.lua
#   maps/    — yahboomcar.yaml + yahboomcar.pgm (mapa de exemplo)

from setuptools import setup
import os
from glob import glob

package_name = 'yahboomcar_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),glob(os.path.join('launch','*launch.py'))),
        (os.path.join('share',package_name,'rviz'),glob(os.path.join('rviz','*.rviz*'))),
        (os.path.join('share', package_name, 'params'), glob(os.path.join('params', '*.*'))),
        (os.path.join('share', package_name, 'maps'), glob(os.path.join('maps', '*.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='1461190907@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scan_filter = yahboomcar_nav.scan_filter:main'
        ],
    },
)
