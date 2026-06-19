-- lds_2d.lua
-- ===========
-- Configuração do Google Cartographer para SLAM 2D com LiDAR único.
-- Usado por: cartographer_launch.py (via map_cartographer_launch.py)
--
-- Modo: 2D apenas (MAP_BUILDER.use_trajectory_builder_2d = true)
-- Tracking frame: base_footprint (frame que o Cartographer estima a pose)
-- Published frame: odom (frame do Cartographer para publicar o TF)
-- provide_odom_frame: false — odometria vem do EKF (/odom), não é gerada aqui
-- use_odometry: true — Cartographer usa /odom para aided odometry
-- use_imu_data: false — sem dados IMU (use_odometry já fornece constraining suficiente)
--
-- LiDAR:
--   num_laser_scans: 1 (um LiDAR 2D)
--   max_range: 8m (adequado para YDLIDAR X4 e RPLIDAR A1/S2)
--   missing_data_ray_length: 0.5 (raios sem retorno = 0.5m, não max_range)
--   use_online_correlative_scan_matching: true (melhor qualidade, mais CPU)
--
-- Qualidade do mapa:
--   constraint_builder.min_score: 0.7 (threshold alto — loop closure conservador)
--   global_localization_min_score: 0.7
--   motion_filter.max_angle_radians: ~0.1° (publicar só quando há movimento angular significativo)
--
-- Relevância para robodog2:
--   Reutilizável diretamente — só mudar tracking_frame e published_frame se necessário.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_footprint",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = false,
  use_odometry = true, 
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 8
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 0.5
TRAJECTORY_BUILDER_2D.use_imu_data = false 
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true 
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)

POSE_GRAPH.constraint_builder.min_score = 0.7
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7

--POSE_GRAPH.optimize_every_n_nodes = 30

return options
