include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  
  -- Navigasyon için standart olan 'base_footprint'tir.
  -- Robotun yerdeki izdüşümünü temsil eder. Bunu değiştirmeyelim.
  tracking_frame = "base_footprint",
  
  -- Cartographer sadece map->odom yayınlasın (Yetki karmaşası olmasın)
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false, -- Odom'u robotun sürücüsü basıyor, biz basmayalım.
  
  publish_frame_projected_to_2d = false,
  
  -- Robotun tekerlek verisini KULLAN (Çünkü içinde IMU füzyonu var)
  use_odometry = true,
  
  use_nav_sat = false,
  use_landmarks = false,
  
  -- Lidar Ayarları
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  
  -- ZAMANLAMA AYARLARI (Extrapolation Hatası Çözümü)
  -- Robotun veriyi beklemesi için toleransı arttırdık.
  lookup_transform_timeout_sec = 1.0,
  
  -- Haritayı saniyede 2 kere yayınla (İşlemciyi yormasın)
  submap_publish_period_sec = 0.5,
  
  -- Konumu saniyede 100 kere yayınla (Gecikmeyi yensin)
  pose_publish_period_sec = 20e-3,
  
  -- Görselleştirmeyi yavaşlat
  trajectory_publish_period_sec = 100e-3,
  
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- Harita Optimizasyon Sıklığı (Jetson için düşürdük)
POSE_GRAPH.optimize_every_n_nodes = 90
POSE_GRAPH.constraint_builder.min_score = 0.65 -- Biraz daha esnek olsun
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3

-- --- KRİTİK AYARLAR ---

-- 1. IMU KAPALI: Çünkü Odom mesajının içinde zaten var.
TRAJECTORY_BUILDER_2D.use_imu_data = false

-- 2. TARAMA EŞLEŞTİRME AÇIK: IMU kapalı olduğu için robot yönünü
--    duvarlara bakarak düzeltsin.
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

-- Lidar Menzili
TRAJECTORY_BUILDER_2D.min_range = 0.3
TRAJECTORY_BUILDER_2D.max_range = 8.0 -- RPLidar için 8m iyidir (6.5 az olabilir)
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0

-- Eşleştirme Ağırlıkları (Lidar verisine daha çok güvensin)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.0 -- Dönüş düzeltmesi önemli

-- Hareket Filtresi (Gereksiz güncellemeleri önle)
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.1
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.5)

return options
