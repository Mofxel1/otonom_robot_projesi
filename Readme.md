# Otonom Robot Projesi

TurtleBot3 Waffle Pi tabanlı, Jetson Nano üzerinde çalışan tam otonom haritalama ve navigasyon sistemi. Robot önce çevresini keşfederek harita çıkarır, ardından kullanıcı arayüzü üzerinden yönetilebilir hale gelir.

---

## Donanım

| Bileşen | Model |
|---|---|
| Robot | TurtleBot3 Waffle Pi (Kobuki tabanlı) |
| Bilgisayar | NVIDIA Jetson Nano |
| Lidar | RPLidar |
| Sürücü | Kobuki / cmd_vel_mux |

---

## Sistem Mimarisi

```
┌─────────────────────────────────────────────────────────┐
│                    jetson_otonom_final.launch            │
│                                                         │
│  ┌──────────────┐  ┌──────────────┐  ┌───────────────┐ │
│  │ cartographer │  │  move_base   │  │   boundary    │ │
│  │     SLAM     │  │  (navigate)  │  │   explorer    │ │
│  └──────┬───────┘  └──────┬───────┘  └───────┬───────┘ │
│         │/map             │                  │         │
│         └────────┬────────┘                  │         │
│                  │              ┌─────────────┘         │
│         ┌────────▼─────────┐   │/explorer_enabled      │
│         │  mission_control │◄──┘                       │
│         │  (görev yöneticisi)                           │
│         └────────┬─────────┘                           │
│                  │ (harita kaydedilir, GUI açılır)      │
│         ┌────────▼─────────┐                           │
│         │    robot_gui     │                           │
│         │  (PyQt5 arayüzü) │                           │
│         └──────────────────┘                           │
└─────────────────────────────────────────────────────────┘
```

---

## Çalışma Akışı

**Faz 1 — Haritalama**

`mission_control` başlar başlamaz `boundary_explorer_v2`'yi aktive eder. Explorer, duvar takibi (PID kontrollü) yaparak odayı/alanı dolaşır ve Cartographer SLAM `/map` topic'ine haritayı yayınlar. Robot başladığı noktaya döndüğünde tur tamamlanmış sayılır, explorer durdurulur.

**Faz 2 — Kayıt**

`map_saver` ile harita `maps/otonom_harita.pgm` ve `maps/otonom_harita.yaml` olarak diske kaydedilir.

**Faz 3 — GUI ve Navigasyon**

`robot_gui.py` açılır. Kullanıcı harita üzerinde:
- Tıklayarak navigasyon hedefi verebilir
- Yasaklı bölge çizebilir (robot bu alanlara girmez)
- Hız bölgesi çizebilir (robot bu alanlarda yavaşlar)

---

## Modüller

### `mission_control.py`
Tüm sistemi yöneten ana düzenleyici. Faz 1 → Faz 2 → Faz 3 geçişlerini yönetir.

### `boundary_explorer_v2.py`
PID tabanlı duvar takip algoritması. Duvardan belirli mesafede (`desired_distance_wall: 0.75m`) ilerler, köşeleri algılayıp döner. `/explorer_enabled` topic'i ile aktive/pasife alınır.

### `smart_navigator.py`
Move Base üzerinde çalışan navigasyon katmanı. Ek özellikleri:
- Önünde dar geçit varsa bekler, açılmazsa bypass eder
- Hız bölgelerine girilince `dynamic_reconfigure` ile hızı düşürür (`max_vel_x: 0.05 m/s`)
- Çıkınca normal hıza (`0.18 m/s`) geri döner

### `robot_gui.py`
PyQt5 tabanlı kontrol arayüzü. Özellikler:
- Kaydedilen pgm haritasını gösterir
- TF üzerinden robotun anlık konumunu ve yön okunu haritada gösterir
- Tıkla-git navigasyonu
- Yasaklı bölge çizimi (costmap'e `lethal: 100` olarak işlenir)
- Hız bölgesi çizimi (SmartNavigator'a iletilir)
- Zone'lar `maps/zones.yaml`'a kaydedilir, yeniden başlatmada yüklenir

### `map_processor.py`
Zone verilerini yönetir. `add_forbidden_zone`, `add_speed_zone`, `clear_all_zones` metodları. Veriler `zones.yaml`'a persist edilir.

---

## Costmap Yapısı

**Global Costmap**

```
gui_layer (StaticLayer) → /gui_zones topic'i
    ↓ merge (harita + zone'lar birleşik)
inflation_layer
```

`robot_gui.py` pgm haritasını okuyup üzerine zone'ları işleyerek `/gui_zones` topic'ine basar. Global costmap sadece bu tek layer'ı dinler.

**Local Costmap**

```
obstacle_layer → /scan (RPLidar)
gui_layer      → /gui_zones (yasaklı bölgeler)
inflation_layer
```

---

## Kurulum

```bash
cd ~/catkin_ws/src
git clone <repo_url> otonom_robot
cd ~/catkin_ws
catkin_make
```

Python bağımlılıkları:

```bash
pip3 install PyQt5 opencv-python pyyaml numpy
```

ROS bağımlılıkları:

```bash
sudo apt install ros-noetic-cartographer-ros \
                 ros-noetic-move-base \
                 ros-noetic-tf2-ros \
                 ros-noetic-rplidar-ros \
                 ros-noetic-turtlebot-bringup
```

---

## Başlatma

```bash
roslaunch otonom_robot jetson_otonom_final.launch
```

Robot otomatik olarak haritalama fazına girer. Tur tamamlanınca GUI açılır.

Manuel haritalama (test için):

```bash
roslaunch otonom_robot manuel_haritalama.launch
```

---

## Konfigürasyon

| Dosya | İçerik |
|---|---|
| `config/cartographer_config.lua` | SLAM parametreleri |
| `config/global_costmap_params.yaml` | Global costmap, lethal_cost_threshold: 80 |
| `config/local_costmap_params.yaml` | Local costmap, 4x4m kayan pencere |
| `config/base_local_planner_params.yaml` | TrajectoryPlannerROS parametreleri |

Önemli parametreler:

```yaml
# Engel etrafı güvenli mesafe
inflation_radius: 0.55
cost_scaling_factor: 3.0

# Hız bölgesinde yavaşlama (smart_navigator.py)
SLOW_SPEED: 0.18   # m/s — çok düşük ayarlanırsa TF timeout olur

# Costmap engel eşiği
lethal_cost_threshold: 80  # Cartographer 100 yerine 90-99 değerleri üretir
```

---

## Topic Haritası

| Topic | Tip | Açıklama |
|---|---|---|
| `/map` | OccupancyGrid | Cartographer canlı harita |
| `/gui_zones` | OccupancyGrid | Harita + GUI zone'ları birleşik |
| `/explorer_enabled` | Bool | Duvar takibini aç/kapat |
| `/boundary_explorer_node/state` | String | Explorer durumu |
| `/move_base/goal` | MoveBaseActionGoal | Navigasyon hedefi |
| `/cmd_vel_mux/input/navi` | Twist | Navigasyon hız komutu |
| `/cmd_vel_mux/input/teleop` | Twist | Explorer hız komutu |
| `/scan` | LaserScan | RPLidar ham verisi |

---

## Bilinen Sınırlamalar

- Kobuki düşük hızlarda (`< 0.15 m/s`) TF timeout verebiliyor; `SLOW_SPEED` bu değerin altına çekilmemeli
- Harita GUI açıldıktan sonra güncellenmez; Cartographer devam etse bile `gui_zones` kayıtlı pgm'den beslenir
- Zone'lar kaydedilip yükleniyor ama oturum arası navigasyon hedefleri saklanmıyor
