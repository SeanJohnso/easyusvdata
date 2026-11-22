# easyusvdata

Sensor bringup for USV sensor evaluation on a Raspberry Pi 5 (Ubuntu 24.04, ROS 2 Jazzy).

`ros2_ws/src/usv_bringup` runs the module test rig: every GPS and IMU module at once,
listed in `config/rig.yaml` (ports, buses, addresses, mounting poses).

| Modules | Driver | Topics |
|---|---|---|
| 5 u-blox GPS (UART) | `ublox_gps` (one node per module) | `/gps/<id>/fix`, `/gps/<id>/vel`, `/gps/<id>/ublox_gps_node/navpvt`, `/gps/<id>/navstate` |
| BNO055 | `bno055` | `/imu/bno055/imu` (fused), `/imu/bno055/imu_raw`, `/imu/bno055/mag`, ... |
| ICM-20948 | `icm20948_ros2` | `/imu/icm20948/imu`, `/imu/icm20948/mag` |
| MPU9250 | `usv_bringup/mpu9250` | `/imu/mpu9250/imu`, `/mag`, `/temp` |
| ISM330DHCX + MMC5983MA / LIS3MDL | `usv_bringup/ism330dhcx` + `mmc5983ma` / `lis3mdl` | `/imu/<id>/imu`, `/imu/<id>/temp`, `/imu/<id>/mag` |

Frames: `gps_<id>`, `imu_<id>`, and `imu_<id>_mag` for separate magnetometer chips, each a
static TF from `base_link` with the pose in `rig.yaml`.

## Setup (once, on the Pi)

```bash
sudo apt install ros-jazzy-ublox-gps ros-jazzy-bno055 python3-smbus2 i2c-tools
```
`icm20948_ros2` is built from source in `ros2_ws/src` (see CLAUDE.md, IMU section).

In `/boot/firmware/config.txt` (check pins and names in `/boot/firmware/overlays/README`):
```
dtparam=uart0=on            # GPIO14/15 -> /dev/ttyAMA0
dtoverlay=uart1-pi5         # GPIO0/1
dtoverlay=uart2-pi5         # GPIO4/5
dtoverlay=uart3-pi5         # GPIO8/9
dtoverlay=uart4-pi5         # GPIO12/13
dtparam=i2c_arm=on          # bus 1, GPIO2/3
dtparam=i2c_arm_baudrate=400000
dtoverlay=i2c3-pi5          # bus 3, GPIO6/7
```
Reboot, then check the device names and addresses against `rig.yaml`:
```bash
ls -l /dev/ttyAMA*
i2cdetect -y 1
i2cdetect -y 3
```

## Build and run

```bash
cd ros2_ws
colcon build --packages-select usv_bringup
source install/setup.bash
ros2 launch usv_bringup rig.launch.py
ros2 run usv_bringup check          # second terminal: status table every 2 s
```

`check` columns: `driver` OK / STALE (nothing for 2 s) / NO DATA, message `rate`, GPS `fix`,
and `pi-gps` = Pi clock minus the GPS-time stamp. A few hundredths of a second is
normal latency; seconds or more means the Pi clock is wrong (fix it before recording).

Record a test:
```bash
ros2 bag record -o ~/data/<static|dynamic>_$(date -u +%Y%m%dT%H%M%SZ) -e '/(gps|imu)/.*' /tf_static /mark
```

## Localization (robot_localization, old single-module setup)

`localization.launch.py` still uses the old `/bno055/imu` and `/gps/fix` topics; it will be
replaced by the dynamic-test replay.

Run alongside `sensors.launch.py`:
```bash
sudo apt install ros-jazzy-robot-localization
ros2 launch usv_bringup localization.launch.py
```

| Output | Topic | Type |
|---|---|---|
| IMU dead reckoning | `/odometry/local` | `nav_msgs/Odometry` (x/y metres, `odom` frame) |
| GPS positions | `/gps/fix` (raw lat/lon), `/odometry/gps` (x/y metres, `map` frame) | `NavSatFix`, `Odometry` |
| EKF (IMU + GPS) | `/odometry/global` (x/y metres, `map` frame), `/gps/filtered` (lat/lon) | `Odometry`, `NavSatFix` |

Record a run:
```bash
ros2 bag record -o ../data/run_$(date -u +%Y%m%dT%H%M%SZ) \
  /odometry/local /odometry/global /odometry/gps /gps/fix /gps/filtered \
  /bno055/imu /bno055/calib_status /tf /tf_static
```

Copy bags to the dev machine (run on the dev machine; wired link, use the Wi-Fi address if it's down):
```bash
rsync -avP bird@192.168.50.2:data/ ~/easyusvdata/postprocessing/data/
```
Keep the trailing `/` on both paths, or rsync nests the folder inside `data/`. Re-running only copies new bags.

Process and view the maps (dev machine):
```bash
~/easyusvdata/postprocessing/run.sh
cd ~/easyusvdata/postprocessing/output/<bag>
google-chrome map_global.html map_local.html map_velocity.html
```

Settings: magnetic declination 7.27° W (`-0.1269` rad) and `yaw_offset: 0` (IMU yaw 0 = east), in `config/navsat.yaml`.
Sensor mounting offsets: `BNO055` and `GPS` at the top of `launch/localization.launch.py` (rebuild after editing).
