# easyusvdata

Sensor bringup for USV sensor evaluation on a Raspberry Pi 5 (Ubuntu 24.04, ROS 2 Jazzy).

`ros2_ws/src/usv_bringup` launches existing drivers:

| Sensor | Driver | Topics |
|---|---|---|
| GPS (UART, `/dev/ttyAMA0`, 9600) | `nmea_navsat_driver` / `nmea_serial_driver` | `/gps/fix`, `/gps/vel`, `/gps/time_reference`, `/gps/heading` |
| BNO055 (I2C on GPIO6/7) | `bno055` | `/bno055/imu`, `/bno055/imu_raw`, `/bno055/mag`, `/bno055/grav`, `/bno055/temp`, `/bno055/calib_status` |
| (diagnostic) | `usv_bringup/imu_euler` | `/bno055/euler_deg` (x=roll, y=pitch, z=yaw, degrees) |

## Setup (once, on the Pi)

```bash
sudo apt install ros-jazzy-nmea-navsat-driver ros-jazzy-bno055 i2c-tools
```

In `/boot/firmware/config.txt`:
```
dtparam=uart0=on
dtoverlay=i2c3-pi5      # I2C3 on GPIO6 (SDA) / GPIO7 (SCL)
```
Reboot, then check the bus number and that the BNO055 shows up at 0x28:
```bash
i2cdetect -l
i2cdetect -y 3
```
Set `i2c_bus` in `config/bno055.yaml` if it isn't 3.

## Build and run

```bash
cd ros2_ws
colcon build --packages-select usv_bringup
source install/setup.bash
ros2 launch usv_bringup sensors.launch.py
```

IMU on its own:
```bash
ros2 run bno055 bno055 --ros-args --params-file install/usv_bringup/share/usv_bringup/config/bno055.yaml
ros2 run usv_bringup imu_euler     # optional, in a second terminal
```

## Localization (robot_localization)

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
