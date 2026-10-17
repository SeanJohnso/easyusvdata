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
