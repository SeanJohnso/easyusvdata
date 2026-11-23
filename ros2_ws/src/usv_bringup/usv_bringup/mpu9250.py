"""MPU9250-type board (MPU6500 accel/gyro + AK8963 magnetometer) over I2C.

Publishes raw data only: imu (accel + gyro, no orientation), mag, temp.
Register addresses from the MPU-9250 register map (RM-MPU-9250A-00) and the
AK8963 datasheet. The magnetometer is read directly at 0x0C through the
MPU's I2C bypass, not through its internal I2C master.
"""
import struct
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField, Temperature
from smbus2 import SMBus

# MPU6500 registers
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0x1C
ACCEL_CONFIG2 = 0x1D
INT_PIN_CFG = 0x37
ACCEL_XOUT_H = 0x3B   # 14 bytes: accel xyz, temp, gyro xyz, big-endian
USER_CTRL = 0x6A
PWR_MGMT_1 = 0x6B
PWR_MGMT_2 = 0x6C
WHO_AM_I = 0x75

# AK8963 registers
AK8963_ADDR = 0x0C
AK_WIA = 0x00
AK_ST1 = 0x02         # 8 bytes: ST1, HXL..HZH (little-endian), ST2
AK_CNTL1 = 0x0A
AK_ASAX = 0x10        # 3 bytes, factory sensitivity adjustment (fuse ROM)

G = 9.80665
DEG = 0.017453292519943295
GYRO_LSB_PER_DPS = (131.0, 65.5, 32.8, 16.4)   # gyro_range 0..3


def parse_accel_gyro(buf, accel_range, gyro_range):
    """14 bytes from ACCEL_XOUT_H -> (accel m/s^2, gyro rad/s, temp degC)."""
    ax, ay, az, t, gx, gy, gz = struct.unpack('>7h', bytes(buf))
    a = G / (16384 >> accel_range)
    w = DEG / GYRO_LSB_PER_DPS[gyro_range]
    return (ax * a, ay * a, az * a), (gx * w, gy * w, gz * w), t / 333.87 + 21.0


def parse_mag(buf, asa):
    """8 bytes from AK8963 ST1 -> field in tesla in the accel/gyro frame.

    Returns None if there is no new sample (ST1 DRDY clear) or the sensor
    overflowed (ST2 HOFL set).
    """
    st1, hx, hy, hz, st2 = struct.unpack('<B3hB', bytes(buf))
    if not st1 & 0x01 or st2 & 0x08:
        return None
    # 16-bit output: 0.15 uT/LSB, times the factory adjustment
    hx, hy, hz = (h * 0.15e-6 * ((a - 128) / 256.0 + 1.0) for h, a in zip((hx, hy, hz), asa))
    # Into the accel/gyro axes. The MPU-9250 datasheet mapping (hy, hx, -hz) is wrong
    # for this board (MPU6500 + separate AK8963): on the rig (2026-11-22) it turned the
    # wrong way against the gyro and differed from the ICM-20948 by 180 deg about y.
    return -hy, hx, hz


class Mpu9250(Node):

    def __init__(self):
        super().__init__('mpu9250')
        self.declare_parameter('i2c_bus', 3)
        self.declare_parameter('i2c_addr', 0x68)
        self.declare_parameter('frame_id', 'mpu9250')
        self.declare_parameter('rate', 100.0)
        self.declare_parameter('accel_range', 0)
        self.declare_parameter('gyro_range', 0)
        self.declare_parameter('dlpf_cfg', 3)
        self.declare_parameter('variance_acc', [0.0, 0.0, 0.0])
        self.declare_parameter('variance_angular_vel', [0.0, 0.0, 0.0])
        self.declare_parameter('variance_mag', [0.0, 0.0, 0.0])
        self.p = p = {n: self.get_parameter(n).value for n in (
            'i2c_bus', 'i2c_addr', 'frame_id', 'rate', 'accel_range', 'gyro_range',
            'dlpf_cfg', 'variance_acc', 'variance_angular_vel', 'variance_mag')}
        self.get_logger().info(f'parameters: {p}')

        self.pub_imu = self.create_publisher(Imu, 'imu', 10)
        self.pub_mag = self.create_publisher(MagneticField, 'mag', 10)
        self.pub_temp = self.create_publisher(Temperature, 'temp', 10)
        self.bus = SMBus(p['i2c_bus'])
        self.ready = False
        self.asa = None
        self.create_timer(1.0 / p['rate'], self.tick)

    def setup(self):
        """Reset and configure both chips. Raises OSError if the board doesn't answer."""
        b, a, p = self.bus, self.p['i2c_addr'], self.p
        b.write_byte_data(a, PWR_MGMT_1, 0x80)        # device reset
        time.sleep(0.1)
        b.write_byte_data(a, PWR_MGMT_1, 0x01)        # wake, PLL clock
        b.write_byte_data(a, PWR_MGMT_2, 0x00)        # all axes on
        b.write_byte_data(a, SMPLRT_DIV, 0)           # 1 kHz internal rate, polled at `rate`
        b.write_byte_data(a, CONFIG, p['dlpf_cfg'])   # gyro + temp low-pass
        b.write_byte_data(a, GYRO_CONFIG, p['gyro_range'] << 3)
        b.write_byte_data(a, ACCEL_CONFIG, p['accel_range'] << 3)
        b.write_byte_data(a, ACCEL_CONFIG2, p['dlpf_cfg'])  # accel low-pass
        b.write_byte_data(a, USER_CTRL, 0x00)         # internal I2C master off
        b.write_byte_data(a, INT_PIN_CFG, 0x02)       # bypass: AK8963 on the Pi's bus
        who = b.read_byte_data(a, WHO_AM_I)

        wia = b.read_byte_data(AK8963_ADDR, AK_WIA)
        b.write_byte_data(AK8963_ADDR, AK_CNTL1, 0x00)  # power down
        time.sleep(0.01)
        b.write_byte_data(AK8963_ADDR, AK_CNTL1, 0x0F)  # fuse ROM access
        time.sleep(0.01)
        self.asa = b.read_i2c_block_data(AK8963_ADDR, AK_ASAX, 3)
        b.write_byte_data(AK8963_ADDR, AK_CNTL1, 0x00)
        time.sleep(0.01)
        b.write_byte_data(AK8963_ADDR, AK_CNTL1, 0x16)  # 16-bit, continuous 100 Hz
        self.get_logger().info(
            f'WHO_AM_I 0x{who:02X}, AK8963 WIA 0x{wia:02X}, ASA {self.asa}')

    def tick(self):
        try:
            if not self.ready:
                self.setup()
                self.ready = True
            raw = self.bus.read_i2c_block_data(self.p['i2c_addr'], ACCEL_XOUT_H, 14)
            raw_mag = self.bus.read_i2c_block_data(AK8963_ADDR, AK_ST1, 8)
        except OSError as e:
            # Unplugged or bus error: set the board up again on the next tick.
            self.ready = False
            self.get_logger().warn(f'I2C error: {e}', throttle_duration_sec=5.0)
            return
        stamp = self.get_clock().now().to_msg()
        frame = self.p['frame_id']
        acc, gyro, temp = parse_accel_gyro(raw, self.p['accel_range'], self.p['gyro_range'])

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = frame
        imu.orientation_covariance[0] = -1.0   # no orientation
        imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = acc
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = gyro
        for i in range(3):
            imu.linear_acceleration_covariance[4 * i] = self.p['variance_acc'][i]
            imu.angular_velocity_covariance[4 * i] = self.p['variance_angular_vel'][i]
        self.pub_imu.publish(imu)

        t = Temperature()
        t.header.stamp = stamp
        t.header.frame_id = frame
        t.temperature = temp
        self.pub_temp.publish(t)

        field = parse_mag(raw_mag, self.asa)
        if field is not None:
            m = MagneticField()
            m.header.stamp = stamp
            m.header.frame_id = frame
            m.magnetic_field.x, m.magnetic_field.y, m.magnetic_field.z = field
            for i in range(3):
                m.magnetic_field_covariance[4 * i] = self.p['variance_mag'][i]
            self.pub_mag.publish(m)


def main():
    rclpy.init()
    rclpy.spin(Mpu9250())


if __name__ == '__main__':
    main()
