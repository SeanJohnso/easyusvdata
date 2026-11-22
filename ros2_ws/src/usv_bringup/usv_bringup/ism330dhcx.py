"""ST ISM330DHCX accelerometer + gyroscope over I2C.

Publishes raw data only: imu (accel + gyro, no orientation) and temp.
Register addresses from the ISM330DHCX datasheet (DS13012). Only samples the
chip flags as new (STATUS_REG XLDA) are published, so poll faster than the ODR.
"""
import struct
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Temperature
from smbus2 import SMBus

WHO_AM_I = 0x0F       # reads 0x6B
CTRL1_XL = 0x10       # accel ODR [7:4], full scale [3:2]
CTRL2_G = 0x11        # gyro ODR [7:4], full scale [3:1]
CTRL3_C = 0x12        # BDU bit 6, IF_INC bit 2, SW_RESET bit 0
STATUS_REG = 0x1E     # 16 bytes from here: status, (reserved), temp, gyro xyz, accel xyz, little-endian

G = 9.80665
DEG = 0.017453292519943295
# accel_range 0..3 = ±2, 4, 8, 16 g: register code, mg per LSB
ACCEL = ((0b00, 0.061), (0b10, 0.122), (0b11, 0.244), (0b01, 0.488))
# gyro_range 0..4 = ±250, 500, 1000, 2000, 4000 deg/s: CTRL2_G bits [3:0], mdps per LSB
GYRO = ((0b0000, 8.75), (0b0100, 17.5), (0b1000, 35.0), (0b1100, 70.0), (0b0001, 140.0))


def parse(buf, accel_range, gyro_range):
    """16 bytes from STATUS_REG -> (new sample?, accel m/s^2, gyro rad/s, temp degC)."""
    status, _, t, gx, gy, gz, ax, ay, az = struct.unpack('<BB7h', bytes(buf))
    a = ACCEL[accel_range][1] * 1e-3 * G
    w = GYRO[gyro_range][1] * 1e-3 * DEG
    return bool(status & 0x01), (ax * a, ay * a, az * a), (gx * w, gy * w, gz * w), t / 256.0 + 25.0


class Ism330dhcx(Node):

    def __init__(self):
        super().__init__('ism330dhcx')
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_addr', 0x6A)
        self.declare_parameter('frame_id', 'ism330dhcx')
        self.declare_parameter('rate', 200.0)
        self.declare_parameter('odr', 4)
        self.declare_parameter('accel_range', 0)
        self.declare_parameter('gyro_range', 0)
        self.declare_parameter('variance_acc', [0.0, 0.0, 0.0])
        self.declare_parameter('variance_angular_vel', [0.0, 0.0, 0.0])
        self.p = p = {n: self.get_parameter(n).value for n in (
            'i2c_bus', 'i2c_addr', 'frame_id', 'rate', 'odr', 'accel_range', 'gyro_range',
            'variance_acc', 'variance_angular_vel')}
        self.get_logger().info(f'parameters: {p}')

        self.pub_imu = self.create_publisher(Imu, 'imu', 10)
        self.pub_temp = self.create_publisher(Temperature, 'temp', 10)
        self.bus = SMBus(p['i2c_bus'])
        self.ready = False
        self.create_timer(1.0 / p['rate'], self.tick)

    def setup(self):
        """Reset and configure the chip. Raises OSError if it doesn't answer."""
        b, a, p = self.bus, self.p['i2c_addr'], self.p
        b.write_byte_data(a, CTRL3_C, 0x01)                   # software reset
        time.sleep(0.05)
        b.write_byte_data(a, CTRL3_C, 0x44)                   # BDU, register auto-increment
        b.write_byte_data(a, CTRL1_XL, p['odr'] << 4 | ACCEL[p['accel_range']][0] << 2)
        b.write_byte_data(a, CTRL2_G, p['odr'] << 4 | GYRO[p['gyro_range']][0])
        who = b.read_byte_data(a, WHO_AM_I)
        self.get_logger().info(f'WHO_AM_I 0x{who:02X}')

    def tick(self):
        try:
            if not self.ready:
                self.setup()
                self.ready = True
            raw = self.bus.read_i2c_block_data(self.p['i2c_addr'], STATUS_REG, 16)
        except OSError as e:
            # Unplugged or bus error: set the chip up again on the next tick.
            self.ready = False
            self.get_logger().warn(f'I2C error: {e}', throttle_duration_sec=5.0)
            return
        new, acc, gyro, temp = parse(raw, self.p['accel_range'], self.p['gyro_range'])
        if not new:
            return
        stamp = self.get_clock().now().to_msg()
        frame = self.p['frame_id']

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


def main():
    rclpy.init()
    rclpy.spin(Ism330dhcx())


if __name__ == '__main__':
    main()
