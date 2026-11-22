"""ST LIS3MDL magnetometer over I2C.

Publishes raw data only: mag (tesla, in the chip's own axes). Register
addresses from the LIS3MDL datasheet (DocID024204). Runs at 80 Hz in
ultra-high-performance mode; only samples flagged new (STATUS_REG ZYXDA) are
published, so poll faster than 80 Hz.
"""
import struct
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField
from smbus2 import SMBus

WHO_AM_I = 0x0F       # reads 0x3D
CTRL_REG1 = 0x20
CTRL_REG2 = 0x21      # full scale [6:5], SOFT_RST bit 2
CTRL_REG3 = 0x22
CTRL_REG4 = 0x23
CTRL_REG5 = 0x24
STATUS_REG = 0x27     # 7 bytes from here: status, x, y, z (little-endian)
AUTO_INC = 0x80       # set in the register address to read several bytes

LSB_PER_GAUSS = (6842.0, 3421.0, 2281.0, 1711.0)   # mag_range 0..3 = ±4, 8, 12, 16 gauss


def parse(buf, mag_range):
    """7 bytes from STATUS_REG -> (new sample?, field in tesla)."""
    status, x, y, z = struct.unpack('<B3h', bytes(buf))
    s = 1e-4 / LSB_PER_GAUSS[mag_range]
    return bool(status & 0x08), (x * s, y * s, z * s)


class Lis3mdl(Node):

    def __init__(self):
        super().__init__('lis3mdl')
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_addr', 0x1C)
        self.declare_parameter('frame_id', 'lis3mdl')
        self.declare_parameter('rate', 100.0)
        self.declare_parameter('mag_range', 0)
        self.declare_parameter('variance_mag', [0.0, 0.0, 0.0])
        self.p = p = {n: self.get_parameter(n).value for n in (
            'i2c_bus', 'i2c_addr', 'frame_id', 'rate', 'mag_range', 'variance_mag')}
        self.get_logger().info(f'parameters: {p}')

        self.pub = self.create_publisher(MagneticField, 'mag', 10)
        self.bus = SMBus(p['i2c_bus'])
        self.ready = False
        self.create_timer(1.0 / p['rate'], self.tick)

    def setup(self):
        """Reset and configure the chip. Raises OSError if it doesn't answer."""
        b, a = self.bus, self.p['i2c_addr']
        b.write_byte_data(a, CTRL_REG2, 0x04)                      # soft reset
        time.sleep(0.05)
        b.write_byte_data(a, CTRL_REG1, 0x7C)                      # X/Y ultra-high performance, 80 Hz
        b.write_byte_data(a, CTRL_REG2, self.p['mag_range'] << 5)
        b.write_byte_data(a, CTRL_REG4, 0x0C)                      # Z ultra-high performance
        b.write_byte_data(a, CTRL_REG5, 0x40)                      # block data update
        b.write_byte_data(a, CTRL_REG3, 0x00)                      # continuous conversion
        who = b.read_byte_data(a, WHO_AM_I)
        self.get_logger().info(f'WHO_AM_I 0x{who:02X}')

    def tick(self):
        try:
            if not self.ready:
                self.setup()
                self.ready = True
            raw = self.bus.read_i2c_block_data(self.p['i2c_addr'], STATUS_REG | AUTO_INC, 7)
        except OSError as e:
            # Unplugged or bus error: set the chip up again on the next tick.
            self.ready = False
            self.get_logger().warn(f'I2C error: {e}', throttle_duration_sec=5.0)
            return
        new, field = parse(raw, self.p['mag_range'])
        if not new:
            return
        m = MagneticField()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.p['frame_id']
        m.magnetic_field.x, m.magnetic_field.y, m.magnetic_field.z = field
        for i in range(3):
            m.magnetic_field_covariance[4 * i] = self.p['variance_mag'][i]
        self.pub.publish(m)


def main():
    rclpy.init()
    rclpy.spin(Lis3mdl())


if __name__ == '__main__':
    main()
