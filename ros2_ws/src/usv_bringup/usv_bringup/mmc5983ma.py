"""MEMSIC MMC5983MA magnetometer over I2C.

Publishes raw data only: mag (tesla, in the chip's own axes, 18-bit output).
Register addresses from the MMC5983MA datasheet. Single measurements with
automatic SET/RESET: each tick reads the previous measurement (if the chip
flags it done) and starts the next one. 100 Hz bandwidth = 8 ms per
measurement, so `rate` must stay below about 100 Hz.
"""
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField
from smbus2 import SMBus

XOUT0 = 0x00          # 7 bytes from here: X/Y/Z high bytes, XYZ low bits
STATUS = 0x08         # bit 0 Meas_M_Done (write 1 to clear). Read it on its own, before
                      # the data: reading the data clears it (seen on the rig, 2026-11-22),
                      # so a burst read 0x00-0x08 always shows "not done".
CTRL0 = 0x09          # bit 0 TM_M (start measurement), bit 5 Auto_SR_en
CTRL1 = 0x0A          # bandwidth [1:0], bit 7 SW_RST
PRODUCT_ID = 0x2F     # reads 0x30

COUNTS_PER_GAUSS = 16384.0   # 18-bit, ±8 gauss
NULL = 131072                # 18-bit zero field


def parse(b):
    """7 bytes from XOUT0 -> field in tesla, in a right-handed frame.

    The chip's own axes are left-handed (x, y as marked, z pointing down: silkscreen,
    and confirmed on the rig 2026-11-22 against the board's gyro and the other
    magnetometers), which no TF rotation can express, so z is negated here. The
    published frame then has the marked x and y, and z up.
    """
    x = b[0] << 10 | b[1] << 2 | (b[6] >> 6) & 0x03
    y = b[2] << 10 | b[3] << 2 | (b[6] >> 4) & 0x03
    z = b[4] << 10 | b[5] << 2 | (b[6] >> 2) & 0x03
    s = 1e-4 / COUNTS_PER_GAUSS
    return (x - NULL) * s, (y - NULL) * s, -(z - NULL) * s


class Mmc5983ma(Node):

    def __init__(self):
        super().__init__('mmc5983ma')
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_addr', 0x30)
        self.declare_parameter('frame_id', 'mmc5983ma')
        self.declare_parameter('rate', 50.0)
        self.declare_parameter('variance_mag', [0.0, 0.0, 0.0])
        self.p = p = {n: self.get_parameter(n).value for n in (
            'i2c_bus', 'i2c_addr', 'frame_id', 'rate', 'variance_mag')}
        self.get_logger().info(f'parameters: {p}')

        self.pub = self.create_publisher(MagneticField, 'mag', 10)
        self.bus = SMBus(p['i2c_bus'])
        self.ready = False
        self.create_timer(1.0 / p['rate'], self.tick)

    def setup(self):
        """Reset the chip and start the first measurement. Raises OSError if it doesn't answer."""
        b, a = self.bus, self.p['i2c_addr']
        b.write_byte_data(a, CTRL1, 0x80)          # software reset
        time.sleep(0.02)
        b.write_byte_data(a, CTRL1, 0x00)          # 100 Hz bandwidth
        pid = b.read_byte_data(a, PRODUCT_ID)
        self.get_logger().info(f'PRODUCT_ID 0x{pid:02X}')
        b.write_byte_data(a, CTRL0, 0x21)          # auto SET/RESET, start a measurement

    def tick(self):
        a = self.p['i2c_addr']
        try:
            if not self.ready:
                self.setup()
                self.ready = True
                return
            done = self.bus.read_byte_data(a, STATUS) & 0x01
            if done:
                field = parse(self.bus.read_i2c_block_data(a, XOUT0, 7))
                self.bus.write_byte_data(a, STATUS, 0x01)   # clear Meas_M_Done
            self.bus.write_byte_data(a, CTRL0, 0x21)        # start the next measurement
        except OSError as e:
            # Unplugged or bus error: set the chip up again on the next tick.
            self.ready = False
            self.get_logger().warn(f'I2C error: {e}', throttle_duration_sec=5.0)
            return
        if not done:
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
    rclpy.spin(Mmc5983ma())


if __name__ == '__main__':
    main()
