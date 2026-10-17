"""Republish bno055/imu orientation as roll/pitch/yaw in degrees (diagnostics only)."""
import math

from geometry_msgs.msg import Vector3Stamped
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuEuler(Node):

    def __init__(self):
        super().__init__('imu_euler')
        self.pub = self.create_publisher(Vector3Stamped, 'bno055/euler_deg', 10)
        self.create_subscription(Imu, 'bno055/imu', self.on_imu, 10)

    def on_imu(self, msg):
        q = msg.orientation
        out = Vector3Stamped()
        out.header = msg.header
        # ZYX (yaw-pitch-roll) convention
        out.vector.x = math.degrees(math.atan2(2 * (q.w * q.x + q.y * q.z),
                                               1 - 2 * (q.x * q.x + q.y * q.y)))
        out.vector.y = math.degrees(math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))))
        out.vector.z = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                               1 - 2 * (q.y * q.y + q.z * q.z)))
        self.pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(ImuEuler())


if __name__ == '__main__':
    main()
