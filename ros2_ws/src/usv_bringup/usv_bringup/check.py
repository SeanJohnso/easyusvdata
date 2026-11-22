"""Print a status table of every module in rig.yaml every 2 s.

driver:  OK = a message in the last 2 s, STALE = data earlier but none in the
         last 2 s, NO DATA = nothing since this node started.
rate:    messages per second over the last 2 s.
fix:     GPS only, NavSatFix status >= 0.
pi-gps:  GPS only, Pi clock at arrival minus the fix stamp, in seconds. The u-blox
         driver stamps fixes with GPS time once it is valid, so this is the Pi
         clock error plus serial/driver latency (normally a few hundredths of a second).
         Seconds or more means the Pi clock is wrong.
"""
import os
import time

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, NavSatFix
import yaml

PERIOD = 2.0


class Module:

    def __init__(self, name, kind):
        self.name = name
        self.kind = kind
        self.count = 0        # messages since the last report
        self.last = None      # arrival time of the last message (monotonic s)
        self.fix = None
        self.offset = None   # s


class Check(Node):

    def __init__(self):
        super().__init__('check')
        default = os.path.join(get_package_share_directory('usv_bringup'), 'config', 'rig.yaml')
        path = self.declare_parameter('rig', default).value
        with open(path) as f:
            rig = yaml.safe_load(f)

        self.modules = []
        for gid in rig['gps']:
            m = Module(gid, 'gps')
            self.modules.append(m)
            self.create_subscription(NavSatFix, f'/gps/{gid}/fix',
                                     lambda msg, m=m: self.on_fix(m, msg), qos_profile_sensor_data)
        for iid in rig['imu']:
            m = Module(iid, 'imu')
            self.modules.append(m)
            self.create_subscription(Imu, f'/imu/{iid}/imu',
                                     lambda msg, m=m: self.on_msg(m), qos_profile_sensor_data)
        self.width = max(len(m.name) for m in self.modules) + 2
        self.create_timer(PERIOD, self.report)

    def on_msg(self, m):
        m.count += 1
        m.last = time.monotonic()

    def on_fix(self, m, msg):
        self.on_msg(m)
        m.fix = msg.status.status >= 0
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        m.offset = time.time() - stamp

    def report(self):
        now = time.monotonic()
        w = self.width
        lines = [f'--- {time.strftime("%H:%M:%S")} ' + '-' * 40,
                 f'{"module":<{w}}{"type":<6}{"driver":<9}{"rate":>9}  {"fix":<8}{"pi-gps":>12}']
        for m in self.modules:
            if m.last is None:
                driver = 'NO DATA'
            elif now - m.last > PERIOD:
                driver = 'STALE'
            else:
                driver = 'OK'
            ok = driver == 'OK'
            rate = f'{m.count / PERIOD:.1f} Hz' if ok else '-'
            if m.kind == 'gps' and ok:
                fix = 'FIX' if m.fix else 'NO FIX'
                offset = f'{m.offset:+.3f} s'
            else:
                fix = offset = '-'
            lines.append(f'{m.name:<{w}}{m.kind:<6}{driver:<9}{rate:>9}  {fix:<8}{offset:>12}')
            m.count = 0
        print('\n'.join(lines), flush=True)


def main():
    rclpy.init()
    try:
        rclpy.spin(Check())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
