"""Step-by-step /mark publisher for the calibration, motor and dynamic recordings.

    ros2 run usv_bringup mark <calib|motor|dynamic>

Run it in its own terminal while `ros2 bag record -a` is running. It walks through
the steps of the chosen mode in a fixed order. Press Enter to start a step: its
name is published on /mark (std_msgs/String). Timed steps end on their own with an
`end` mark and a beep; untimed steps end when Enter is pressed again. Ctrl-C during
a step publishes `abort`, and processing ignores that step.

The processing scripts (postprocessing/calibrate.py and later) find the steps by
these names, so keep them in sync when changing a mode.
"""
import sys
import time

import rclpy
from std_msgs.msg import String

# (mark, seconds or None = until Enter, instruction)
FACE = 15
MODES = {
    # Rig off the vehicle, away from metal. Faces: that rig axis points up.
    'calib': [
        ('+z', FACE, 'rig flat, top up, still'),
        ('-z', FACE, 'rig upside down, still'),
        ('+x', FACE, 'rig on its back edge, front (+x) pointing up, still'),
        ('-x', FACE, 'rig on its front edge, back (-x) pointing up, still'),
        ('+y', FACE, 'rig on its right edge, left side (+y) pointing up, still'),
        ('-y', FACE, 'rig on its left edge, right side (-y) pointing up, still'),
        ('tumble', None, 'turn the rig slowly through every orientation, upside down '
                         'included, for 1-2 min'),
    ],
    # Rig on the vehicle.
    'motor': [
        ('off', 60, 'wheels lifted, motors off'),
        ('low', 60, 'wheels lifted, motors running slow'),
        ('mid', 60, 'wheels lifted, motors at half speed'),
        ('high', 60, 'wheels lifted, motors at full speed'),
        ('off', 60, 'wheels lifted, motors off again'),
        ('turns', None, 'wheels down: 2-3 slow turns in place each way'),
    ],
    'dynamic': [
        ('still', 120, 'vehicle at the start, still'),
        ('cw', None, 'drive the clockwise loops (corner stops 20-30 s)'),
        ('ccw', None, 'drive the counter-clockwise loops (corner stops 20-30 s)'),
        ('still', 120, 'vehicle at the start, still'),
    ],
}


def main():
    if len(sys.argv) < 2 or sys.argv[1] not in MODES:
        print(f'usage: ros2 run usv_bringup mark <{"|".join(MODES)}>')
        return
    steps = MODES[sys.argv[1]]
    rclpy.init()
    node = rclpy.create_node('mark')
    pub = node.create_publisher(String, '/mark', 10)

    def mark(text):
        pub.publish(String(data=text))
        print(f'   /mark "{text}"  ({time.strftime("%H:%M:%S")})')

    try:
        for i, (name, secs, what) in enumerate(steps, 1):
            print(f'\n[{i}/{len(steps)}] {name}: {what}'
                  + (f' ({secs} s)' if secs else ''))
            input('   Enter to start ')
            if pub.get_subscription_count() == 0:
                print('   WARNING: nothing subscribes to /mark. Is the bag recording?')
            mark(name)
            try:
                if secs:
                    for left in range(secs, 0, -1):
                        print(f'\r   {left:3d} s ', end='', flush=True)
                        time.sleep(1)
                    print('\r', end='')
                else:
                    input('   Enter when done ')
            except KeyboardInterrupt:
                print()
                mark('abort')
                if input('   step aborted. Enter to redo it, q to quit: ').strip() == 'q':
                    return
                steps.insert(i, (name, secs, what))   # run the same step again next
                continue
            mark('end')
            print('\a', end='', flush=True)
        print('\nall steps done; stop the bag recording now.')
    except (KeyboardInterrupt, EOFError):
        print('\nquit')
    finally:
        time.sleep(0.3)   # let the last mark go out
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
