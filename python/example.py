import sys
import pykrang
import time
from math import *


def str(xvec):
    s = '[ '
    for x in xvec:
        s += "%.4f" % x
        s += ' '
    s += ']'
    return s


interface_context = pykrang.InterfaceContext('pykrang')
world = pykrang.WorldInterface(interface_context, 'sim-cmd')
imu = pykrang.FloatingBaseStateSensorInterface(interface_context, 'imu-data')
wheels = pykrang.MotorInterface(interface_context, 'wheels', 'amc-cmd',
                                'amc-state', 2)
torso = pykrang.MotorInterface(interface_context, 'torso', 'torso-cmd',
                               'torso-state', 1)
torso.LockCommand()
left_arm = pykrang.MotorInterface(interface_context, 'left-arm', 'llwa-cmd',
                                  'llwa-state', 7)
left_arm.LockCommand()
right_arm = pykrang.MotorInterface(interface_context, 'right-arm', 'rlwa-cmd',
                                   'rlwa-state', 7)
right_arm.LockCommand()

iter = 0
dt = 0.001
while True:
    try:
        current_command = pykrang.VectorOfDoubles()
        period = 5.0  # sec
        current_command[:] = [
            30 * sin(2 * pi * iter * dt / period),
            30 * sin(2 * pi * iter * dt / period)
        ]
        wheels.CurrentCommand(current_command)

        world.Step()

        iter = iter + 1

        if iter % 500 == 0:
            #imu.UpdateState()
            #print 'imu angle: ', "%.4f" % (imu.base_angle * 180.0 / pi)
            #print 'imu speed: ', "%.4f" % imu.base_angular_speed

            wheels.UpdateState()
            print 'wheel pos: ', str(wheels.GetPosition())
            print 'wheel vel: ', str(wheels.GetVelocity())
            print 'wheel cur: ', str(wheels.GetCurrent())

            torso.UpdateState()
            print 'torso pos: ', str(torso.GetPosition())
            print 'torso vel: ', str(torso.GetVelocity())
            print 'torso cur: ', str(torso.GetCurrent())

            left_arm.UpdateState()
            print 'L-arm pos: ', str(left_arm.GetPosition())
            print 'L-arm vel: ', str(left_arm.GetVelocity())
            print 'L-arm cur: ', str(left_arm.GetCurrent())

            right_arm.UpdateState()
            print 'R-arm pos: ', str(right_arm.GetPosition())
            print 'R-arm vel: ', str(right_arm.GetVelocity())
            print 'R-arm cur: ', str(right_arm.GetCurrent())

        interface_context.Run()
    except KeyboardInterrupt:
        sys.exit(0)
