import time, sys, os, traceback
import serial
from velociroach import *
sys.path.append(os.path.dirname("../../imageproc-settings/"))
sys.path.append(os.path.dirname("../imageproc-settings/"))
import shared


DEFAULT_ADDRS = shared.DEST_ADDR
serial_port = shared.BS_COMPORT
baud_rate = shared.BS_BAUDRATE


def stop():
    xb = setupSerial(serial_port, baud_rate)
    robot = Velociroach(DEFAULT_ADDRS, xb)
    time.sleep(0.1)
    robot.setMotorMode(0, 0)
    robot.pidStopMotors()
    time.sleep(0.1)
    xb_safe_exit(xb)
    return

if __name__ == '__main__':
    stop()
