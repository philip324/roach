#!/usr/bin/env python
import math
import rospy
import numpy as np
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
import datetime
import signal
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from imageproc.msg import velroach_msg
import IPython
import time, sys, os, traceback
import serial
import pickle

sys.path.append(os.path.dirname("../../imageproc-settings/"))
sys.path.append(os.path.dirname("../imageproc-settings/"))
from lib import command
import shared_multi as shared
from velociroach import *
from stop_robot import stop

###############################
####### VARS TO SPECIFY #######
###############################
frequency_value = 100    # can go as high as 240 Hz
MOTORGAINS  = [200,200,0,0,0 , 200,200,0,0,0]
STRIDE_FREQ = 10
EXPERIMENT_RUN_TIME_MS     = 3000 #ms
EXPERIMENT_LEADIN_TIME_MS  = 100  #ms
EXPERIMENT_LEADOUT_TIME_MS = 100  #ms
GAIT = "alt_tripod"

#init ROS node
rospy.init_node(GAIT+'_data', anonymous=True)
rate = rospy.Rate(frequency_value)

# callback for mocap info
def callback_mocap(data):
    global mocap_info
    mocap_info = data

# mocap data, published by mocap.launch
sub_mocap = rospy.Subscriber('/mocap/pose', PoseStamped, callback_mocap)
mocap_info = PoseStamped()

#setup ROS publishers
publish_robotinfo= rospy.Publisher('/robot0/robotinfo', velroach_msg, queue_size=5) #publish robotinfo from roach

def main():
    xb = setupSerial(shared.BS_COMPORT, shared.BS_BAUDRATE)
    robot = Velociroach('\x21\x61', xb)
    shared.ROBOTS.append(robot) #This is necessary so callbackfunc can reference robots
    shared.xb = xb           #This is necessary so callbackfunc can halt before exit

    for r in shared.ROBOTS:
        if r.RESET:
            r.reset()
            time.sleep(0.35)
    for r in shared.ROBOTS:
        r.query(retries = 3)
    verifyAllQueried()

    # set gait config
    simpleAltTripod = GaitConfig(motorgains=MOTORGAINS)
    simpleAltTripod.leftFreq = STRIDE_FREQ
    simpleAltTripod.rightFreq = STRIDE_FREQ
    simpleAltTripod.phase = PHASE_180_DEG
    simpleAltTripod.deltasLeft = [0.25, 0.25, 0.25]
    simpleAltTripod.deltasRight = [0.25, 0.25, 0.25]
    robot.setGait(simpleAltTripod)

    # start experiment
    list_mocap_info = []
    time.sleep(EXPERIMENT_LEADIN_TIME_MS / 1000.0)
    robot.startTimedRun(EXPERIMENT_RUN_TIME_MS)
    start_time = time.time()
    while (time.time() - start_time) < (EXPERIMENT_RUN_TIME_MS / 1000.0):
        list_mocap_info.append(mocap_info)
        rate.sleep()
    time.sleep(EXPERIMENT_LEADOUT_TIME_MS / 1000.0)
    stop()  # stop experiment
    # saving data
    print("\nsaving data")
    exp_name = GAIT + '_' + datetime.datetime.now().strftime('%Y.%m.%d_%H:%M:%S')
    data_dir = os.path.join(os.path.join(os.getcwd()), "Data")
    if not os.path.exists(data_dir):
        os.makedirs(data_dir)
    mocap_file = data_dir + "/" + "mocap_" + exp_name + '.obj'
    pickle.dump(list_mocap_info, open(mocap_file,'w'))
    print("Done.\n")


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\nRecieved Ctrl+C, exiting.")
    except Exception as args:
        print("\nGeneral exception from main:\n",args,'\n')
        print("\n    ******    TRACEBACK    ******    ")
        traceback.print_exc()
        print("    *****************************    \n")
        print("Attempting to exit cleanly...")
    finally:
        xb_safe_exit(shared.xb)
