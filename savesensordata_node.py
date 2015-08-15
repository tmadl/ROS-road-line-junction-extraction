import signal
import sys
import time
import numpy

import rospy
from rosutils import *
from sensor_msgs.msg import PointCloud2
import nav_msgs
from nav_msgs.msg import Odometry

from threading import Thread

IMGTOPIC = "/multisense_sl/camera/left/image_color/compressed"
IMGPATH = "snapshot.jpg"

DEPTHTOPIC = "/multisense_sl/points2"
DEPTHPATH = "depth.dat"

topics = [IMGTOPIC, DEPTHTOPIC, "/ground_truth_odom"]
paths = [IMGPATH, DEPTHPATH, "gtpos.dat"]
types = [CompressedImage, PointCloud2, Odometry]

threads = []

running = False
times = []
def subscribe(topic, savepath, stype, sleep=1):
    global running
    print "subscribing to "+topic+" (running: "+str(running)+")"
    while running:
        imgsub = SingleSubscription()
        imgsub.snapshot(topic, savepath, stype, True)
        imgsub.waituntildone()
        sys.stdout.write(".")
	time.sleep(sleep)
    exitsignal()

def savesensordata():
    global running
    running = True
    for i in range(len(topics)):
	p = Thread(target=subscribe, args=(topics[i], paths[i], types[i]))
        p.start()
	threads.append(p)

def exitsignal(signal=None, frame=None):
    running = False
    del threads[:]

if __name__ == "__main__":
    rospy.init_node('savesensordata_node')
    print('Storing robot sensor data. Press Ctrl+C to exit.')
    signal.signal(signal.SIGINT, exitsignal)
    savesensordata()
