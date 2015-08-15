import rospy
import os
import time
import pickle
import numpy
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2

class SingleSubscription(object):
    def snapshot(self, rospath, dest_path, msg_type = CompressedImage, save_to_file=True):    
        """
        Subscribe to the ROS topic and prepare to
        save the incoming image to disk in the dest_dir directory
        """
	self.topic = rospath
        self.save_to_file = save_to_file
        # flag to indicate when the image has been saved
        self.done = False          
        self.path = os.path.abspath(dest_path) 
	self.msg_type = msg_type
        # subscribe to ROS topic        
        self.subscriber = rospy.Subscriber(rospath, msg_type, self.callback)

    def callback(self, ros_msg):
        """ 
        This method is invoked each time a new ROS message is generated.
        the message is of type CompressedImage, with data and format
        """
        self.msg = ros_msg
        
        # we don't need to be called again
        self.subscriber.unregister()
        
        if self.save_to_file:
            # create directories if necessary
            dest_dir = os.path.split(self.path)[0]    
            if not os.path.exists(dest_dir):
                os.makedirs(dest_dir)
            # write data to disk
	    if self.msg_type == CompressedImage:
                f = open(self.path, 'w')
		f.write(ros_msg.data)
		f.close()
	    elif self.msg_type == PointCloud2:
		rawpoints = numpy.array(list(point_cloud2.read_points(ros_msg, skip_nans=False)), dtype=numpy.float32)[:, :3]
		notnanindices = ~numpy.isnan(rawpoints[:, 0])
		f = open(self.path, 'wb')
		pickle.dump((rawpoints[notnanindices], notnanindices, len(rawpoints)), f)
		f.close()
	    else:
                f = open(self.path, 'wb')
		pickle.dump(ros_msg, f)
		f.close()
        
        self.done = True
        
    def waituntildone(self, timeout=30):
        for i in range(timeout*20):
            if self.done: 
                return True
            time.sleep(0.05)
        print "TIMEOUT while listening to ",self.topic,"!"
        return False
        
        
############## world state utils (used to get ground truth)

import sys
import StringIO
from rosservice import _rosservice_cmd_call

def getobjpose(modelname='atlas'):
    stdout = sys.stdout
    sys.stdout = StringIO.StringIO() # replace by stringio to capture model command
    _rosservice_cmd_call([1,2,'gazebo/get_model_state', '{model_name: '+modelname+'}'])
    sys.stdout.seek(0)
    outstr = sys.stdout.read()
    sys.stdout = stdout # restore sysout
    
    parsevars = ["x", "y", "z"]
    pos = []
    for v in parsevars:
        s = outstr[(outstr.index(v+": ")+3):(outstr.index(v+": ")+14)]
        s = s.split(' ')[0].strip()
        pos.append(float(s))
    outstr = outstr[outstr.index('orientation:'):]
    parsevars = ["x", "y", "z", "w"]
    angle = []
    for v in parsevars:
        s = outstr[(outstr.index(v+": ")+3):(outstr.index(v+": ")+14)]
        s = s.split(' ')[0].strip()
        angle.append(float(s))
    return pos, angle
    
"""
def setobjpose(modelname='atlas', pos, angle, twist=[0,0,0,0,0,0]):
    args = "{model_state: { model_name: %s, pose: { position: { x: %f, y: %f, z: %f }, orientation: {x: %f, y: %f, z: %f, w: %f } }, twist: { linear: {x: %f , y: %f ,z: %f } , angular: { x: %f , y: %f, z: %f} } , reference_frame: world } }" % (modelname, pos[0], pos[1], pos[2], angle[0], angle[1], angle[2], angle[3], twist[0], twist[1], twist[2], twist[3], twist[4], twist[5])
    _rosservice_cmd_call(["-","-",'gazebo/set_model_state', args])
"""
