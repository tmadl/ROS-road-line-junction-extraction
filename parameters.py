import os

CN24PATH = os.environ['CN24PATH'] # /home/USER/cn24/build

SIMPLIFICATION_TOLERANCE=120 # (in px) how much to simplify road line polygon (max distance between points)
CLOSELINEMAXDIST = 200 # (in px) maximum distance over which points may be connected
ANGLETOLERANCE = 0.5 # (in rad) sharpest angle allowed to be a junction

# Image size
try:
    ISIZE = int(os.environ['IMAGESIZE'])
except:
    ISIZE = 800

ROBOT_HEIGHT = 1.85 # (in m) approx. height of camera of robot above floor (this is for the Atlas robot by Boston Dynamics)
FLOORTOLERANCE = 0.7 # (in m) floor has to be ROBOT_HEIGHT below sensor, plus/minus this tolerance
MAXACCURATEDEPTH = 10 # (in m) max depth of point cloud estimated from stereo camera - inaccurate beyond this
    
IMGLOAD_RETRIES = 10 # how often to re-try loading the image and point clouds (these are obtained from ROS and may be corrupted sometimes)

BORDERCUTOFF = 10 # (in px) ignore noisy border in road detection
ROAD_CLOSEKERNELSIZE = 18 # (in px) kernel size of OpenCV close operation when closing holes and artifacts in recognized role

MINARCLEN=20 # (in px) arcs of the thinned road below this length are discarded as noise

# exclude some parts of birds-eye view projection (to remove parts of the image that are black [impossible to compute from first-person perspective] or likely to be erroneous)
BIRDSEYE_EXCLUDED_AREAS = [
    [600, 800, 450, 800], # (in px) black triangle in lower right corner of birds-eye projection arising from limited perspective
    [0, 140, 0, 800], # (in px) top part of birds-eye projected img (too far away)
    [660, 800, 0, 800] # (in px) bottom part of birds-eye projected img (too close away)
]
BIRDSEYEZOOM = 0.5 # controls how much will be included in the birds-eye image (roughly the "height" of a birds-eye camera)

