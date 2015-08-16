import traceback
import numpy
import time
import itertools
import pickle
import matplotlib.image as mpimg
import cv2
import scipy.spatial
from scipy.spatial.distance import pdist, squareform

try:
    import pcl
except:
    print "Warning: PCL library not found. Road line segment density will not be reduced. MST calculation will be VERY SLOW."

from parameters import *
from geometry_utils import *

import scipy.ndimage
import skimage.morphology
def skeletonize(image, size=(5,5)):
    distance_img = scipy.ndimage.distance_transform_edt(image)
    morph_laplace_img = scipy.ndimage.morphological_laplace(distance_img, size)
    midline = (morph_laplace_img < morph_laplace_img.min()/2).astype(numpy.uint8)
    midline=skimage.morphology.skeletonize(midline).astype(numpy.uint8)
    return midline

####################

def getcameraimg(outpath="snapshot.jpg"):
    # ros interfacing is done by savesensordata_node.py
    for i in range(IMGLOAD_RETRIES):
        try:
            img = mpimg.imread(outpath)
            break
        except Exception, message:
            time.sleep(0.05)
    return img

def getdepthimg(outpath="depth.dat", excludeimg=None):
    # ros interfacing is done by savesensordata_node.py
    for i in range(IMGLOAD_RETRIES):
        try:
            with open(outpath, 'rb') as f: 
                (points, notnanindices, N) = pickle.load(f)
            break
        except Exception, message:
            time.sleep(0.05)

    # calc depth map
    depthmap = numpy.zeros((ISIZE*ISIZE,))
    if numpy.any(notnanindices):
        depthmap[notnanindices] = numpy.sqrt(numpy.sum(numpy.square(points), axis = 1))

        # remove points overlapping with excludeimg
        if excludeimg != None:
            exclude = numpy.reshape(excludeimg, (ISIZE*ISIZE,))
            points = points[exclude[notnanindices]==0, :]
            depthmap[exclude!=0] = 0
            notnanindices = notnanindices[exclude!=0]
    return numpy.reshape(depthmap, (ISIZE, ISIZE)), points, notnanindices

def getroadimg(inpath="snapshot.jpg", outpath="road_out.jpg", cutoff_y=400, confidence_threshold=0.5):
    cmd = CN24PATH+"/classifyImage "+CN24PATH+"/kitti_um_road.set "+CN24PATH+"/kitti.net "+CN24PATH+"/kitti_pretrained.Tensor "+inpath+" "+outpath
    os.system(cmd)
    img = mpimg.imread(outpath)
    img[:cutoff_y, :] = 0
    img[img<(255.0*confidence_threshold)] = 0
    img = numpy.sum(img, axis=2)
    img[img>0] = 255

    img = (img!=0).astype(numpy.uint8)
    # cut off border
    img[range(BORDERCUTOFF)+range(ISIZE-BORDERCUTOFF,ISIZE), :] = 0
    img[:, range(BORDERCUTOFF)+range(ISIZE-BORDERCUTOFF,ISIZE)] = 0
    # close holes
    kernel = numpy.ones((ROAD_CLOSEKERNELSIZE,ROAD_CLOSEKERNELSIZE), numpy.uint8)
    img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel) 
    
    return img

def getbirdseyeview(camimg, roadimg, depthmap, points, notnanindices):
    # get only camera pixels corresponding to floor
    camimg_floor = numpy.sum(numpy.copy(camimg)+1, axis=2).flatten()
    miny, maxy, maxz = ROBOT_HEIGHT-FLOORTOLERANCE/2, ROBOT_HEIGHT+FLOORTOLERANCE/2, MAXACCURATEDEPTH
    nnfloor = ((points[:, 1]>=miny)&(points[:, 1]<=maxy)&(points[:, 2]<=maxz))
    nnfloor = numpy.logical_and(nnfloor, roadimg.flatten()[numpy.where(notnanindices)])
    floor = numpy.zeros(notnanindices.shape)
    floor[numpy.where(notnanindices)] = nnfloor
    camimg_floor[numpy.logical_not(numpy.logical_and(notnanindices, floor))] = 0
    camimg_floor = numpy.reshape(camimg_floor, (ISIZE,ISIZE)).astype(int)
    ny, nx = numpy.nonzero(camimg_floor)
    camimg_floor_coords = numpy.vstack((nx, ny)).T.astype(numpy.float32)
    # corners
    depthimgcorneridx = [numpy.argmin(ny), numpy.argmin(nx), numpy.argmax(ny), numpy.argmax(nx)]
    depthimgcorners = []
    for i in depthimgcorneridx: 
        depthimgcorners.append(points[nnfloor][i, [0,2]])
    # warp to birds eye perspective
    destpoints = points[nnfloor][:, [0,2]]
    destpoints -= numpy.min(destpoints, axis=0)
    destpoints /= numpy.max(destpoints, axis=0)
    destpoints *= [abs(nx[depthimgcorneridx[3]]-nx[depthimgcorneridx[1]]), abs(ny[depthimgcorneridx[2]]-ny[depthimgcorneridx[0]])]
    destpoints *= BIRDSEYEZOOM
    H = cv2.findHomography(camimg_floor_coords, destpoints.astype(numpy.float32))
    warpedimg = cv2.warpPerspective(camimg, H[0], (ISIZE,ISIZE))
    warpedimg = numpy.flipud(warpedimg)
    return warpedimg, H

def getmidpoint(camimg):
    try:
        nzidx = numpy.nonzero(camimg[ISIZE-BORDERCUTOFF-1, :])[0]
        mn, mx = numpy.min(nzidx), numpy.max(nzidx)
        midpoint = (mx-mn)/2+mn
    except:
        midpoint = ISIZE/2
    return midpoint

def getroadlinesegments(camimg, warpedimg, H, accurate_intersections=False):
    scipy.misc.imsave("birdseyesnapshot.jpg", warpedimg)
    # get road in birds eye perspective
    wroadimg = getroadimg("birdseyesnapshot.jpg", cutoff_y=0, confidence_threshold=0.05)
    warpedimg[wroadimg!=0] = [0,255,0] # paint birds-eye road green for visualization
    midline = skeletonize(wroadimg, size=(5,5))
    
    # remove artefacts from road midline
    for [ymin, ymax, xmin, xmax] in BIRDSEYE_EXCLUDED_AREAS:
        midline[ymin:ymax, xmin:xmax] = 0
    # find contours, get rid of tiny arcs
    line_endpoints = []
    epsilon = 0
    (cnts, _) = cv2.findContours(midline.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    coords = []
    trees = []
    for c in cnts: 
        points = []
        if cv2.arcLength(c, False) > MINARCLEN:
            cpts = numpy.array(cv2.approxPolyDP(c, epsilon, False)[:, 0, :])
            d = numpy.sum(numpy.square(cpts-numpy.array([ISIZE, 0])), axis=1)
            cpts = cpts[numpy.argsort(d), :]
            coords.append(cpts)
            trees.append(scipy.spatial.cKDTree(cpts))
    # concat points
    allpoints = numpy.vstack(coords)
    points = numpy.copy(allpoints)

    # keep (append later) nearest neighbor points which are non-adjacent (>SIMPLIFICATION_TOLERANCE), to preserve precision of intersections
    if accurate_intersections:
        def nn(tree, points, k=1, bound=CLOSELINEMAXDIST):
            dist, indices = tree.query(points)
            i = numpy.argmin(dist)
            return dist[i], indices[i]
        toappend = []
        for i in range(len(trees)):
            for j in range(i+1, len(trees)):
                d1, k1 = nn(trees[i], coords[j])
                d2, k2 = nn(trees[j], coords[i])
                if d1 < CLOSELINEMAXDIST and d2 < CLOSELINEMAXDIST:
                    toappend.append(coords[i][k1, :])
                    toappend.append(coords[j][k2, :])
        rows = [tuple(row) for row in toappend]
        toappend = np.unique(rows)

    # downsample points (reduce their density)
    try:
        f = pcl.PointCloud(numpy.hstack((points, numpy.zeros((len(points), 1)))).astype(numpy.float32)).make_voxel_grid_filter()
        f.set_leaf_size(SIMPLIFICATION_TOLERANCE, SIMPLIFICATION_TOLERANCE, SIMPLIFICATION_TOLERANCE)
        points = numpy.asarray(f.filter())[:, :2]

        # append back points at intersections, but delete closeby downsampled one first to avoid redundancy        
        if accurate_intersections and len(toappend)>0:
            ptree = scipy.spatial.cKDTree(points)
            removelist = []
            for pt in toappend:
                d, idx = ptree.query(pt, k=1+len(removelist))
                if type(d) == float: 
                    d = [d]
                    idx = [idx]
                i = 0
                while i<len(d) and idx[i] in removelist and d[i] < SIMPLIFICATION_TOLERANCE: 
                    i+= 1
                if d[i] < SIMPLIFICATION_TOLERANCE:
                    removelist.append(idx[i])
            points = numpy.delete(points, removelist, axis=0)
            points = numpy.vstack((points, numpy.array(list(toappend))))
    except:
        pass
        
    # add midpoint
    midpoint = getmidpoint(camimg)
    wmidpoint = homographyProjection(midpoint, ISIZE, H[0])
    points = numpy.vstack((points, [wmidpoint[0], ISIZE]))
    #d = numpy.sum(numpy.square(points-numpy.array([wmidpoint[0], ISIZE])), axis=1)
    #points = points[numpy.argsort(d), :]
    # correctly connect - minimum spanning tree excluding non-road edges
    D = squareform(pdist(points))
    
    # set distances crossing non-road pixels to high value (we want to avoid connecting these)
    ADD = 1e6
    pxcheckinterval = 20
    for i in range(len(points)):
        for j in range(i+1, len(points)):
            pts = scipy.spatial.distance.euclidean(points[i, :], points[j, :]) / pxcheckinterval
            if pts < 1: continue
            try:
                linepx = get_line_float(points[i,0], points[i,1], points[j,0], points[j,1], pts).astype(int).tolist()
            except:
                pass
            for lpx in linepx:
                if wroadimg[lpx[1], lpx[0]] == 0: # connecting line lies outside of road - make it unlikely to connect
                    D[i,j] += ADD
                    D[j,i] += ADD

    # obtain MST without crossing road pixels - good approximation of road center
    edge_list = minimum_spanning_tree(D)
    edge_list = numpy.sort(edge_list, axis=1)
    keep = []
    for i in range(len(edge_list)):
        if D[edge_list[i][0], edge_list[i][1]] < ADD:
            keep.append(i) # only keep if edge not outside of road
    edge_list = edge_list[keep, :]
    # find junctions
    icounts = numpy.array([0]*len(points))
    icounts += numpy.bincount(edge_list[:, 0], minlength=len(points))
    icounts += numpy.bincount(edge_list[:, 1], minlength=len(points))
    midpointidx = numpy.where(points[:, 1]==ISIZE)[0]
    if midpointidx: icounts[midpointidx]+=1
    junctionpointidx = numpy.where(icounts>2)[0]
    # remove junctions with too sharp angles
    toosharpidx = []
    for i in junctionpointidx:
        sourcepoint = points[i, :]
        targetidx = list(set(edge_list[numpy.where((edge_list[:,0]==i)|(edge_list[:,1]==i))[0], :].flatten()).difference(set([i])))
        for idxpair in itertools.combinations(targetidx, 2):
            alpha = numpy.arctan2(points[idxpair[0], 1] - points[i, 1], points[idxpair[0], 0] - points[i, 0]) 
            beta = numpy.arctan2(points[idxpair[1], 1] - points[i, 1], points[idxpair[1], 0] - points[i, 0]) 
            if numpy.abs(alpha-beta) < ANGLETOLERANCE:
                toosharpidx.append(i)
    junctionpointidx = list(set(junctionpointidx).difference(set(toosharpidx)))
    
    return points, edge_list, junctionpointidx, allpoints


if __name__ == "__main__":
    try:
        camimg = getcameraimg()
    except Exception,e:
        traceback.print_exc()
        raise Exception("Failed to read cam image.")
        
    try:
        roadimg = getroadimg()
    except Exception,e:
        traceback.print_exc()
        raise Exception("Failed to read road image.")

    try:
        depthmap, points, notnanindices = getdepthimg()
    except Exception,e:
        traceback.print_exc()
        raise Exception("Failed to read depth points from stereo. Try running savesensordata_node.py")
    
    # get birds-eye view
    warpedimg, H = getbirdseyeview(camimg, roadimg, depthmap, points, notnanindices)
    # run road detection   
    points, edge_list, junctionpointidx, allpoints = getroadlinesegments(camimg, warpedimg, H)
    
    # plot results
    import matplotlib
    import matplotlib.pyplot as plt
    
    camimg2 = numpy.copy(camimg)
    #camimg2[roadimg!=0] = [0,255,0] # paint road green for visualization
    
    # project points from birds-eye view onto actual image
    M = numpy.linalg.inv(H[0]) # projection matrix
    projected_points = []
    for i in range(len(points)):
        tPx, tPy = homographyProjection(points[i, 0], ISIZE-points[i, 1], M)
        projected_points.append([tPx, tPy])
    projected_points = numpy.array(projected_points)
    
    # plot both actual image and birds-eye view
    imgs = [warpedimg, camimg2]
    pts = [points, projected_points]
    for c in range(2):
        plt.subplot(1,2,c)
        plt.hold(True)
        plt.imshow(imgs[c])
        plt.scatter(pts[c][:, 0], pts[c][:, 1], c='red', s=30)
        for i in junctionpointidx:
            plt.scatter(pts[c][i, 0], pts[c][i, 1], c='red', s=300)
        for edge in edge_list:
            i, j = edge
            plt.plot([pts[c][i, 0], pts[c][j, 0]], [pts[c][i, 1], pts[c][j, 1]], c='r')
    plt.show()
