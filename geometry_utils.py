import numpy as np

def minimum_spanning_tree(X, copy_X=True):
    """X are edge weights of fully connected graph"""
    if copy_X:
        X = X.copy()
 
    if X.shape[0] != X.shape[1]:
        raise ValueError("X needs to be square matrix of edge weights")
    n_vertices = X.shape[0]
    spanning_edges = []
     
    # initialize with node 0:                                                                                         
    visited_vertices = [0]                                                                                            
    num_visited = 1
    # exclude self connections:
    diag_indices = np.arange(n_vertices)
    X[diag_indices, diag_indices] = np.inf
     
    while num_visited != n_vertices:
        new_edge = np.argmin(X[visited_vertices], axis=None)
        # 2d encoding of new_edge from flat, get correct indices                                                      
        new_edge = divmod(new_edge, n_vertices)
        new_edge = [visited_vertices[new_edge[0]], new_edge[1]]                                                       
        # add edge to tree
        spanning_edges.append(new_edge)
        visited_vertices.append(new_edge[1])
        # remove all edges inside current tree
        X[visited_vertices, new_edge[1]] = np.inf
        X[new_edge[1], visited_vertices] = np.inf                                                                     
        num_visited += 1
    return np.vstack(spanning_edges)
    
# project a point based on a homography projection obtained using openCV's findHomography (based on warpPerspective)    
def homographyProjection(x,y,M):
    destx, desty = [(M[0,0]*x+M[0,1]*y+M[0,2])/(M[2,0]*x+M[2,1]*y+M[2,2]), (M[1,0]*x+M[1,1]*y+M[1,2])/(M[2,0]*x+M[2,1]*y+M[2,2])]
    return destx, desty

# Bresenham's line algorithm
def get_line(x1, y1, x2, y2):
    points = []
    issteep = abs(y2-y1) > abs(x2-x1)
    if issteep:
        x1, y1 = y1, x1
        x2, y2 = y2, x2
    rev = False
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        rev = True
    deltax = x2 - x1
    deltay = abs(y2-y1)
    error = int(deltax / 2)
    y = y1
    ystep = None
    if y1 < y2:
        ystep = 1
    else:
        ystep = -1
    for x in range(x1, x2 + 1):
        if issteep:
            points.append((y, x))
        else:
            points.append((x, y))
        error -= deltay
        if error < 0:
            y += ystep
            error += deltax
    # Reverse the list if the coordinates were reversed
    if rev:
        points.reverse()
    return points

def get_line_float(x1, y1, x2, y2, steps=80):
    steps = float(steps)
    xminus = np.min((x1, x2))
    yminus = np.min((y1, y2))
    xdivide = np.abs(x2-x1)/steps
    ydivide = np.abs(y2-y1)/steps
    newcoords = [int((x1-xminus)/xdivide), int((y1-yminus)/ydivide), int((x2-xminus)/xdivide), int((y2-yminus)/ydivide)]
    nx1, ny1, nx2, ny2 = newcoords
    opoints = np.array(get_line(nx1, ny1, nx2, ny2))
    points = opoints.astype(float)
    points[:, 0] *= xdivide
    points[:, 0] += xminus
    points[:, 1] *= ydivide
    points[:, 1] += yminus
    return points
