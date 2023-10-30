import numpy as np
from numpy import inf, matrix, cos, arctan2, sqrt, pi, sin, cos
from numpy.random import randn
from scipy.optimize import minimize, Bounds, LinearConstraint, NonlinearConstraint, BFGS, basinhopping
from scipy.spatial import ConvexHull, convex_hull_plot_2d
def extend_waypoints_set(set_):
    set_length = len(set_)
    # begin
    d_begin = set_[1] - set_[0]
    set_begin = set_[0] - d_begin
    
    # end
    d_end = set_[set_length-1] - set_[set_length-2]
    set_end_1 = set_[set_length-1] + d_end
    set_end_2 = set_[set_length-1] + 2*d_end
    
    # extend
    set_new = np.vstack(( set_begin,set_))
    set_new = np.vstack(( set_new, set_end_1))
    set_new = np.vstack(( set_new, set_end_2))


    return set_new


def M_spline_from_set(set_):
    set_length = len(set_)
    set_extend = extend_waypoints_set(set_)
    
    M = np.array([[1, -3, 3, -1],
                  [4, 0, -6, 3],
                  [1, 3, 3, -3],
                  [0, 0, 0, 1]
                ])
    M = M / 6
    spline = []
    for i in range(1, set_length):
        ri = np.hstack((set_extend[i-1].reshape((3,1)), set_extend[i].reshape((3,1)), set_extend[i+1].reshape((3,1)), set_extend[i+2].reshape((3,1))))
        si = np.dot(ri,M)
        spline.append(si)

    return np.array(spline)

def spline_point(cfs, s):
    x = cfs[0, 3] * s*s*s + cfs[0, 2] * s*s + cfs[0, 1] * s + cfs[0, 0]
    y = cfs[1, 3] * s*s*s + cfs[1, 2] * s*s + cfs[1, 1] * s + cfs[1, 0]
    z = cfs[2, 3] * s*s*s + cfs[2, 2] * s*s + cfs[2, 1] * s + cfs[2, 0]    
    return np.array([x, y, z])

def spline_point_derv(cfs, s):
    x = 3*cfs[0, 3] * s*s + 2*cfs[0, 2] * s + cfs[0, 1]
    y = 3*cfs[1, 3] * s*s + 2*cfs[1, 2] * s + cfs[1, 1]
    z = 3*cfs[2, 3] * s*s + 2*cfs[2, 2] * s + cfs[2, 1]
    return np.array([x, y, z])

def spline_point_double_derv(cfs, s):
    x = 6*cfs[0, 3] * s + 2*cfs[0, 2]
    y = 6*cfs[1, 3] * s + 2*cfs[1, 2]
    z = 6*cfs[2, 3] * s + 2*cfs[2, 2]
    return np.array([x, y, z])

def spline_projection_criteria(cfs,  s,  y):
    res = (y - spline_point(cfs ,s)).dot(spline_point_derv(cfs, s))
    return res


def spline_projection_criteria_derv(cfs, s, y):
    res1 = -spline_point_derv(cfs, s).dot(spline_point_derv(cfs ,s))
    res2 = (y - spline_point(cfs ,s)).dot(spline_point_double_derv(cfs ,s))
    return res1 + res2

def spline_find_point_state2d(cfs, s):
    derv_vector = spline_point_derv(cfs, s)
    point = spline_point(cfs, s)
    fi = np.arctan2(derv_vector[1],derv_vector[0])
    state = np.array([point[0], point[1], fi])
    return state

def spline_seed(spline_set, delta):
    points = []
    for spline in spline_set:
        elem_length = np.linalg.norm(spline_point(spline, 0.0) - spline_point(spline, 1.0))
        #print("elem_length: ", elem_length)
        length = 0
        while(length < elem_length):
            s = length/elem_length
            p = spline_point(spline, s)
            points.append(p)
            length+=delta
        points.append(spline_point(spline, 1.0))
    points = np.array(points)
    return points

def spline_length(cfg):
    x_a = spline_point(cfg, 0)
    x_b = spline_point(cfg, 1.0)
    return np.linalg.norm(x_a -  x_b)

def spline_content_point(y, spline):
    N_a = spline_point_derv(spline, 0)
    x_a = spline_point(spline, 0)
    N_b = spline_point_derv(spline, 1)
    x_b = spline_point(spline, 1)
    projection1 = N_a.dot(y-x_a)
    projection2 = N_b.dot(y-x_b)
    if(projection1 > 0) and (projection2 < 0):
        return True
    # if(projection1 == 0) and (projection2 < 0):
    #     print("внутри")
        #return True
    return False

def spline_param_of_point_projection(y, cfs):
    eps = 1e-3
    slim = 1.0
    step = slim / 5.0
    for s in [0, 0.2, 0.4, 0.6, 0.8]:
        x0 = s
        fx0 = 0.0
        fx0_dot = 0.0
        for i in range(5):
            fx0 = spline_projection_criteria(cfs ,x0, y)
            fx0_dot = spline_projection_criteria_derv(cfs ,x0, y)
            x0 = x0 - fx0/fx0_dot
            
            if (abs(fx0) < eps and x0 > -eps and x0 < slim+eps):
                return x0
    #return x0    
    return -1


