import numpy as np
import copy
from champ.geometry import *


def project_foot(foot_positions, step_x, step_y, theta):
    trans = np.zeros((4,3))
    trans[:,0] = step_x
    trans[:,1] = step_y

    projected_foot = copy.deepcopy(foot_positions)
    projected_foot = translate(projected_foot, trans)
    projected_foot = rotate_z(projected_foot, theta)
    
    return projected_foot


def raibert_heuristic(req_vel, stance_duration, center_to_nominal=None):
    theta = 0
    step_x = (stance_duration / 2.0) * req_vel.linear.x
    step_y = (stance_duration / 2.0) * req_vel.linear.y

    if center_to_nominal is not None:
        tangential_velocity = req_vel.angular.z * center_to_nominal
        arc = (stance_duration / 2.0) * tangential_velocity
        theta = arc / center_to_nominal

    return step_x, step_y, theta