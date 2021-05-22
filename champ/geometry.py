import time
import math
import numpy as np


def vectorize(vector, scalar):
    if isinstance(scalar, int) or isinstance(scalar, float):
        n, _ = vector.shape
        new_vector = np.zeros((n,1))
        new_vector[:, 0] = scalar
        return new_vector
    else:
        return scalar

def translate(vector, translation):
    vector += translation

    return vector

def translate_x(vector, translation):
    translation = vectorize(vector, translation)
    vector[:, 0] += translation[:,0]

    return vector

def translate_y(vector, translation):
    translation = vectorize(vector, translation)
    vector[:, 1] += translation[:,0]
    
    return vector

def translate_z(vector, translation):
    translation = vectorize(vector, translation)
    vector[:, 2] += translation[:,0]
    
    return vector

def rotate_x(vector, theta):
    theta = vectorize(vector, theta)

    cos_theta = np.cos(theta[:, 0])
    sin_theta = np.sin(theta[:, 0])
    
    y = vector[:, 1] * cos_theta - vector[:, 2] * sin_theta
    z = vector[:, 1] * sin_theta + vector[:, 2] * cos_theta

    vector[:, 1] = y
    vector[:, 2] = z

    return vector

def rotate_y(vector, theta):
    theta = vectorize(vector, theta)

    cos_theta = np.cos(theta[:, 0])
    sin_theta = np.sin(theta[:, 0])

    x =  vector[:, 0] * cos_theta + vector[:, 2] * sin_theta
    z = -vector[:, 0] * sin_theta + vector[:, 2] * cos_theta

    vector[:, 0] = x
    vector[:, 2] = z
  
    return vector

def rotate_z(vector, theta):
    theta = vectorize(vector, theta)

    cos_theta = np.cos(theta[:, 0])
    sin_theta = np.sin(theta[:, 0])

    x = vector[:, 0] * cos_theta - vector[:, 1] * sin_theta
    y = vector[:, 0] * sin_theta + vector[:, 1] * cos_theta

    vector[:, 0] = x
    vector[:, 1] = y

    return vector

def quat_to_matrix(x=0, y=0, z=0, w=1):
    rotation_matrix = np.zeros((3,3))

    w_squared = w * w
    y_z = y * z
    x_y = x * y
    x_z = x * z
    w_x = w * x
    w_y = w * y
    w_z = w * z

    rotation_matrix[0, 0] = 2 * (w_squared + x * x) - 1
    rotation_matrix[0, 1] = 2 * (x_y - w_z)
    rotation_matrix[0, 2] = 2 * (x_z + w_y)
     
    rotation_matrix[1, 0] = 2 * (x_y + w_z)
    rotation_matrix[1, 1] = 2 * (w_squared + y * y) - 1
    rotation_matrix[1, 2] = 2 * (y_z - w_x)
     
    rotation_matrix[2, 0] = 2 * (x_z - w_y)
    rotation_matrix[2, 1] = 2 * (y_z + w_x)
    rotation_matrix[2, 2] = 2 * (w_squared + z * z) - 1
                            
    return rotation_matrix

def rpy_to_matrix(roll=0, pitch=0, yaw=0):
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    cos_pitch = math.cos(pitch)
    sin_pitch = math.sin(pitch)
    cos_roll = math.cos(roll)
    sin_roll = math.sin(roll)

    yaw_r = np.array([[cos_yaw, -sin_yaw,   0],
                      [sin_yaw,  cos_yaw,   0],
                      [       0,       0,   1]])

    pitch_r = np.array([[ cos_pitch,   0, sin_pitch],
                        [         0,   1,         0],
                        [-sin_pitch,   0, cos_pitch]])

    roll_r = np.array([[  1,        0,         0],
                       [  0, cos_roll, -sin_roll],
                       [  0, sin_roll,  cos_roll]])

    return yaw_r * pitch_r * roll_r