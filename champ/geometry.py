import time
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