import time
import numpy as np

def vectorize_theta(vector, theta):
    if isinstance(theta, int) or isinstance(theta, float):
        n, _ = vector.shape
        new_theta = np.zeros((n,1))
        new_theta[:,0] = theta
        return new_theta
    else:
        return theta

def translate(vector, translation):
    vector += translation

    return vector

def translate_x(vector, translation):
    vector[:, 0] += translation

    return vector

def translate_y(vector, translation):
    vector[:, 1] += translation
    
    return vector

def translate_z(vector, translation):
    vector[:, 2] += translation
    
    return vector

def rotate_x(vector, theta):
    theta = vectorize_theta(vector, theta)

    cos_theta = np.cos(theta[:, 0])
    sin_theta = np.sin(theta[:, 0])
    
    y = vector[:, 1] * cos_theta - vector[:, 2] * sin_theta
    z = vector[:, 1] * sin_theta + vector[:, 2] * cos_theta

    vector[:, 1] = y
    vector[:, 2] = z

    return vector

def rotate_y(vector, theta):
    theta = vectorize_theta(vector, theta)

    cos_theta = np.cos(theta[:, 0])
    sin_theta = np.sin(theta[:, 0])

    x =  vector[:, 0] * cos_theta + vector[:, 2] * sin_theta
    z = -vector[:, 0] * sin_theta + vector[:, 2] * cos_theta

    vector[:, 0] = x
    vector[:, 2] = z
  
    return vector

def rotate_z(vector, theta):
    theta = vectorize_theta(vector, theta)

    cos_theta = np.cos(theta[:, 0])
    sin_theta = np.sin(theta[:, 0])

    x = vector[:, 0] * cos_theta - vector[:, 1] * sin_theta
    y = vector[:, 0] * sin_theta + vector[:, 1] * cos_theta

    vector[:, 0] = x
    vector[:, 1] = y

    return vector