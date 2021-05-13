import copy
import numpy as np
from champ.types import Point, Euler
from champ.geometry import *

class Joint(object):
    def __init__(self, parent, id):
        self._parent = parent
        self._id = id
        self._translation = np.zeros((1,3))
        self._rotation = np.zeros((1,3))
        self._theta = 0
        self._name = ''

    @property
    def translation(self):
        return self._translation

    @property
    def rotation(self):
        return self._rotation

    @property
    def theta(self):
        return self._theta

    @property
    def position(self):
        position = np.zeros((1,3))
        for i in range(self._id, -1, -1):
            position = translate(position, self._parent.joints[i].translation)
            if i > 1:
                position = rotate_y(position, self._parent.joints[i-1].theta)
            elif i == 1:
                position = rotate_x(position, self._parent.joints[i-1].theta)

        return position

    #localhip
    # @property
    # def position(self):
    #     position = np.zeros((1,3))
    #     for i in range(self._id, 0, -1):
    #         if i > 0:
    #             position = translate(position, self._parent.joints[i].translation)
    #         if i > 1:
    #             position = rotate_y(position, self._parent.joints[i-1].theta)
    #         elif i == 1:
    #             position = rotate_x(position, self._parent.joints[i-1].theta)

    #     return position

    @theta.setter
    def theta(self, angle):
        self._theta = angle

    def set_translation(self, x, y, z):
        self._translation[:, 0] = x
        self._translation[:, 1] = y
        self._translation[:, 2] = z
        self._translation.flags.writeable = False

    def set_rotation(self, roll, pitch, yaw):
        self._rotation[:, 0] = roll
        self._rotation[:, 1] = pitch
        self._rotation[:, 2] = yaw
        self._rotation.flags.writeable = False

    def set_origin(self, x, y, z, roll=0.0, pitch=0.0, yaw=0.0, name=''):
        self._name = name
        self._translation[:, 0] = x
        self._translation[:, 1] = y
        self._translation[:, 2] = z
        self._rotation[:, 0] = roll
        self._rotation[:, 1] = pitch
        self._rotation[:, 2] = yaw
        self._translation.flags.writeable = False
        self._rotation.flags.writeable = False

    @property
    def name(self):
        return self._name

    @property
    def x(self):
        return self._translation[:, 0]
    
    @property
    def y(self):
        return self._translation[:, 1]

    @property
    def z(self):
        return self._translation[:, 2]

    @property
    def roll(self):
        return self._rotation[:, 0]
    
    @property
    def pitch(self):
        return self._rotation[:, 1]

    @property
    def yaw(self):
        return self._rotation[:, 2]


class JointVector:
    def __init__(self, parent, id):
        self._parent = parent
        self._id = id
        self._translation = np.zeros((4,3))
        self._vectorized = False
        self._theta = np.zeros((4,1))

    @property
    def translation(self):
        if not self._vectorized:
            self._vectorized = True
            for i in range(4):
                self._translation[i, :] = self._parent.legs[i].joints[self._id].translation

            self._translation.flags.writeable = False

        return self._translation

    @property 
    def position(self):
        position = np.zeros((4,3))
        for i in range(self._id, -1, -1):
            position = translate(position, self._parent.joints[i].translation)
            if i > 1:
                position = rotate_y(position, self._parent.joints[i-1].theta)
            elif i == 1:
                position = rotate_x(position, self._parent.joints[i-1].theta)

        return position

    @property
    def theta(self):
        return self._theta

    @theta.setter
    def theta(self, theta):
        self._theta = theta