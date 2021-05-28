import copy
import numpy as np
from champ.types import Point, Euler
from champ.geometry import *


class Translation(object):
    def __init__(self, parent, id):
        self._parent = parent
        self._id = id
        self._translation = np.zeros((4,3))
    
    @property
    def xyz(self):
        return self._translation

    @property
    def x(self):
        return self._translation[:, 0].reshape(4,1)

    @property
    def y(self):
        return self._translation[:, 1].reshape(4,1)

    @property
    def z(self):
        return self._translation[:, 2].reshape(4,1)

    def set_translation(self, leg_id, x, y, z):
        self._translation[leg_id, 0] = x
        self._translation[leg_id, 1] = y
        self._translation[leg_id, 2] = z

class Joint(object):
    def __init__(self, parent, id):
        self.translation = Translation(parent, id)
        self._parent = parent
        self._id = id
        self._theta = np.zeros((4,1))
        self._velocity = np.zeros((4,1))

    @property 
    def position(self):
        position = np.zeros((4,3))
        for i in range(self._id, -1, -1):
            position = translate(position, self._parent.joints[i].translation.xyz)
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

    @property
    def velocity(self):
        return self._velocity

    @velocity.setter
    def velocity(self, velocity):
        self._velocity = velocity

    def set_origin(self, leg_id, x, y, z):
        self.translation.set_translation(leg_id, x, y, z)
