import numpy as np


class Point:
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z


class Euler:
    def __init__(self, roll=0, pitch=0, yaw=0):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw


class Quat:
    def __init__(self, w=1, x=0, y=0, z=0):
        self.w = w
        self.x = x
        self.y = y
        self.z = z


class Pose:
    def __init__(self):
        self.position = Point()
        self.orientation = Euler()


class Linear:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class Angular:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class Twist:
    def __init__(self):
        self.linear = Linear()
        self.angular = Angular()


class PoseVector:
    def __init__(self):
        self._position = np.zeros(3)
        self._orientation = np.zeros(4)
        self._orientation[3] = 1.0

    @property
    def position(self):
        return self._position

    @position.setter
    def position(self, position):
        self._position[0] = position[0]
        self._position[1] = position[1]
        self._position[2] = position[2]

    @property
    def orientation(self):
        return self._orientation

    @orientation.setter
    def orientation(self, orientation):
        self._orientation[0] = orientation[0]
        self._orientation[1] = orientation[1]
        self._orientation[2] = orientation[2]
        self._orientation[3] = orientation[3]


class TwistVector:
    def __init__(self):
        self._linear = np.zeros(3)
        self._angular = np.zeros(3)

    @property
    def linear(self):
        return self._linear

    @linear.setter
    def linear(self, linear):
        self._linear[0] = linear[0]
        self._linear[1] = linear[1]
        self._linear[2] = linear[2]

    @property
    def angular(self):
        return self._angular

    @angular.setter
    def angular(self, angular):
        self._angular[0] = angular[0]
        self._angular[1] = angular[1]
        self._angular[2] = angular[2]