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


class Velocities:
    def __init__(self):
        self.linear = Linear()
        self.angular = Angular()
