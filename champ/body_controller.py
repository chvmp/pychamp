import copy
import math
import time
import numpy as np
from champ.geometry import *


class PoseGenerator:
    def __init__(self, amp=1.5, freq=0.5, phase_shift=0.0):
        self._amplitude = amp
        self._frequency = freq
        self._start = 0.0
        self._elapsed = 0.0
        self._phase_shift = phase_shift

    def sine(self, now=None):
        if now is None:
            now = time.time()

        self._elapsed = now - self._start
        if self._elapsed >= (1.0 / self._frequency):
            self._start = now
            self._elapsed = 0.0

        t = self._elapsed / (1.0 / self._frequency)

        return self._amplitude * math.sin((2.0 * math.pi * t) + self._phase_shift)

class BodyController(object):
    def __init__(self, base, com_x_translation=0.0):
        self._base = base
        self._zero_stances = base.legs.zero_stances
        self._com_x_translation = com_x_translation

    def pose_command(self, req_pose):
        foot_positions = copy.deepcopy(self._zero_stances)
        
        req_translation_x = -req_pose.position.x - self._com_x_translation
        req_translation_y = -req_pose.position.y
        req_translation_z = -(foot_positions[0, 2] + req_pose.position.z)
        #cap the translation in z axis to a 65 percent of the robot's leg length
        max_translation_z = -foot_positions[0, 2] * 0.65
        if req_translation_z < 0.0:
            req_translation_z = 0.0
        elif req_translation_z > max_translation_z:
            req_translation_z = max_translation_z
        
        #translate the base in the X, Y and Z axis
        translate(foot_positions, [[req_translation_x,
                                    req_translation_y,
                                    req_translation_z]])
        
        #rotate the base in X, Y, and Z axis
        rotate_z(foot_positions, -req_pose.orientation.yaw)
        rotate_y(foot_positions, -req_pose.orientation.pitch)
        rotate_x(foot_positions, -req_pose.orientation.roll)

        return self._base.transform_to_hip(foot_positions)
