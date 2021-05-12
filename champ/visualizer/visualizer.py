import matplotlib
from matplotlib import pyplot as plt
import numpy as np
matplotlib.use('TkAgg')

class Visualizer:
    def __init__(self, base, x_min=-0.15, x_max=0.15, y_min=-0.25, y_max=0.1, add_directional_count=False, latch_buffer_size=100):
        self._latch_buffer_size = latch_buffer_size
        self._base = base
        self._fig, self._ax = plt.subplots()
        self._ax.set_aspect('equal')
        self._ax.set_xlim(x_min, x_max)
        self._ax.set_ylim(y_min, y_max)

        x = np.empty(0)
        y = np.empty(0)

        vec = np.empty((0, 3))

        (self._points,) = self._ax.plot(x, y, 'gh', animated=True)
        (self._legs,) = self._ax.plot(vec[:, 0], vec[:, 1])
        (self._joints,) = self._ax.plot(vec[:, 0], vec[:, 1], 'gh', markersize=10)
        (self._foot,) = self._ax.plot(vec[:, 0], vec[:, 1], 'or', markersize=1)
        self._latch_pool = np.empty((self._latch_buffer_size,3))
        self._init = False

    def init(self):
        self._init = True
        plt.grid(color='gray', linestyle='-', linewidth=0.1)
        plt.show(block=False)
        plt.pause(0.1)
        
        self._background = self._fig.canvas.copy_from_bbox(self._fig.bbox)
        self._ax.draw_artist(self._points)
        self._fig.canvas.blit(self._fig.bbox)

    def plot_leg(self, id, view='s', latch=100):
        if latch > self._latch_buffer_size:
            latch = self._latch_buffer_size

        if not self._init:
            self.init()
     
        self._fig.canvas.restore_region(self._background)

        x_idx, y_idx = self.__view_to_ids(view)

        if id > 3:
            print("Invalid Leg ID")
            self._fig.canvas.flush_events()
            return

        legs = np.empty((0,3))
        legs = np.append(legs, self._base.legs[id].upper_leg_from_hip, axis=0)
        legs = np.append(legs, self._base.legs[id].lower_leg_from_hip, axis=0)
        legs = np.append(legs, self._base.legs[id].foot_from_hip, axis=0)

        to_render = []
        if latch:
            self._latch_pool[0,] = legs[2,]
            self._latch_pool = np.roll(self._latch_pool, 1, axis=0)
            
            self._foot.set_data(self._latch_pool[:latch-1, x_idx], self._latch_pool[:latch-1, y_idx])
            to_render.append(self._foot)

        self._legs.set_data(legs[:, x_idx], legs[:, y_idx])
        self._joints.set_data(legs[:, x_idx], legs[:, y_idx])
        to_render.append(self._legs)
        to_render.append(self._joints)

        self.__render(to_render)

    def __render(self, render_list):
        for obj in render_list:
            self._ax.draw_artist(obj)
            
        self._fig.canvas.blit(self._fig.bbox)
        self._fig.canvas.flush_events()

    def __view_to_ids(self, view):
        if view == 's':
            x_idx = 0
            y_idx = 2
        elif view == 'f':
            x_idx = 1
            y_idx = 2
        elif view == 't':
            x_idx = 0
            y_idx = 1
        else:
            x_idx = 0
            y_idx = 2

        return x_idx, y_idx