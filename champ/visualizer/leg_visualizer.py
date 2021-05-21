import matplotlib
from matplotlib import pyplot as plt
import numpy as np
matplotlib.use('TkAgg')

class LegVisualizer:
    def __init__(self, base, x_min=-0.2, x_max=0.2, y_min=-0.25, y_max=0.1, latch_buffer_size=100):
        self._latch_buffer_size = latch_buffer_size
        self._base = base
        self._fig, self._ax = plt.subplots(1,3, figsize=(12, 5))

        self._joints = [None, None, None]
        self._legs = [None, None, None]
        self._foot = [None, None, None]
        self._latch_pool = [None, None, None]
        
        for i in range(3):
            self._ax[i].set_aspect('equal')
            self._ax[i].grid()

            self._ax[i].set_xlim(x_min, x_max)
            self._ax[i].set_ylim(y_min, y_max)
            self._ax[i].grid
            x = np.empty(0)
            y = np.empty(0)

            vec = np.empty((0, 3))

            (self._legs[i],) = self._ax[i].plot(vec[:, 0], vec[:, 1])
            (self._joints[i],) = self._ax[i].plot(vec[:, 0], vec[:, 1], 'gh', markersize=10)
            (self._foot[i],) = self._ax[i].plot(vec[:, 0], vec[:, 1],'ro', markersize=1)
            self._latch_pool[i] = np.empty((self._latch_buffer_size, 3))
        
        self._init = False

    def init(self):
        self._init = True
        plt.show(block=False)
        plt.pause(0.1)
        self._background = self._fig.canvas.copy_from_bbox(self._fig.bbox)
        self._fig.canvas.blit(self._fig.bbox)

    def plot_legs(self, id=0, latch=0):
        if latch > self._latch_buffer_size:
            latch = self._latch_buffer_size

        if not self._init:
            self.init()
     
        self._fig.canvas.restore_region(self._background)
        self._fig.canvas.flush_events()

        legs = np.empty((0,3))
        for i in range(1, 4):
            leg_pos = self._base.joints[i].position[i, :].reshape(1,3)
            leg_pos = self._base.transform_to_hip(leg_pos, leg_id=id)
            legs = np.append(legs, leg_pos, axis=0)

        views = ['side', 'front', 'top']
        for i in range(3):
            to_render = []
            x_idx, y_idx = self.__view_to_ids(views[i])

            if latch > 0:
                self._latch_pool[i] = np.roll(self._latch_pool[i], 1, axis=0)
                self._latch_pool[i][0,] = legs[2,]
                
                self._foot[i].set_data(self._latch_pool[i][:latch-1, x_idx], self._latch_pool[i][:latch-1, y_idx])
                to_render.append(self._foot[i])
                
            self._legs[i].set_data(legs[:, x_idx], legs[:, y_idx])
            self._joints[i].set_data(legs[:, x_idx], legs[:, y_idx])
            to_render.append(self._legs[i])
            to_render.append(self._joints[i])

            self.__render(i, to_render)
        
        self._fig.canvas.blit(self._fig.bbox)
        self._fig.canvas.flush_events()

    def __render(self, window_id, render_list):
        x_idx, y_idx = self.__view_to_ids(window_id)
        x_max = np.max(self._joints[window_id].get_data()[0])
        y_max = np.max(self._joints[window_id].get_data()[1])

        for obj in render_list:
            self._ax[window_id].draw_artist(obj)
            
    def __view_to_ids(self, view):
        if view == 'side':
            x_idx = 0
            y_idx = 2
        elif view == 'front':
            x_idx = 1
            y_idx = 2
        elif view == 'top':
            x_idx = 0
            y_idx = 1
        else:
            x_idx = 0
            y_idx = 2

        return x_idx, y_idx