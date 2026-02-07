import collections
import matplotlib.pyplot as plt
import numpy as np
import time

STEPS = 100
TIME_INTERVAL = 0.01 # [s]

class Plotter:
    def __init__(self, steps=STEPS, time_interval=TIME_INTERVAL):
        self.time_interval = time_interval

        t = np.arange(0, steps)
        self.values = [collections.deque(np.zeros(t.shape)) for i in range(6)]
        self.last_value = [0.0 for i in range(6)]
        self.limits = [(0.0, 0.0) for i in range(2)]

        self.fig, self.axes = plt.subplots(1, 2)

        self.axes[0].set_title('forces')
        self.axes[0].set_animated(True)

        self.axes[1].set_title('moments')
        self.axes[1].set_animated(True)

        (self.ln_fx,) = self.axes[0].plot(self.values[0], label='x', color='red')
        (self.ln_fy,) = self.axes[0].plot(self.values[1], label='y', color='green')
        (self.ln_fz,) = self.axes[0].plot(self.values[2], label='z', color='blue')
        (self.ln_mx,) = self.axes[1].plot(self.values[3], label='x', color='red')
        (self.ln_my,) = self.axes[1].plot(self.values[4], label='y', color='green')
        (self.ln_mz,) = self.axes[1].plot(self.values[5], label='z', color='blue')

        plt.show(block=False)
        plt.pause(0.1)

        self.bg = self.fig.canvas.copy_from_bbox(self.fig.bbox)

        self.fig.draw_artist(self.axes[0])
        self.fig.draw_artist(self.axes[1])

        # https://matplotlib.org/stable/users/explain/animations/blitting.html + https://stackoverflow.com/a/15724978
        self.fig.canvas.blit(self.fig.bbox)

    def update_forces(self, forces):
        self.last_value[0:3] = forces
        self.limits[0] = (min([self.limits[0][0]] + self.last_value[0:3]), max([self.limits[0][1]] + self.last_value[0:3]))

    def update_moments(self, moments):
        self.last_value[3:6] = moments
        self.limits[1] = (min([self.limits[1][0]] + self.last_value[3:6]), max([self.limits[1][1]] + self.last_value[3:6]))

    def update(self, values):
        self.last_value = values
        self.limits[0] = (min([self.limits[0][0]] + self.last_value[0:3]), max([self.limits[0][1]] + self.last_value[0:3]))
        self.limits[1] = (min([self.limits[1][0]] + self.last_value[3:6]), max([self.limits[1][1]] + self.last_value[3:6]))

    def plot(self):
        for i in range(len(self.values)):
            self.values[i].popleft()
            self.values[i].append(self.last_value[i])

        self.fig.canvas.restore_region(self.bg)

        self.ln_fx.set_ydata(self.values[0])
        self.ln_fy.set_ydata(self.values[1])
        self.ln_fz.set_ydata(self.values[2])
        self.ln_mx.set_ydata(self.values[3])
        self.ln_my.set_ydata(self.values[4])
        self.ln_mz.set_ydata(self.values[5])

        self.axes[0].set_ylim(self.limits[0][0], self.limits[0][1])
        self.axes[1].set_ylim(self.limits[1][0], self.limits[1][1])

        self.fig.draw_artist(self.axes[0])
        self.fig.draw_artist(self.axes[1])

        self.fig.canvas.blit(self.fig.bbox)
        self.fig.canvas.flush_events()

        time.sleep(self.time_interval)
