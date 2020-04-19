import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import random

class LiveTrajectoryPlot:
    def __init__(self):
        self.estimated_traj = {'x':[],'y':[],'z':[]}
        self.true_traj = {'x':[],'y':[],'z':[]}
        plt.ion()
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')

    def updateEstimatedTraj(self, x, y, z):
        self.estimated_traj['x'].append(x)
        self.estimated_traj['y'].append(y)
        self.estimated_traj['z'].append(z)

    def updateTrueTraj(self, x, y, z):
        self.true_traj['x'].append(x)
        self.true_traj['y'].append(y)
        self.true_traj['z'].append(z)

    def render(self, delay=0.001, text=''):
        self.ax.cla()
        self.fig.suptitle(text)
        self.ax.plot(self.estimated_traj['x'], self.estimated_traj['y'], self.estimated_traj['z'], color='blue', label='estimated trajectory',alpha=0.5)
        self.ax.plot(self.true_traj['x'], self.true_traj['y'], self.true_traj['z'], color='red', label='ground truth trajectory',alpha=0.5)
        self.ax.legend()
        plt.draw()
        plt.pause(delay)
    
    def close(self):
        plt.close(self.fig)


    # import random
    # from itertools import count
    # from matplotlib.animation import FuncAnimation
    # from mpl_toolkits.mplot3d import axes3d
    # import matplotlib.pyplot as plt
    # plt.style.use('fivethirtyeight')
    # x_vals = []
    # y_vals = []
    # plt.plot(x_vals, y_vals)

    # index = count()
    # def animate(i):
    #     x_vals.append(next(index))
    #     y_vals.append(random.randint(0,5))
    #     plt.cla()
    #     plt.plot(x_vals, y_vals)

    # ani = FuncAnimation(plt.gcf(), animate, interval=100)
    # plt.tight_layout()
    # plt.show()