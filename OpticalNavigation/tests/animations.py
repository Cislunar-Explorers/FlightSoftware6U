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

    def renderUKF(self, delay=0.001, text=''):
        self.ax.cla()
        self.fig.suptitle(text)
        self.ax.plot(self.estimated_traj['x'], self.estimated_traj['y'], self.estimated_traj['z'], color='blue', label='estimated trajectory',alpha=0.5)
        self.ax.plot(self.true_traj['x'], self.true_traj['y'], self.true_traj['z'], color='red', label='ground truth trajectory',alpha=0.5)
        self.ax.set_xlabel('$X$', fontsize=20, rotation=150)
        self.ax.set_ylabel('$Y$', fontsize=20)
        self.ax.set_zlabel('$Z$', fontsize=20, rotation=60)
        self.ax.legend()
        plt.draw()
        plt.pause(delay)

    def preRender(self, text=''):
        self.ax.cla()
        self.fig.suptitle(text)

    def render(self, xs, ys, zs, label='', color='blue', marker='o'):
        self.ax.plot(xs, ys, zs, color=color, label=label,alpha=0.5, marker=marker)

    def postRender(self, delay=0.001):
        self.ax.set_xlabel('$X$', fontsize=20, rotation=150)
        self.ax.set_ylabel('$Y$', fontsize=20)
        self.ax.set_zlabel('$Z$', fontsize=20, rotation=60)
        self.ax.legend()
        plt.draw()
        plt.pause(delay)
    
    def close(self):
        plt.close(self.fig)