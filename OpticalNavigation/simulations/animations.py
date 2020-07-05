import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d
import random
import numpy as np

class Arrow3D(FancyArrowPatch):
    def __init__(self, xs, ys, zs, *args, **kwargs):
        FancyArrowPatch.__init__(self, (0,0), (0,0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
        self.set_positions((xs[0],ys[0]),(xs[1],ys[1]))
        FancyArrowPatch.draw(self, renderer)

def testArrow(): 
    fig = plt.figure(figsize=(15,15))
    ax = fig.add_subplot(111, projection='3d')
    a = Arrow3D([0, 1], [0,0], 
                [0,0], mutation_scale=20, 
                lw=3, arrowstyle="-|>", color="r")
    ax.add_artist(a)
    ax.set_xlabel('x_values')
    ax.set_ylabel('y_values')
    ax.set_zlabel('z_values')

    plt.title('Eigenvectors')

    plt.draw()
    plt.show()

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

class LiveMultipleTrajectoryPlot:
    def __init__(self, size):
        self.trajectories = []
        self.settings = []
        for i in range(size):
            self.trajectories.append({'x':[],'y':[],'z':[]})
            self.settings.append({'color':'','label':'','alpha':''})

        plt.ion()
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')

    def setTrajectorySettings(self, i, color, label, alpha):
        self.settings[i]['color'] = color
        self.settings[i]['label'] = label
        self.settings[i]['alpha'] = alpha


    def updateTraj(self, i, x, y, z):
        self.trajectories[i]['x'].append(x)
        self.trajectories[i]['y'].append(y)
        self.trajectories[i]['z'].append(z)

    def renderUKF(self, delay=0.001, text=''):
        self.ax.cla()
        self.fig.suptitle(text)
        for i in range(len(self.trajectories)):
            traj = self.trajectories[i]
            settings = self.settings[i]
            self.ax.plot(traj['x'], traj['y'], traj['z'], color=settings['color'], label=settings['label'],alpha=settings['alpha'])

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
