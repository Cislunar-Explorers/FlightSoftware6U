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
    """
    Plot trajectory and attitude data
    For arrow styles, visit: https://matplotlib.org/3.1.1/api/_as_gen/matplotlib.patches.ArrowStyle.html
    For line styles (ls), visit: https://matplotlib.org/3.1.1/api/_as_gen/matplotlib.patches.Arrow.html#matplotlib.patches.Arrow
    """
    def __init__(self, trajectories=1, trackingArrows=1, bounds=None):
        self.trajectories = []
        self.settings = []
        self.trackingArrows = []
        self.trackingArrowsSettings = []
        self.traceLimits = []
        self.leadingBlob = []
        for i in range(trajectories):
            self.trajectories.append({'x':[],'y':[],'z':[]})
            self.settings.append({'color':'','label':'','alpha':'','ls':''})
            self.traceLimits.append(None)
            self.leadingBlob.append({'size':0, 'shape':'0', 'color':'red', 'alpha':0, 'label':'N/A'})
        for i in range(trackingArrows):
            self.trackingArrows.append({'x':[0,0], 'y':[0,0], 'z':[0,0]})
            self.trackingArrowsSettings.append({'mutation_scale':20, 'lw':2, 'arrowstyle':"-|>", 'color':"r", 'ls':'-'})

        plt.ion()
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.bounds = bounds;

    # def setSimulationBounds(self, ):
    #     self.ax.set_aspect('auto')


    #     self.ax.set_xbound(lower=xmin, upper=xmax)
    #     self.ax.set_ybound(lower=ymin, upper=ymax)
    #     self.ax.set_zbound(lower=zmin, upper=zmax)

    def setTrajectorySettings(self, i, color, label, alpha, ls):
        """
        https://matplotlib.org/3.2.1/api/_as_gen/matplotlib.axes.Axes.plot.html
        '-' 	solid line style
        '--' 	dashed line style
        '-.' 	dash-dot line style
        ':' 	dotted line style
        """
        if color is None or label is None or alpha is None or ls is None:
            self.settings[i] = None
        else:
            self.settings[i]['color'] = color
            self.settings[i]['label'] = label
            self.settings[i]['alpha'] = alpha
            self.settings[i]['ls'] = ls

    def updateTraj(self, i, x, y, z):
        self.trajectories[i]['x'].append(x)
        self.trajectories[i]['y'].append(y)
        self.trajectories[i]['z'].append(z)

    def updateAtt(self, i, start_pos, new_vector):
        start_pos = start_pos.flatten()
        new_vector = new_vector.flatten()
        xbounds = self.ax.get_xlim3d()
        ybounds = self.ax.get_ylim3d()
        zbounds = self.ax.get_zlim3d()
        scaleX = np.abs(xbounds[1] - xbounds[0])
        scaleY = np.abs(ybounds[1] - ybounds[0])
        scaleZ = np.abs(zbounds[1] - zbounds[0])
        scale = max((scaleX/10 + scaleY/10 + scaleZ/10) / 3, (scaleX/5 + scaleY/5 + scaleZ/5) / 3)
        # scale = np.min([scaleX, scaleY, scaleZ])
        end_pos = start_pos + new_vector * scale;
        self.trackingArrows[i]['x'][0] = start_pos[0]
        self.trackingArrows[i]['x'][1] = end_pos[0]
        self.trackingArrows[i]['y'][0] = start_pos[1]
        self.trackingArrows[i]['y'][1] = end_pos[1]
        self.trackingArrows[i]['z'][0] = start_pos[2]
        self.trackingArrows[i]['z'][1] = end_pos[2]

    def setTrackingArrowSettings(self, i, mutation_scale=20, lw=2, ls='-', style="-|>", color="r"):
        self.trackingArrowsSettings[i]['mutation_scale'] = mutation_scale
        self.trackingArrowsSettings[i]['lw'] = lw
        self.trackingArrowsSettings[i]['ls'] = ls
        self.trackingArrowsSettings[i]['arrowstyle'] = style 
        self.trackingArrowsSettings[i]['color'] = color

    def setTraceLimit(self, i, lim):
        self.traceLimits[i] = lim

    def setLeadingBlob(self, i, size, shape, color, alpha, label):
        """
        [size]: marker size (int)
        [shape]: marker style (https://matplotlib.org/3.1.0/api/markers_api.html)
        [color]: marker color
        [alpha]: marker alpha
        [label]: marker label
        """
        self.leadingBlob[i]['size'] = size
        self.leadingBlob[i]['shape'] = shape
        self.leadingBlob[i]['color'] = color
        self.leadingBlob[i]['alpha'] = alpha
        self.leadingBlob[i]['label'] = label

    def renderUKF(self, delay=0.001, text=''):
        self.ax.cla()
        self.fig.suptitle(text)
        # Trajectories
        for i in range(len(self.trajectories)):
            traj = self.trajectories[i]
            settings = self.settings[i]
            traceLim = self.traceLimits[i]  # Grab last N elements, which removes older portions of the trajectory
            blob = self.leadingBlob[i]
            if traceLim is None:
                traceLim = len(traj['x'])   # Do not remove any portion of the path
            if traceLim > 0 and settings is not None:
                self.ax.plot(traj['x'][-traceLim:], traj['y'][-traceLim:], traj['z'][-traceLim:], color=settings['color'], label=settings['label'],alpha=settings['alpha'], ls=settings['ls'])
            if blob['size'] > 0:
                self.ax.plot([traj['x'][-1]], [traj['y'][-1]], [traj['z'][-1]], color=blob['color'], label=blob['label'], alpha=blob['alpha'], markersize=blob['size'], marker=blob['shape'])
        # Arrows
        for i in range(len(self.trackingArrows)):
            a = Arrow3D(self.trackingArrows[i]['x'], self.trackingArrows[i]['y'], self.trackingArrows[i]['z'], 
                        mutation_scale=self.trackingArrowsSettings[i]['mutation_scale'], 
                        lw=self.trackingArrowsSettings[i]['lw'], 
                        ls=self.trackingArrowsSettings[i]['ls'],
                        arrowstyle=self.trackingArrowsSettings[i]['arrowstyle'], 
                        color=self.trackingArrowsSettings[i]['color'])
            self.ax.add_artist(a)

        self.ax.set_xlabel('$X$', fontsize=20, rotation=150)
        self.ax.set_ylabel('$Y$', fontsize=20)
        self.ax.set_zlabel('$Z$', fontsize=20, rotation=60)

        if self.bounds is not None:
            self.ax.auto_scale_xyz([self.bounds[0], self.bounds[1]], [self.bounds[2], self.bounds[3]], [self.bounds[4], self.bounds[5]])
            
        self.ax.legend()
        plt.draw()
        plt.pause(delay)

    # def preRender(self, text=''):
    #     self.ax.cla()
    #     self.fig.suptitle(text)

    # def render(self, xs, ys, zs, label='', color='blue', marker='o'):
    #     self.ax.plot(xs, ys, zs, color=color, label=label,alpha=0.5, marker=marker)

    # def postRender(self, delay=0.001):
    #     self.ax.set_xlabel('$X$', fontsize=20, rotation=150)
    #     self.ax.set_ylabel('$Y$', fontsize=20)
    #     self.ax.set_zlabel('$Z$', fontsize=20, rotation=60)
    #     self.ax.legend()
    #     plt.draw()
    #     plt.pause(delay)
    
    def close(self):
        plt.close(self.fig)
