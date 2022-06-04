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


class Live2DPlot:
    """
    For 2d graphs
    """
    def __init__(self, bounds=None):
        self.settings = []
        self.graphs = []
        self.traceLimits = []
        plt.ion()
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(1,1,1)
        self.bounds = bounds;

    def addGraph(self, color, label, alpha, ls, traceLim=None):
        id = len(self.graphs)
        self.graphs.append({'x':[], 'y':[]})
        self.settings.append({'color':color, 'label':label, 'alpha':alpha, 'ls':ls})
        self.traceLimits.append(traceLim)
        assert len(self.graphs) == len(self.settings) == len(self.traceLimits)
        return id

    def updateGraph(self, id, x, y):
        assert id >= 0 and id < len(self.graphs)
        traceLim = self.traceLimits[id]  # Grab last N elements, which removes older portions of the trajectory
        if traceLim is None:
            traceLim = len(self.graphs[id]['x'])   # Do not remove any portion of the path
        self.graphs[id]['x'].append(x)
        self.graphs[id]['y'].append(y)

        self.graphs[id]['x'] = self.graphs[id]['x'][-traceLim:]
        self.graphs[id]['y'] = self.graphs[id]['y'][-traceLim:]

    def render(self, delay=0.001, text=''):
        self.ax.cla()
        self.fig.suptitle(text)
        # Trajectories
        for i in range(len(self.graphs)):
            traj = self.graphs[i]
            settings = self.settings[i]
            # blob = self.leadingBlob[i]
            
            self.ax.plot(traj['x'], traj['y'], color=settings['color'], label=settings['label'],alpha=settings['alpha'], ls=settings['ls'])
            # if blob['size'] > 0:
            #     self.ax.plot([traj['x'][-1]], [traj['y'][-1]], [traj['z'][-1]], color=blob['color'], label=blob['label'], alpha=blob['alpha'], markersize=blob['size'], marker=blob['shape'])

        # self.ax.xlabel('$X$', fontsize=20, rotation=150)
        # self.ax.ylabel('$Y$', fontsize=20)
        # self.ax.set_zlabel('$Z$', fontsize=20, rotation=60)

        # if self.bounds is not None:
        #     self.ax.auto_scale_xyz([self.bounds[0], self.bounds[1]], [self.bounds[2], self.bounds[3]], [self.bounds[4], self.bounds[5]])
            
        self.ax.legend()
        plt.draw()
        plt.pause(delay)

    
    def close(self):
        plt.close(self.fig)

class LiveMultipleAttitudePlot:
    """
    Plot Attitude data
    For arrow styles, visit: https://matplotlib.org/3.1.1/api/_as_gen/matplotlib.patches.ArrowStyle.html
    For line styles (ls), visit: https://matplotlib.org/3.1.1/api/_as_gen/matplotlib.patches.Arrow.html#matplotlib.patches.Arrow
    """
    def __init__(self, bounds=None):
        self.settings = []
        self.arrows = []
        plt.ion()
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.bounds = bounds;

    def addArrowFromOrigin(self, alpha=1.0, mutation_scale=20, lw=2, arrow_style="-|>", color="r", ls='-', label='',size=10):
        """
        Plots an arrow that starts at (0,0,0) and has the slope [dx], [dy] and [dz]
        Returns: id of arrow
        """
        id = len(self.arrows)
        self.arrows.append({'x':[0, 0], 'y':[0, 0], 'z':[0, 0]})
        self.settings.append({'alpha':alpha,'mutation_scale':mutation_scale, 'lw':lw, 'arrowstyle':arrow_style, 'color':color, 'ls':ls, 'label':label, 'size':size})
        assert len(self.arrows) == len(self.settings)
        return id

    def updateOriginArrow(self, id, dx, dy, dz):
        self.arrows[id]['x'][1] = dx
        self.arrows[id]['y'][1] = dy
        self.arrows[id]['z'][1] = dz

    def render(self, delay=0.001, text=''):
        self.ax.cla()
        self.fig.suptitle(text)
        for i in range(len(self.arrows)):
            endpoint = self.arrows[i]
            a = Arrow3D(self.arrows[i]['x'], self.arrows[i]['y'], self.arrows[i]['z'], 
                        mutation_scale=self.settings[i]['mutation_scale'], 
                        lw=self.settings[i]['lw'], 
                        ls=self.settings[i]['ls'],
                        arrowstyle=self.settings[i]['arrowstyle'], 
                        color=self.settings[i]['color'],alpha=self.settings[i]['alpha'])
            self.ax.add_artist(a)
            name = self.settings[i]['label']
            self.ax.plot([endpoint['x'][1]], [endpoint['y'][1]], [endpoint['z'][1]], color=self.settings[i]['color'], alpha=0.5, markersize=self.settings[i]['size'], marker=f'${name}$')

        self.ax.set_xlabel('$X$', fontsize=20, rotation=150)
        self.ax.set_ylabel('$Y$', fontsize=20)
        self.ax.set_zlabel('$Z$', fontsize=20, rotation=60)

        if self.bounds is not None:
            self.ax.auto_scale_xyz([self.bounds[0], self.bounds[1]], [self.bounds[2], self.bounds[3]], [self.bounds[4], self.bounds[5]])
            
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
        self.trackingArrows = []
        # for i in range(trajectories):
        #     self.trajectories.append({'x':[],'y':[],'z':[]})
        #     self.settings.append({'color':'','label':'','alpha':'','ls':''})
        #     self.traceLimits.append(None)
        #     self.leadingBlob.append({'size':0, 'shape':'0', 'color':'red', 'alpha':0, 'label':'N/A'})
        # for i in range(trackingArrows):
        #     self.trackingArrows.append({'x':[0,0], 'y':[0,0], 'z':[0,0]})
        #     self.trackingArrowsSettings.append({'mutation_scale':20, 'lw':2, 'arrowstyle':"-|>", 'color':"r", 'ls':'-'})
        plt.ion()
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.bounds = bounds;

    def addTrajectory(self, color, label, alpha, ls, traceLim, blobsize, blobshape, bloblabel):
        """
        https://matplotlib.org/3.2.1/api/_as_gen/matplotlib.axes.Axes.plot.html
        '-' 	solid line style
        '--' 	dashed line style
        '-.' 	dash-dot line style
        ':' 	dotted line style
        """
        """
        [blobsize]: marker size (int)
        [blobshape]: marker style (https://matplotlib.org/3.1.0/api/markers_api.html)
        [bloblabel]: marker label
        """
        i = len(self.trajectories)
        self.trajectories.append({'x':[], 'y':[], 'z': [], 'color':color,'label':label,'alpha':alpha,'ls':ls,'trace_lim':traceLim,
                                    'leading_blob':{'size':blobsize, 'shape':blobshape, 'label':bloblabel}})
        return i

    def updateTraj(self, i, x, y, z):
        self.trajectories[i]['x'].append(x)
        self.trajectories[i]['y'].append(y)
        self.trajectories[i]['z'].append(z)

        self.trajectories[i]['x'] = self.trajectories[i]['x'][-self.trajectories[i]['trace_lim']:]
        self.trajectories[i]['y'] = self.trajectories[i]['y'][-self.trajectories[i]['trace_lim']:]
        self.trajectories[i]['z'] = self.trajectories[i]['z'][-self.trajectories[i]['trace_lim']:]

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

    def addTrackingArrow(self, mutation_scale=20, lw=2, ls='-', style="-|>", color="r", name='', size=10):
        i = len(self.trackingArrows)
        self.trackingArrows.append({'x':[None,None], 'y':[None,None], 'z':[None,None], 'mutation_scale':mutation_scale, 'lw':lw, 'arrowstyle':style, 'color':color, 'ls':ls, 'name':name, 'markersize':size})
        return i

    def renderUKF(self, delay=0.001, text=''):
        self.ax.cla()
        self.fig.suptitle(text)
        # Trajectories
        for i in range(len(self.trajectories)):
            traj = self.trajectories[i]
            if len(traj['x']) <= 0:
                continue
            blob = traj['leading_blob']
            # trackingArrow = self.trackingArrows[i]
            if traj['trace_lim'] > 0:
                self.ax.plot(traj['x'], traj['y'], traj['z'], color=traj['color'], label=traj['label'],alpha=traj['alpha'], ls=traj['ls'])
            if blob['size'] > 0:
                self.ax.plot([traj['x'][-1]], [traj['y'][-1]], [traj['z'][-1]], color=traj['color'], label=blob['label'], alpha=traj['alpha'], markersize=blob['size'], marker=blob['shape'])
        # Arrows
        for i in range(len(self.trackingArrows)):
            arrow = self.trackingArrows[i]
            name = arrow['name']
            if arrow['x'][0] is None:
                continue
            a = Arrow3D(arrow['x'], arrow['y'], arrow['z'], 
                        mutation_scale=arrow['mutation_scale'], 
                        lw=arrow['lw'], 
                        ls=arrow['ls'],
                        arrowstyle=arrow['arrowstyle'], 
                        color=arrow['color'])
            self.ax.add_artist(a)
            self.ax.plot([arrow['x'][1]], [arrow['y'][1]], [arrow['z'][1]], color=arrow['color'], alpha=0.5, markersize=arrow['markersize'], marker=f'${name}$')

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
