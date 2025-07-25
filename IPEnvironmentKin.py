
"""
This code is part of a series of notebooks regarding  "Introduction to robot path planning".

Author: Gergely Soti, adapted by Bjoern Hein

License is based on Creative Commons: Attribution-NonCommercial 4.0 International (CC BY-NC 4.0) (pls. check: http://creativecommons.org/licenses/by-nc/4.0/)
"""
from IPPlanarManipulator import PlanarJoint, PlanarRobot
from IPEnvironment import CollisionChecker
from shapely.geometry import Point, Polygon, LineString
from shapely import plotting
import numpy as np
import copy

def interpolate_line(startPos, endPos, step_l):
    steps = []
    line = np.array(endPos) - np.array(startPos)
    line_l = np.linalg.norm(line)
    step = line / line_l * step_l
    n_steps = np.floor(line_l / step_l).astype(np.int32)
    c_step = np.array(startPos)
    for i in range(n_steps):
        steps.append(copy.deepcopy(c_step))
        c_step += step
    if not (c_step == np.array(endPos)).all():
        steps.append(np.array(endPos))
    return steps


class KinChainCollisionChecker(CollisionChecker):
    def __init__(self, kin_chain, scene, limits=[[-3.0, 3.0], [-3.0, 3.0]], statistic=None, fk_resolution=0.1):
        super(KinChainCollisionChecker, self).__init__(scene, limits)
        self.statistic = statistic
        if len(limits) != 2 or len(limits[0]) != kin_chain.dim or len(limits[1]) != kin_chain.dim:
            raise ValueError("Limits must be a list [lower_limits, upper_limits] matching robot dimensions.")
        self.kin_chain = kin_chain
        self.fk_resolution = fk_resolution
        self.dim = self.kin_chain.dim

    def point_in_collision(self, pos):  # Geändert von pointInCollision
        self.kin_chain.move(pos)
        joint_positions = self.kin_chain.get_transforms()
        for i in range(1, len(joint_positions)):
            if self.segment_in_collision(joint_positions[i - 1], joint_positions[i]):
                return True
        return False

    def line_in_collision(self, startPos, endPos):  # Geändert von lineInCollision
        assert len(startPos) == self.get_dim()
        assert len(endPos) == self.get_dim()
        steps = self._interpolate_line(startPos, endPos, self.fk_resolution)
        for pos in steps:
            if self.point_in_collision(pos):
                return True
        return False

    def segment_in_collision(self, startPos, endPos):  # Geändert von segmentInCollision
        for key, value in self.scene.items():
            if value.intersects(LineString([(startPos[0], startPos[1]), (endPos[0], endPos[1])])):
                return True
        return False

    def get_dim(self):  # Geändert von getDim
        return self.dim

    def _interpolate_line(self, start, end, resolution):
        diff = np.array(end) - np.array(start)
        distance = np.linalg.norm(diff)
        num_steps = max(2, int(distance / resolution))
        return [start + diff * t for t in np.linspace(0, 1, num_steps)]



def planarRobotVisualize(kin_chain, ax):
    joint_positions = kin_chain.get_transforms()
    for i in range(1, len(joint_positions)):
        xs = [joint_positions[i-1][0], joint_positions[i][0]]
        ys = [joint_positions[i-1][1], joint_positions[i][1]]
        ax.plot(xs, ys, color='g')
        
  
import matplotlib.pyplot as plt              
import matplotlib.animation
from IPython.display import HTML

matplotlib.rcParams['animation.embed_limit'] = 64
def animateSolution(planner, environment, solution, visualizer):
    _planner = planner
    _environment = environment
    _solution = solution
    _prmVisualizer = visualizer
    
    if _environment.getDim() == 2:
    
        fig_local = plt.figure(figsize=(14, 7))
        ax1 = fig_local.add_subplot(1, 2, 1)
        ax2 = fig_local.add_subplot(1, 2, 2)
        ## get positions for solution
        solution_pos = [_planner.graph.nodes[node]['pos'] for node in _solution]
        ## interpolate to obtain a smoother movement
        i_solution_pos = [solution_pos[0]]
        for i in range(1, len(solution_pos)):
            segment_s = solution_pos[i-1]
            segment_e = solution_pos[i]
            i_solution_pos = i_solution_pos + interpolate_line(segment_s, segment_e, 0.1)[1:]
        ## animate
        frames = len(i_solution_pos)
        
        r = environment.kin_chain
        
        def animate(t):
            ## clear taks space figure
            ax1.cla()
            ## fix figure size
            ## draw obstacles
            _environment.drawObstacles(ax1)
            ## update robot position
            pos = i_solution_pos[t]
            r.move(pos)
            planarRobotVisualize(r, ax1)
        
            ## clear joint space figure
            ax2.cla()
            ## draw graph and path
            _prmVisualizer(_planner, solution, ax2)
            ## draw current position in joint space
            ax2.scatter(i_solution_pos[t][0], i_solution_pos[t][1], color='r', zorder=10, s=250)

        ani = matplotlib.animation.FuncAnimation(fig_local, animate, frames=frames)
        html = HTML(ani.to_jshtml())
        display(html)
        plt.close()
    else:
        fig_local = plt.figure(figsize=(7, 7))
        ax1 = fig_local.add_subplot(1, 1, 1)
        ## get positions for solution
        solution_pos = [_planner.graph.nodes[node]['pos'] for node in _solution]
        ## interpolate to obtain a smoother movement
        i_solution_pos = [solution_pos[0]]
        for i in range(1, len(solution_pos)):
            segment_s = solution_pos[i-1]
            segment_e = solution_pos[i]
            i_solution_pos = i_solution_pos + interpolate_line(segment_s, segment_e, 0.1)[1:]
        ## animate
        frames = len(i_solution_pos)
        
        r = environment.kin_chain
        
        def animate(t):
            ## clear taks space figure
            ax1.cla()
            ## fix figure size
            ## draw obstacles
            _environment.drawObstacles(ax1)
            ## update robot position
            pos = i_solution_pos[t]
            r.move(pos)
            planarRobotVisualize(r, ax1)
        
        
        ani = matplotlib.animation.FuncAnimation(fig_local, animate, frames=frames)
        html = HTML(ani.to_jshtml())
        display(html)
        plt.close()