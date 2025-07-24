# utils/planarRobot.py

import numpy as np
from shapely.geometry import LineString, Point


class PlanarRobot:
    def __init__(self, n_joints=2, link_lengths=None):
        self.n_joints = n_joints
        if link_lengths is None:
            self.link_lengths = [1.0] * n_joints
        else:
            assert len(link_lengths) == n_joints
            self.link_lengths = link_lengths

    def forward_kinematics(self, config):
        """
        config: list of joint angles (in radians)
        returns: list of (x, y) tuples representing joint positions
        """
        positions = [(0, 0)]
        x, y = 0, 0
        theta = 0
        for i, angle in enumerate(config):
            theta += angle
            dx = self.link_lengths[i] * np.cos(theta)
            dy = self.link_lengths[i] * np.sin(theta)
            x += dx
            y += dy
            positions.append((x, y))
        return positions


class KinChainCollisionChecker:
    def __init__(self, kin_chain: PlanarRobot, scene, limits=None, fk_resolution=0.2):
        self.kin_chain = kin_chain
        self.scene = scene
        self.fk_resolution = fk_resolution
        if limits is None:
            self.limits = [[-np.pi, np.pi]] * kin_chain.n_joints
        else:
            self.limits = limits

    def point_in_collision(self, config):
        points = self.kin_chain.forward_kinematics(config)
        for i in range(len(points) - 1):
            link = LineString([points[i], points[i + 1]])
            for obstacle in self.scene.values():
                if link.intersects(obstacle):
                    return True
        return False

    def line_in_collision(self, from_config, to_config):
        from_config = np.array(from_config)
        to_config = np.array(to_config)
        diff = to_config - from_config
        dist = np.linalg.norm(diff)
        steps = max(int(dist / self.fk_resolution), 1)
        for i in range(steps + 1):
            alpha = i / steps
            interp_config = from_config + alpha * diff
            if self.point_in_collision(interp_config):
                return True
        return False

    def draw_obstacles(self, ax):
        for obs in self.scene.values():
            if hasattr(obs, 'exterior'):
                x, y = obs.exterior.xy
                ax.fill(x, y, color='lightcoral', alpha=0.5)
