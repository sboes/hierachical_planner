# utils/collision_checker.py

import numpy as np
from shapely.geometry import Point, LineString


class CollisionChecker:
    def __init__(self, scene, limits=((0, 22), (0, 22))):
        self.scene = scene
        self.limits = limits

    def pointInCollision(self, pos):
        point = Point(pos[0], pos[1])
        return any(obstacle.intersects(point) for obstacle in self.scene.values())

    def lineInCollision(self, p1, p2):
        line = LineString([p1, p2])
        return any(obstacle.intersects(line) for obstacle in self.scene.values())

    def point_in_collision(self, pos):
        """Check if a point is in collision with any obstacle."""
        return self.pointInCollision(pos)

    def line_in_collision(self, startPos, endPos):
        """Check if a line segment from startPos to endPos is in collision with any obstacle."""
        return self.lineInCollision(startPos, endPos)
