from shapely.geometry import Point, LineString
from typing import List, Tuple

class VisibilityConnector:
    def __init__(self, checker, subplanner_class, config):
        self.checker = checker
        self.subplanner_class = subplanner_class
        self.config = config

    def get_connections(self, new_node: Tuple[float, float], existing_nodes: List[Tuple[int, Tuple[float, float]]]):
        visible = []
        subplanned = []
        areas = []
        paths = []

        for _, other_pos in existing_nodes:
            if other_pos == new_node:
                continue

            if not self.checker.lineInCollision(new_node, other_pos):
                visible.append((other_pos, 'visibility'))
            else:
                bounds_x = (min(new_node[0], other_pos[0]) - 2.0, max(new_node[0], other_pos[0]) + 2.0)
                bounds_y = (min(new_node[1], other_pos[1]) - 2.0, max(new_node[1], other_pos[1]) + 2.0)
                sub = self.subplanner_class(self.checker)
                if hasattr(sub, 'setSamplingBounds'):
                    sub.setSamplingBounds((bounds_x, bounds_y))
                path = sub.planPath([new_node], [other_pos], self.config)
                if path and len(path) > 1:
                    subplanned.append((other_pos, 'subplan'))
                    areas.append((bounds_x, bounds_y))
                    paths.append(path)

        return visible, subplanned, areas, paths
