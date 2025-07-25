# hierachial_planner_2.py
# we use the Visbility PRM as main algorithm and the lazy or basic PRM as subplanner
# coding: utf-8

"""
This code is part of the course "Introduction to robot path planning" (Author: Bjoern Hein).

License is based on Creative Commons: Attribution-NonCommercial 4.0 International (CC BY-NC 4.0) (pls. check: http://creativecommons.org/licenses/by-nc/4.0/)
"""

from lectures.IPPRMBase import PRMBase
from lectures.IPLazyPRM import LazyPRM
from lectures.IPPerfMonitor import IPPerfMonitor
import networkx as nx
from scipy.spatial import cKDTree
from shapely.geometry import box
import random

class VisibilityStatsHandler:
    def __init__(self):
        self.graph = nx.Graph()

    def addNodeAtPos(self, nodeNumber, pos):
        self.graph.add_node(nodeNumber, pos=pos, color='yellow')

    def addVisTest(self, fr, to):
        self.graph.add_edge(fr, to)


class HierachialPRM(PRMBase):
    def __init__(self, _collChecker, _subplanner_class, _subplanner_config, _statsHandler=None):
        super(HierachialPRM, self).__init__(_collChecker)
        self.graph = nx.Graph()
        self.statsHandler = VisibilityStatsHandler()
        self.subplanner_class = _subplanner_class
        self.subplanner_config = _subplanner_config

    def _isVisible(self, pos, guardPos):
        return not self._collisionChecker.lineInCollision(pos, guardPos)

    def _get_bounding_area(self, p1, p2, padding=2.0):
        x_min = min(p1[0], p2[0]) - padding
        x_max = max(p1[0], p2[0]) + padding
        y_min = min(p1[1], p2[1]) - padding
        y_max = max(p1[1], p2[1]) + padding
        return ((x_min, x_max), (y_min, y_max))

    def _subplan_connection(self, pos1, pos2):
        bounds_x, bounds_y = self._get_bounding_area(pos1, pos2, padding=2.0)
        subplanner = self.subplanner_class(self._collisionChecker)
        if hasattr(subplanner, 'setSamplingBounds'):
            subplanner.setSamplingBounds((bounds_x, bounds_y))
        path = subplanner.planPath([pos1], [pos2], self.subplanner_config)
        return len(path) > 0

    @IPPerfMonitor
    def _learnRoadmap(self, ntry):
        nodeNumber = 0
        currTry = 0
        while currTry < ntry:
            q_pos = self._getRandomFreePosition()
            if self.statsHandler:
                self.statsHandler.addNodeAtPos(nodeNumber, q_pos)

            g_vis = None
            merged = False

            for comp in nx.connected_components(self.graph):
                for g in comp:
                    if self.graph.nodes()[g]['nodeType'] != 'Guard':
                        continue

                    guard_pos = self.graph.nodes()[g]['pos']
                    if self.statsHandler:
                        self.statsHandler.addVisTest(nodeNumber, g)

                    if self._isVisible(q_pos, guard_pos):
                        if g_vis is None:
                            g_vis = g
                        else:
                            self.graph.add_node(nodeNumber, pos=q_pos, color='lightblue', nodeType='Connection')
                            self.graph.add_edge(nodeNumber, g)
                            self.graph.add_edge(nodeNumber, g_vis)
                            merged = True
                        continue

                    # keine Sichtlinie: versuche Subplanung
                    if self._subplan_connection(q_pos, guard_pos):
                        if g_vis is None:
                            g_vis = g
                        else:
                            self.graph.add_node(nodeNumber, pos=q_pos, color='lightblue', nodeType='Connection')
                            self.graph.add_edge(nodeNumber, g)
                            self.graph.add_edge(nodeNumber, g_vis)
                            merged = True
                        continue

                if merged:
                    break

            if not merged and g_vis is None:
                self.graph.add_node(nodeNumber, pos=q_pos, color='red', nodeType='Guard')
                currTry = 0
            else:
                currTry += 1

            nodeNumber += 1

    @IPPerfMonitor
    def planPath(self, startList, goalList, config):
        self.graph.clear()
        checkedStartList, checkedGoalList = self._checkStartGoal(startList, goalList)

        self._learnRoadmap(config["ntry"])

        posList = nx.get_node_attributes(self.graph, 'pos')
        kdTree = cKDTree(list(posList.values()))

        result = kdTree.query(checkedStartList[0], k=5)
        for node in result[1]:
            if not self._collisionChecker.lineInCollision(checkedStartList[0], self.graph.nodes()[list(posList.keys())[node]]['pos']):
                self.graph.add_node("start", pos=checkedStartList[0], color='lightgreen')
                self.graph.add_edge("start", list(posList.keys())[node])
                break

        result = kdTree.query(checkedGoalList[0], k=5)
        for node in result[1]:
            if not self._collisionChecker.lineInCollision(checkedGoalList[0], self.graph.nodes()[list(posList.keys())[node]]['pos']):
                self.graph.add_node("goal", pos=checkedGoalList[0], color='lightgreen')
                self.graph.add_edge("goal", list(posList.keys())[node])
                break

        try:
            path = nx.shortest_path(self.graph, "start", "goal")
        except:
            return []
        return path
