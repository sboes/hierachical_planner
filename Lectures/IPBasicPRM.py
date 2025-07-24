# coding: utf-8

"""
This code is part of the course "Introduction to robot path planning" (Author: Bjoern Hein).

License is based on Creative Commons: Attribution-NonCommercial 4.0 International (CC BY-NC 4.0) (pls. check: http://creativecommons.org/licenses/by-nc/4.0/)
"""

import IPPRMBase
from IPPerfMonitor import IPPerfMonitor
import networkx as nx
import random
import numpy as np
import math

# reduce coding effort by using function provided by scipy
from scipy.spatial.distance import euclidean, cityblock

class BasicPRM(IPPRMBase.PRMBase):

    def __init__(self, _collChecker):
        super(BasicPRM, self).__init__(_collChecker)
        self.graph = nx.Graph()

    
    @IPPerfMonitor
    def _inSameConnectedComponent(self, node1, node2):
        """ Check whether to nodes are part of the same connected component using
            functionality from NetworkX
        """
        for connectedComponent in nx.connected_components(self.graph):
            if (node1 in connectedComponent) & (node2 in connectedComponent):
                return True

        return False

    
    @IPPerfMonitor
    def _nearestNeighbours(self, pos, radius):
        """ Brute Force method to find all nodes of a 
        graph near the given position **pos** with in the distance of
        **radius** """

        result = list()
        for node in self.graph.nodes(data=True):
            if euclidean(node[1]['pos'],pos) <= radius:
                result.append(node)

        return result
    
    @IPPerfMonitor
    def _learnRoadmapNearestNeighbour(self, radius, numNodes):
        """ Generate a roadmap by given number of nodes and radius, that should be tested for connection."""
        # nodeID is used for uniquely enumerating all nodes and as their name
        nodeID = 1
        while nodeID <= numNodes:
        
            # Generate a 'randomly chosen, free configuration'
            newNodePos = self._getRandomFreePosition()
            self.graph.add_node(nodeID, pos=newNodePos)
            
            # Find set of candidates to connect to sorted by distance
            result = self._nearestNeighbours(newNodePos, radius)

            # for all nearest neighbours check whether a connection is possible
            for data in result:
                if self._inSameConnectedComponent(nodeID,data[0]):
                    break
                
                if not self._collisionChecker.lineInCollision(newNodePos,data[1]['pos']):
                    self.graph.add_edge(nodeID,data[0])
            
            nodeID += 1
            
    @IPPerfMonitor
    def planPath(self, startList, goalList, config):
        """
        
        Args:
            start (array): start position in planning space
            goal (array) : goal position in planning space
            config (dict): dictionary with the needed information about the configuration options
            
        Example:
        
            config["radius"]   = 5.0
            config["numNodes"] = 300
            config["useKDTree"] = True
            
            startList = [[1,1]]
            goalList  = [[10,1]]
            
            instance.planPath(startList,goalList,config)
        
        """
        # 0. reset
        self.graph.clear()
        
        # 1. check start and goal whether collision free (s. BaseClass)
        checkedStartList, checkedGoalList = self._checkStartGoal(startList,goalList)
        
        # 2. learn Roadmap
        self._learnRoadmapNearestNeighbour(config["radius"],config["numNodes"])

        # 3. find connection of start and goal to roadmap
        # find nearest, collision-free connection between node on graph and start
        result = self._nearestNeighbours(checkedStartList[0], config["radius"])
        for node in result:
            if not self._collisionChecker.lineInCollision(checkedStartList[0],node[1]['pos']):
                 self.graph.add_node("start", pos=checkedStartList[0], color='lightgreen')
                 self.graph.add_edge("start", node[0])
                 break

        result = self._nearestNeighbours(checkedGoalList[0], config["radius"])
        for node in result:
            if not self._collisionChecker.lineInCollision(checkedGoalList[0],node[1]['pos']):
                 self.graph.add_node("goal", pos=checkedGoalList[0], color='lightgreen')
                 self.graph.add_edge("goal", node[0])
                 break

        try:
            path = nx.shortest_path(self.graph,"start","goal")
        except:
            return []
        return path

            # Erweiterung für HierarchicalPRM-Kompatibilität
        def reset(self):
            self.graph.clear()
    
        def setSamplingRegion(self, pos1, pos2, margin=0.1):
            self._region_min = np.minimum(pos1, pos2) - margin
            self._region_max = np.maximum(pos1, pos2) + margin
    
        def buildRoadmap(self, numNodes):
            self.reset()
            radius = 2.0  # oder konfigurierbar machen
            nodeID = 1
            count = 0
            while count < numNodes:
                q = self._getRandomFreePosition()
                if np.all(q >= self._region_min) and np.all(q <= self._region_max):
                    self.graph.add_node(nodeID, pos=q)
                    neighbors = self._nearestNeighbours(q, radius)
                    for neighbor in neighbors:
                        if not self._collisionChecker.lineInCollision(q, neighbor[1]['pos']):
                            self.graph.add_edge(nodeID, neighbor[0])
                    nodeID += 1
                    count += 1
    
        def query(self, start_pos, goal_pos):
            self.graph.add_node("start", pos=start_pos)
            self.graph.add_node("goal", pos=goal_pos)
            radius = 2.0
    
            for node, data in self.graph.nodes(data=True):
                if node in ["start", "goal"]:
                    continue
                if not self._collisionChecker.lineInCollision(start_pos, data["pos"]):
                    self.graph.add_edge("start", node)
                if not self._collisionChecker.lineInCollision(goal_pos, data["pos"]):
                    self.graph.add_edge("goal", node)
    
            try:
                path = nx.shortest_path(self.graph, "start", "goal")
                return True, [self.graph.nodes[n]["pos"] for n in path]
            except:
                return False, []
    
