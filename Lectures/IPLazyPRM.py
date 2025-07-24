# coding: utf-8
"""
This code is part of the course "Introduction to robot path planning" (Author: Bjoern Hein). It is based on the slides given during the course, so please **read the information in theses slides first**

License is based on Creative Commons: Attribution-NonCommercial 4.0 International (CC BY-NC 4.0) (pls. check: http://creativecommons.org/licenses/by-nc/4.0/)
"""

from IPPRMBase import PRMBase
from scipy.spatial import cKDTree
import networkx as nx
import random

from IPPerfMonitor import IPPerfMonitor

class LazyPRM(PRMBase):

    def __init__(self, _collChecker):
        super(LazyPRM, self).__init__(_collChecker)
        
        self.graph = nx.Graph()
        self.lastGeneratedNodeNumber = 0
        self.collidingEdges = []
        self.nonCollidingEdges =[]
        
    @IPPerfMonitor
    def _buildRoadmap(self, numNodes, kNearest):
        
        # generate #numNodes nodes
        addedNodes = []
        for i in range(numNodes):
            pos = self._getRandomPosition()
            self.graph.add_node(self.lastGeneratedNodeNumber, pos=pos)
            addedNodes.append(self.lastGeneratedNodeNumber)
            self.lastGeneratedNodeNumber += 1


        # for every node in graph find nearest neigbhours
        posList = list(nx.get_node_attributes(self.graph,'pos').values())
        #print posList
        kdTree = cKDTree(posList)
        
        # to see when _buildRoadmap has to be called again
        #print addedNodes
        for node in addedNodes:
        #for node in self.graph.nodes():
        # Find set of candidates to connect to sorted by distance
            result = kdTree.query(self.graph.nodes[node]['pos'],k=kNearest)
            for data in result[1]:
                c_node = [x for x, y in self.graph.nodes(data=True) if (y['pos']==posList[data])][0]
                if node!=c_node:
                    if (node, c_node) not in self.collidingEdges:
                        self.graph.add_edge(node,c_node)
                    else:
                        continue
                        #print "not adding already checked colliding edge"
    
    @IPPerfMonitor
    def _checkForCollisionAndUpdate(self,path):
        # first check all nodes
        for nodeNumber in path:
            if self._collisionChecker.pointInCollision(self.graph.nodes[nodeNumber]['pos']):
                self.graph.remove_node(nodeNumber)
                #print "Colliding Node"
                return True
        
        # check all path segments
        for elem in zip(path,path[1:]):
            #print elem
            x = elem[0]
            y = elem[1]
            if self._collisionChecker.lineInCollision(self.graph.nodes()[x]['pos'],self.graph.nodes()[y]['pos']):
                self.graph.remove_edge(x,y)
                self.collidingEdges.append((x,y))
                return True
            else:
                self.nonCollidingEdges.append((x,y))
                                                                                          
            
        return False
        
    @IPPerfMonitor   
    def planPath(self, startList, goalList, config):
        """
        
        Args:
            startList (array): start position in planning space
            goalList (array) : goal position in planning space
            config (dict): dictionary with the needed information about the configuration options
            
        Example:
        
            config["initialRoadmapSize"] = 40 # number of nodes of first roadmap
            config["updateRoadmapSize"]  = 20 # number of nodes to add if there is no connection from start to end
            config["kNearest"] = 5 # number of nodes to connect to during setup
            config["maxIterations"] = 40 # number of iterations trying to refine the roadmap
            
        """
        # 0. reset
        self.graph.clear()
        self.lastGeneratedNodeNumber = 0
        self.collidingEdges = []
        
        # 1. check start and goal whether collision free (s. BaseClass)
        checkedStartList, checkedGoalList = self._checkStartGoal(startList,goalList)
        
        # 2. add start and goal to graph
        self.graph.add_node("start", pos=checkedStartList[0])
        self.graph.add_node("goal", pos=checkedGoalList[0])
        
        # 3. build initial roadmap
        self._buildRoadmap(config["initialRoadmapSize"], config["kNearest"])
        
        maxTry = 0
        while maxTry < config["maxIterations"]: 
            try:
                path = nx.shortest_path(self.graph,"start","goal")
            except:
                self._buildRoadmap(config["updateRoadmapSize"], config["kNearest"])
                maxTry += 1
                continue
              
            if self._checkForCollisionAndUpdate(path):
                continue
            else:
                #print "Found solution"
                return path
            
        return []

        def reset(self):
            self.graph.clear()
            self.collidingEdges = []
            self.nonCollidingEdges = []

        def setSamplingRegion(self, pos1, pos2, margin=0.1):
            self._region_min = np.minimum(pos1, pos2) - margin
            self._region_max = np.maximum(pos1, pos2) + margin
    
        def buildRoadmap(self, numNodes):
            from scipy.spatial.distance import euclidean
            import random
            self.reset()
            nodeID = 0
            while nodeID < numNodes:
                q = self._getRandomFreePosition()
                if np.all(q >= self._region_min) and np.all(q <= self._region_max):
                    self.graph.add_node(nodeID, pos=q)
                    nodeID += 1
    
            # K-Nächste Nachbarn-Verbindung (Lazy)
            self._connectKNearest(k=10)
    
        def _connectKNearest(self, k=10):
            import numpy as np
            from scipy.spatial import cKDTree
    
            posDict = nx.get_node_attributes(self.graph, 'pos')
            keys = list(posDict.keys())
            positions = np.array([posDict[k] for k in keys])
            if len(positions) == 0:
                return
    
            tree = cKDTree(positions)
            for idx, key in enumerate(keys):
                q = positions[idx]
                _, neighbor_idxs = tree.query(q, k=k + 1)
                for ni in neighbor_idxs:
                    if ni == idx:
                        continue
                    u, v = key, keys[ni]
                    if not self.graph.has_edge(u, v):
                        self.graph.add_edge(u, v)
    
        def query(self, start_pos, goal_pos):
            self.graph.add_node("start", pos=start_pos)
            self.graph.add_node("goal", pos=goal_pos)
    
            for node, data in self.graph.nodes(data=True):
                if node in ["start", "goal"]:
                    continue
                if not self._collisionChecker.lineInCollision(start_pos, data["pos"]):
                    self.graph.add_edge("start", node)
                if not self._collisionChecker.lineInCollision(goal_pos, data["pos"]):
                    self.graph.add_edge("goal", node)
    
            try:
                path = nx.shortest_path(self.graph, "start", "goal")
            except:
                return False, []
    
            # Lazy-Evaluation: Kollisionen prüfen
            valid_path = True
            for u, v in zip(path[:-1], path[1:]):
                p1 = self.graph.nodes[u]["pos"]
                p2 = self.graph.nodes[v]["pos"]
                if self._collisionChecker.lineInCollision(p1, p2):
                    valid_path = False
                    self.collidingEdges.append((u, v))
                else:
                    self.nonCollidingEdges.append((u, v))
    
            if valid_path:
                return True, [self.graph.nodes[n]["pos"] for n in path]
            else:
                return False, []
    

    