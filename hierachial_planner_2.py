import networkx as nx
from scipy.spatial import cKDTree
from shapely.geometry import box
from lectures.IPPRMBase import PRMBase
from lectures.IPPerfMonitor import IPPerfMonitor

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

    def _isVisible(self, pos, guardPos): # Check if a position is visible to a guard
        return not self._collisionChecker.lineInCollision(pos, guardPos)

    def _get_bounding_area(self, p1, p2, padding=2.0): # Calculate the bounding area for two points with padding
        x_min = min(p1[0], p2[0]) - padding
        x_max = max(p1[0], p2[0]) + padding
        y_min = min(p1[1], p2[1]) - padding
        y_max = max(p1[1], p2[1]) + padding
        return ((x_min, x_max), (y_min, y_max))

    def _subplan_connection(self, pos1, pos2): # Use a subplanner to find a path between two positions
        bounds_x, bounds_y = self._get_bounding_area(pos1, pos2, padding=2.0)
        subplanner = self.subplanner_class(self._collisionChecker)
        if hasattr(subplanner, 'setSamplingBounds'):
            subplanner.setSamplingBounds((bounds_x, bounds_y))
        path = subplanner.planPath([pos1], [pos2], self.subplanner_config)
        return path

    def learnSinglePoint(self, pos, node_id=None, verbose=True): # Learn a single point and its connections to guards
        if node_id is None:
            node_id = len(self.graph.nodes)

        if self.statsHandler:
            self.statsHandler.addNodeAtPos(node_id, pos)

        connections = []
        for comp in nx.connected_components(self.graph):
            for g in comp:
                if self.graph.nodes[g].get('nodeType') != 'Guard':
                    continue

                guard_pos = self.graph.nodes[g]['pos']
                if self.statsHandler:
                    self.statsHandler.addVisTest(node_id, g)

                if self._isVisible(pos, guard_pos):
                    connections.append((g, "visibility", [pos, guard_pos]))
                    if verbose:
                        print(f"[{node_id}] sees Guard {g} via VISIBILITY")
                else:
                    path = self._subplan_connection(pos, guard_pos)
                    if path and len(path) > 1:
                        connections.append((g, "subplan", path))
                        if verbose:
                            print(f"[{node_id}] connects to Guard {g} via SUBPLANNER")

        guard_count = sum(1 for _, data in self.graph.nodes(data=True) if data.get('nodeType') == 'Guard')

        if len(connections) == 0:
            # Es gibt keine Verbindung – also ist das ein neuer Guard
            self.graph.add_node(node_id, pos=pos, color='red', nodeType='Guard')
            if verbose:
                print(f"[{node_id}] Added GUARD (no connections)")
            return "guard", []

        # Prüfe, ob überhaupt Guards vorhanden sind
        if guard_count == 0:
            self.graph.add_node(node_id, pos=pos, color='red', nodeType='Guard')
            if verbose:
                print(f"[{node_id}] Added GUARD (first one)")
            return "guard", []

        # Prüfe ob mindestens eine Verbindung neue Information bringt
        existing_edges = set(frozenset((a, b)) for a, b in self.graph.edges)
        existing_nodes = set(self.graph.nodes)
        brings_new_info = False

        for g, mode, path in connections:
            if len(path) < 2:
                continue
            for p in path[1:-1]:  # Zwischenknoten prüfen
                if p not in existing_nodes:
                    brings_new_info = True
                    break
            for a, b in zip(path[:-1], path[1:]):
                edge = frozenset((a, b))
                if edge not in existing_edges:
                    brings_new_info = True
                    break
            if brings_new_info:
                break

        if not brings_new_info:
            if verbose:
                print(f"[{node_id}] Rejected – no new graph elements introduced")
            return "rejected", []

        if len(connections) == 0:
            self.graph.add_node(node_id, pos=pos, color='red', nodeType='Guard')
            if verbose:
                print(f"[{node_id}] Added GUARD (no connections)")
            return "guard", []

        elif len(connections) == 1:
            if guard_count <= 1:
                g, mode, path = connections[0]
                self.graph.add_node(node_id, pos=pos, color='lightblue', nodeType='Connection')
                last = node_id
                for i, p in enumerate(path[1:-1], start=1):
                    inter_id = f"{node_id}_p{i}"
                    self.graph.add_node(inter_id, pos=p, color='gray', nodeType='SubplanPoint')
                    self.graph.add_edge(last, inter_id)
                    last = inter_id
                self.graph.add_edge(last, g)
                if verbose:
                    print(f"[{node_id}] Connected to Guard {g} (only one, but allowed due to few guards)")
                return "connection", [g]
            else:
                if verbose:
                    print(f"[{node_id}] Rejected – only one connection, and multiple guards exist")
                return "rejected", []

        else:
            self.graph.add_node(node_id, pos=pos, color='lightblue', nodeType='Connection')
            for g, mode, path in connections[:2]:
                last = node_id
                for i, p in enumerate(path[1:-1], start=1):
                    inter_id = f"{node_id}_p{i}"
                    self.graph.add_node(inter_id, pos=p, color='gray', nodeType='SubplanPoint')
                    self.graph.add_edge(last, inter_id)
                    last = inter_id
                self.graph.add_edge(last, g)
            if verbose:
                print(f"[{node_id}] Connected to {len(connections[:2])} Guards")
            return "connection", [c[0] for c in connections[:2]]

    @IPPerfMonitor
    def _learnRoadmap(self, ntry):
        nodeNumber = 0
        currTry = 0
        while currTry < ntry:
            q_pos = self._getRandomFreePosition()
            result, _ = self.learnSinglePoint(q_pos, node_id=nodeNumber)
            if result != "rejected":
                currTry += 1
            nodeNumber += 1

    @IPPerfMonitor
    def planPath(self, startList, goalList, config):
        self.graph.clear()
        checkedStartList, checkedGoalList = self._checkStartGoal(startList, goalList)

        self._learnRoadmap(config["ntry"])

        posList = nx.get_node_attributes(self.graph, 'pos')
        kdTree = cKDTree(list(posList.values()))

        for label, pos in [("start", checkedStartList[0]), ("goal", checkedGoalList[0])]:
            result = kdTree.query(pos, k=5)
            for node in result[1]:
                nearest_node = list(posList.keys())[node]
                if not self._collisionChecker.lineInCollision(pos, self.graph.nodes[nearest_node]['pos']):
                    self.graph.add_node(label, pos=pos, color='green')
                    self.graph.add_edge(label, nearest_node)
                    break

        try:
            return nx.shortest_path(self.graph, "start", "goal")
        except nx.NetworkXNoPath:
            return []
