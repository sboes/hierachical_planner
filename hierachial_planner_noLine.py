# coding: utf-8

from lectures.IPPRMBase import PRMBase
import networkx as nx
import numpy as np
from lectures.IPPerfMonitor import IPPerfMonitor
from scipy.spatial import distance


class HierarchicalPRMBase(PRMBase):
    def __init__(self, _collChecker, _internalPlanner):
        super(HierarchicalPRMBase, self).__init__(_collChecker)
        self.internalPlanner = _internalPlanner
        self.graph = nx.Graph()
        self.subplanner_graphs = {}  # Stores subplanner roadmaps by center node
        self.debug = False
        self.stats = {
            'nodes_created': 0,
            'nodes_discarded': 0,
            'connections_attempted': 0,
            'connections_successful': 0,
            'subplanner_calls': 0
        }

    def set_debug(self, debug_mode):
        self.debug = debug_mode
        if hasattr(self.internalPlanner, 'set_debug'):
            self.internalPlanner.set_debug(debug_mode)

    def _configureInternalPlanner(self, center_pos, config):
        """Configure the internal planner's search space around center_pos"""
        bbox_size = config.get("boundingBoxSize", 2.0)
        min_coords = np.array(center_pos) - bbox_size / 2
        max_coords = np.array(center_pos) + bbox_size / 2

        # Clip to environment boundaries
        env_limits = self._collisionChecker.getEnvironmentLimits()
        for i in range(len(min_coords)):
            min_coords[i] = max(min_coords[i], env_limits[i][0])
            max_coords[i] = min(max_coords[i], env_limits[i][1])

        if hasattr(self.internalPlanner, 'setSamplingBounds'):
            bounds = list(zip(min_coords, max_coords))
            self.internalPlanner.setSamplingBounds(bounds)

        if self.debug:
            print(f"🔳 Bounding Box @ {center_pos}: x=[{min_coords[0]:.2f}, {max_coords[0]:.2f}], "
                  f"y=[{min_coords[1]:.2f}, {max_coords[1]:.2f}]")

    def _findNodesInBoundingBox(self, center_pos, radius):
        """Find existing nodes within radius of center_pos"""
        posList = nx.get_node_attributes(self.graph, 'pos')
        if not posList:
            return []

        nodes_in_radius = []
        for node, pos in posList.items():
            if distance.euclidean(center_pos, pos) <= radius:
                nodes_in_radius.append(node)
        return nodes_in_radius

    def _processNewNode(self, new_pos, config):
        """Handle a new node with sub-planner in local bounding box"""
        node_id = len(self.graph.nodes)
        self.graph.add_node(node_id, pos=new_pos, color='yellow', nodeType='Candidate')
        self.stats['nodes_created'] += 1

        # Configure sub-planner for this node's region
        self._configureInternalPlanner(new_pos, config)

        # Find existing nodes in bounding box
        bbox_radius = config.get("boundingBoxSize", 2.0) / 2
        nearby_nodes = self._findNodesInBoundingBox(new_pos, bbox_radius)

        if self.debug:
            print(f"🔍 New node {node_id} @ {new_pos} has {len(nearby_nodes)} nearby nodes")

        # Try to connect to nearby nodes using sub-planner
        successful_connections = []
        connection_paths = {}

        for existing_node in nearby_nodes:
            existing_pos = self.graph.nodes[existing_node]['pos']

            # Skip if already in same connected component
            if nx.has_path(self.graph, node_id, existing_node):
                continue

            self.stats['connections_attempted'] += 1
            self.stats['subplanner_calls'] += 1

            # Use sub-planner to check connection
            self.internalPlanner.graph.clear()
            path = self.internalPlanner.planPath([new_pos], [existing_pos],
                                                 config.get("internalConfig", {}))

            if path:
                self.stats['connections_successful'] += 1
                successful_connections.append(existing_node)
                connection_paths[existing_node] = path

                if self.debug:
                    print(f"  ✅ Connected to node {existing_node} via sub-planner path")
            else:
                if self.debug:
                    print(f"  ❌ Failed to connect to node {existing_node}")

        # Store sub-planner roadmap for visualization
        self.subplanner_graphs[node_id] = self.internalPlanner.graph.copy()

        # Apply connection logic
        if config.get("discardOverconnected", True) and len(successful_connections) >= 3:
            # Node is over-connected - discard it
            self.graph.remove_node(node_id)
            self.stats['nodes_discarded'] += 1
            if self.debug:
                print(f"🗑️ Discarding node {node_id} (over-connected to {len(successful_connections)} nodes)")
            return False

        # Keep the node and add successful connections
        for connected_node in successful_connections:
            self.graph.add_edge(node_id, connected_node)
            # Store path information in edge attributes
            self.graph.edges[node_id, connected_node]['path'] = connection_paths[connected_node]

        # Determine node type based on connection results
        if len(successful_connections) >= 2:
            node_type = 'Connection'
            color = 'lightblue'
        else:
            node_type = 'Guard'
            color = 'red'

        nx.set_node_attributes(self.graph, {node_id: {'nodeType': node_type, 'color': color}})

        return True

    @IPPerfMonitor
    def _learnRoadmap(self, config):
        """Learn the hierarchical roadmap"""
        self.graph.clear()
        self.subplanner_graphs.clear()
        self.stats = {k: 0 for k in self.stats}

        num_nodes = config.get("numNodes", 50)
        bbox_size = config.get("boundingBoxSize", 2.0)

        if self.debug:
            print(f"🏗️ Building hierarchical roadmap with {num_nodes} nodes (bbox size: {bbox_size})")

        for _ in range(num_nodes):
            q_pos = self._getRandomFreePosition()
            self._processNewNode(q_pos, config)

        if self.debug:
            print("\n📊 Roadmap Statistics:")
            print(f"- Nodes created: {self.stats['nodes_created']}")
            print(f"- Nodes discarded: {self.stats['nodes_discarded']}")
            print(f"- Connection attempts: {self.stats['connections_attempted']}")
            print(f"- Successful connections: {self.stats['connections_successful']}")
            print(f"- Sub-planner calls: {self.stats['subplanner_calls']}")
            print(f"- Final roadmap size: {len(self.graph.nodes)} nodes, {len(self.graph.edges)} edges")

    @IPPerfMonitor
    def planPath(self, startList, goalList, config):
        """Main planning method"""
        self.set_debug(config.get("debug", False))

        # 1. Learn the hierarchical roadmap
        self._learnRoadmap(config)

        # 2. Check start and goal
        checkedStartList, checkedGoalList = self._checkStartGoal(startList, goalList)
        start_pos = checkedStartList[0]
        goal_pos = checkedGoalList[0]

        # 3. Connect start and goal to roadmap
        start_id = "start"
        goal_id = "goal"
        self.graph.add_node(start_id, pos=start_pos, color='lightgreen', nodeType='Terminal')
        self.graph.add_node(goal_id, pos=goal_pos, color='lightgreen', nodeType='Terminal')

        # Find nearest nodes within bounding box
        bbox_radius = config.get("boundingBoxSize", 2.0) / 2
        start_neighbors = self._findNodesInBoundingBox(start_pos, bbox_radius)
        goal_neighbors = self._findNodesInBoundingBox(goal_pos, bbox_radius)

        # Try to connect start and goal using sub-planner
        start_connected = False
        goal_connected = False

        for node in start_neighbors:
            self.internalPlanner.graph.clear()
            path = self.internalPlanner.planPath([start_pos], [self.graph.nodes[node]['pos']],
                                                 config.get("internalConfig", {}))
            if path:
                self.graph.add_edge(start_id, node)
                self.graph.edges[start_id, node]['path'] = path
                start_connected = True
                break

        for node in goal_neighbors:
            self.internalPlanner.graph.clear()
            path = self.internalPlanner.planPath([goal_pos], [self.graph.nodes[node]['pos']],
                                                 config.get("internalConfig", {}))
            if path:
                self.graph.add_edge(goal_id, node)
                self.graph.edges[goal_id, node]['path'] = path
                goal_connected = True
                break

        if not (start_connected and goal_connected):
            if self.debug:
                print("❌ Failed to connect start or goal to roadmap")
            return []

        # 4. Find path in main roadmap
        try:
            node_path = nx.shortest_path(self.graph, start_id, goal_id)
        except nx.NetworkXNoPath:
            if self.debug:
                print("❌ No path found in roadmap")
            return []

        # 5. Reconstruct full path using stored sub-planner paths
        full_path = [start_pos]

        for i in range(len(node_path) - 1):
            u, v = node_path[i], node_path[i + 1]

            if 'path' in self.graph.edges[u, v]:
                # Use stored sub-planner path
                path_segment = self.graph.edges[u, v]['path']
                full_path.extend(path_segment[1:])  # Skip first point to avoid duplicates
            else:
                # Direct connection (shouldn't happen with current implementation)
                full_path.append(self.graph.nodes[v]['pos'])

        return full_path


class VisPRM_with_LazyPRM(HierarchicalPRMBase):
    def __init__(self, _collChecker):
        from lectures.IPLazyPRM import LazyPRM
        internal_planner = LazyPRM(_collChecker)
        super(VisPRM_with_LazyPRM, self).__init__(_collChecker, internal_planner)

    def _getInternalPlannerConfig(self):
        return {
            "initialRoadmapSize": 15,
            "updateRoadmapSize": 5,
            "kNearest": 5,
            "maxIterations": 10
        }


class VisPRM_with_BasicPRM(HierarchicalPRMBase):
    def __init__(self, _collChecker):
        from lectures.IPBasicPRM import BasicPRM
        internal_planner = BasicPRM(_collChecker)
        super(VisPRM_with_BasicPRM, self).__init__(_collChecker, internal_planner)

    def _getInternalPlannerConfig(self):
        return {
            "radius": 1.0,
            "numNodes": 20,
            "useKDTree": True
        }