'''import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPolygon, Rectangle
from matplotlib.widgets import Button
from shapely.geometry import Point, LineString
from environment import get_all_scenes
from hierachial_planner_2 import HierachialPRM
from lectures.IPLazyPRM import LazyPRM
from visibility_connector import VisibilityConnector

# --- Konfiguration ---
subplanner_config_lazy = {
    "initialRoadmapSize": 40,
    "updateRoadmapSize": 20,
    "kNearest": 5,
    "maxIterations": 10
}

# --- Dummy Collision Checker ---
class ShapelyCollisionChecker:
    def __init__(self, obstacles):
        self.obstacles = list(obstacles.values())

    def getDim(self):
        return 2

    def pointInCollision(self, pos):
        return any(ob.contains(Point(pos)) for ob in self.obstacles)

    def lineInCollision(self, p1, p2):
        return any(ob.intersects(LineString([p1, p2])) for ob in self.obstacles)

# --- Setup Environment & Planner ---
scene_name, (obstacles, limits, _) = get_all_scenes()[0]
checker = ShapelyCollisionChecker(obstacles)
planner = HierachialPRM(checker, LazyPRM, subplanner_config_lazy)
connector = VisibilityConnector(checker, LazyPRM, subplanner_config_lazy)

# --- Visualisierung vorbereiten ---
fig, (ax_vis, ax_sub) = plt.subplots(1, 2, figsize=(12, 6))
plt.subplots_adjust(bottom=0.2)

for ax in [ax_vis, ax_sub]:
    ax.set_xlim(*limits[0])
    ax.set_ylim(*limits[1])
    ax.set_aspect('equal')

ax_vis.set_title("Visibility Graph")
ax_sub.set_title("Subplanner Exploration")

# Hindernisse einzeichnen
for ax in [ax_vis, ax_sub]:
    for poly in obstacles.values():
        patch = MplPolygon(list(poly.exterior.coords), closed=True, color='gray', alpha=0.5)
        ax.add_patch(patch)

# --- Speicher ---
nodes = []
node_artists = []
vis_lines = []
sub_lines = []
sub_areas = []
node_id = 1

# --- Interaktion ---
def on_click(event):
    global node_id
    if event.inaxes != ax_vis or event.button != 1:
        return

    pos = (event.xdata, event.ydata)
    print(f"\n→ Clicked at {pos}")

    result, connected = planner.learnSinglePoint(pos, node_id)

    color = {"guard": 'ro', "connection": 'bo', "rejected": 'o'}
    alpha = {"guard": 1.0, "connection": 1.0, "rejected": 0.3}

    if result == "rejected":
        ax_vis.plot(*pos, color[result], alpha=alpha[result])
        ax_sub.plot(*pos, color[result], alpha=alpha[result])
        fig.canvas.draw()
        return

    # Position merken
    nodes.append((node_id, pos))
    a1 = ax_vis.plot(*pos, color[result], alpha=alpha[result])[0]
    a2 = ax_sub.plot(*pos, color[result], alpha=alpha[result])[0]
    node_artists.append((a1, a2))
    node_id += 1
    fig.canvas.draw()


def clear_all(event):
    global nodes, node_artists, vis_lines, sub_lines, sub_areas, node_id
    for a1, a2 in node_artists:
        a1.remove()
        a2.remove()
    for line in vis_lines + sub_lines:
        line.remove()
    for rect in sub_areas:
        rect.remove()
    nodes.clear()
    node_artists.clear()
    vis_lines.clear()
    sub_lines.clear()
    sub_areas.clear()
    node_id = 1
    fig.canvas.draw()

# --- Buttons ---
ax_clear = plt.axes([0.45, 0.05, 0.1, 0.075])
btn_clear = Button(ax_clear, 'Clear All')
btn_clear.on_clicked(clear_all)

fig.canvas.mpl_connect('button_press_event', on_click)
plt.show()
'''

import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPolygon, Rectangle
from matplotlib.widgets import Button
from shapely.geometry import Point, LineString
from environment import get_all_scenes
from hierachial_planner_noLine import HierachialPRM  # <— DEINE DATEI!
from lectures.IPLazyPRM import LazyPRM
import networkx as nx

# --- Konfiguration ---
subplanner_config_lazy = {
    "initialRoadmapSize": 40,
    "updateRoadmapSize": 20,
    "kNearest": 5,
    "maxIterations": 10
}

# --- Dummy Collision Checker ---
class ShapelyCollisionChecker:
    def __init__(self, obstacles):
        self.obstacles = list(obstacles.values())

    def getDim(self):
        return 2

    def pointInCollision(self, pos):
        return any(ob.contains(Point(pos)) for ob in self.obstacles)

    def lineInCollision(self, p1, p2):
        return any(ob.intersects(LineString([p1, p2])) for ob in self.obstacles)

# --- Setup Environment & Planner ---
scene_name, (obstacles, limits, _) = get_all_scenes()[0]
checker = ShapelyCollisionChecker(obstacles)
planner = HierachialPRM(checker, LazyPRM, subplanner_config_lazy)

# --- Visualisierung vorbereiten ---
fig, ax = plt.subplots(figsize=(6, 6))
plt.subplots_adjust(bottom=0.2)
ax.set_xlim(*limits[0])
ax.set_ylim(*limits[1])
ax.set_aspect('equal')
ax.set_title("Subplanner-Only Roadmap")

# Hindernisse zeichnen
for poly in obstacles.values():
    patch = MplPolygon(list(poly.exterior.coords), closed=True, color='gray', alpha=0.5)
    ax.add_patch(patch)

# --- Speicher ---
nodes = []
lines = []
intermediate_nodes = []
node_id = 1

# --- Interaktion ---
def on_click(event):
    global node_id
    if event.inaxes != ax or event.button != 1:
        return

    pos = (event.xdata, event.ydata)
    print(f"\n→ Clicked at {pos}")

    result, connected = planner.learnSinglePoint(pos, node_id=node_id, verbose=True)

    color_map = {
        "guard": "red",
        "connection": "blue",
        "rejected": "gray"
    }

    # Zeichne den Punkt
    if result == "rejected":
        ax.plot(*pos, 'o', color=color_map[result], alpha=0.3)
    else:
        ax.plot(*pos, 'o', color=color_map.get(result, "black"))
        nodes.append(pos)

    # Zeichne Verbindungen
    if result == "connection":
        pos_map = nx.get_node_attributes(planner.graph, 'pos')
        for target_id in connected:
            path = []
            current = node_id
            while current != target_id:
                neighbors = list(planner.graph.neighbors(current))
                next_node = next((n for n in neighbors if n != current), None)
                if next_node is None:
                    break
                path.append(pos_map[current])
                current = next_node
            path.append(pos_map[target_id])
            if len(path) >= 2:
                xs, ys = zip(*path)
                lines.append(ax.plot(xs, ys, color='orange')[0])

    node_id += 1
    fig.canvas.draw()

def clear_all(event):
    global nodes, lines, intermediate_nodes, node_id
    for l in lines:
        l.remove()
    for artist in ax.lines[len(obstacles):]:  # skip obstacle patches
        artist.remove()
    nodes.clear()
    lines.clear()
    node_id = 1
    fig.canvas.draw()

# --- Buttons ---
ax_clear = plt.axes([0.4, 0.05, 0.2, 0.075])
btn_clear = Button(ax_clear, 'Clear All')
btn_clear.on_clicked(clear_all)

fig.canvas.mpl_connect('button_press_event', on_click)
plt.show()
