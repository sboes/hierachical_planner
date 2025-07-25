import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPolygon
from matplotlib.widgets import Button
from shapely.geometry import Point, LineString
from environment import get_all_scenes
from hierachial_planner_2 import HierachialPRM
from lectures.IPLazyPRM import LazyPRM
from scipy.spatial.distance import euclidean
import networkx as nx

# Subplanner-Konfiguration
subplanner_config_lazy = {
    "initialRoadmapSize": 40,
    "updateRoadmapSize": 20,
    "kNearest": 5,
    "maxIterations": 10
}

# CollisionChecker
class ShapelyCollisionChecker:
    def __init__(self, obstacles):
        self.obstacles = list(obstacles.values())

    def getDim(self):
        return 2

    def pointInCollision(self, pos):
        point = Point(pos)
        return any(ob.contains(point) for ob in self.obstacles)

    def lineInCollision(self, p1, p2):
        line = LineString([p1, p2])
        return any(ob.intersects(line) for ob in self.obstacles)

# Szene laden
scene_name, (obstacles, limits, (start, goal)) = get_all_scenes()[0]
checker = ShapelyCollisionChecker(obstacles)
planner = HierachialPRM(checker, LazyPRM, subplanner_config_lazy)

# Plot vorbereiten
fig, ax = plt.subplots()
plt.subplots_adjust(bottom=0.25)
ax.set_xlim(*limits[0])
ax.set_ylim(*limits[1])
ax.set_aspect('equal')
ax.set_title(f"Scene: {scene_name} — Click to add Nodes")

# Hindernisse zeichnen
for poly in obstacles.values():
    patch = MplPolygon(list(poly.exterior.coords), closed=True, color='gray', alpha=0.5)
    ax.add_patch(patch)

# Speicher für Nodes und Verbindungen
nodes = []
node_artists = []
connection_lines = []

# Graph zur Gruppierung von verbundenen Komponenten
G = nx.Graph()

# Klick → neuen Node hinzufügen
def on_click(event):
    if event.button == 1 and event.inaxes == ax:
        pos = (event.xdata, event.ydata)
        if not checker.pointInCollision(pos):
            index = len(nodes)
            nodes.append(pos)
            G.add_node(index, pos=pos)
            pt = ax.plot(pos[0], pos[1], 'ro')[0]
            ax.text(pos[0]+0.2, pos[1]+0.2, f"{index+1}", fontsize=9)
            node_artists.append(pt)
            fig.canvas.draw()
            print(f"Added node #{index+1} at {pos}")

# Button-Action: Sichtverbindungen & Subplan prüfen
def check_visibility(event=None):
    G.clear_edges()
    for line in connection_lines:
        line.remove()
    connection_lines.clear()

    for i, pos1 in enumerate(nodes):
        for j, pos2 in enumerate(nodes):
            if i >= j:
                continue
            if not checker.lineInCollision(pos1, pos2):
                ln, = ax.plot([pos1[0], pos2[0]], [pos1[1], pos2[1]], color='green', linewidth=1.5)
                connection_lines.append(ln)
                G.add_edge(i, j)
            elif planner._subplan_connection(pos1, pos2):
                ln, = ax.plot([pos1[0], pos2[0]], [pos1[1], pos2[1]], color='blue', linestyle='--', linewidth=1.2)
                connection_lines.append(ln)
                G.add_edge(i, j)

    # Knoten einfärben je nach Status
    for artist in node_artists:
        artist.set_color('red')  # Standard: unverbunden

    for comp in nx.connected_components(G):
        for idx in comp:
            node_artists[idx].set_color('yellow')  # Verbunden

    fig.canvas.draw()

# Button-Action: Alles löschen
def clear_all(event):
    for artist in node_artists + connection_lines:
        artist.remove()
    nodes.clear()
    node_artists.clear()
    connection_lines.clear()
    G.clear()
    fig.canvas.draw()

# Buttons definieren
ax_check = plt.axes([0.1, 0.05, 0.3, 0.075])
ax_clear = plt.axes([0.45, 0.05, 0.2, 0.075])
btn_check = Button(ax_check, 'Check Visibility/Subplan')
btn_clear = Button(ax_clear, 'Clear All')

btn_check.on_clicked(check_visibility)
btn_clear.on_clicked(clear_all)
fig.canvas.mpl_connect('button_press_event', on_click)

plt.show()
