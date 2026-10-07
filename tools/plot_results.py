"""Optional: redraw presentation charts from results/*.csv and data/*.csv.
Usage: python tools/plot_results.py   (needs matplotlib, pandas-free)"""
import csv, os, math
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
R = os.path.join(ROOT, "results"); OUT = os.path.join(R, "charts"); os.makedirs(OUT, exist_ok=True)
INK, MUTED, GRID, BG = "#1b2430", "#5c6670", "#dcd8cf", "#f6f4ef"
ORANGE, BLUE, TEAL, GREY = "#d9480f", "#1c5d99", "#2a9d8f", "#9aa1a8"
plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 15, "axes.edgecolor": MUTED, "axes.labelcolor": INK,
    "xtick.color": MUTED, "ytick.color": MUTED, "axes.spines.top": False, "axes.spines.right": False,
    "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.8, "figure.facecolor": BG, "axes.facecolor": BG,
    "legend.frameon": False, "axes.titleweight": "bold", "axes.titlesize": 17, "axes.titlecolor": INK})

def read(name):
    with open(os.path.join(R, name)) as f: return list(csv.DictReader(f))
def col(rows, k, t=float): return [t(r[k]) if r[k] != "" else None for r in rows]
def save(fig, name): fig.savefig(os.path.join(OUT, name), dpi=150, bbox_inches="tight"); plt.close(fig)

# 1. routing time vs V — REAL Colombo sub-networks
s = read("scaling_real.csv"); V = col(s, "nodes", int)
fig, ax = plt.subplots(figsize=(8, 5.4))
bfV = [v for v, b in zip(V, col(s, "bellmanford_ms")) if b is not None]; bf = [b for b in col(s, "bellmanford_ms") if b is not None]
ax.loglog(bfV, bf, "o-", color=GREY, lw=2.5, label="Bellman-Ford  O(VE)")
ax.loglog(V, col(s, "dijkstra_dist_ms"), "o-", color=BLUE, lw=2.5, label="Dijkstra (heap)")
ax.loglog(V, col(s, "astar_time_ms"), "s--", color=TEAL, lw=2.5, label="A*, time metric")
ax.loglog(V, col(s, "astar_dist_ms"), "o-", color=ORANGE, lw=3, label="A*, distance metric")
ax.set_xlabel("Junctions V (log) — real Colombo sub-networks"); ax.set_ylabel("Mean query time, ms (log)")
ax.set_title("Query time on the real network"); ax.legend(fontsize=13)
save(fig, "routing_scaling.png")

# 2. settled nodes
fig, ax = plt.subplots(figsize=(8, 5.4))
ax.plot([v/1000 for v in V], [x/1000 for x in col(s, "dijkstra_settled")], "o-", color=BLUE, lw=2.5, label="Dijkstra")
ax.plot([v/1000 for v in V], [x/1000 for x in col(s, "astar_settled")], "o-", color=ORANGE, lw=3, label="A* (distance)")
ax.set_xlabel("Junctions V (thousands)"); ax.set_ylabel("Nodes settled per query (thousands)")
ax.set_title("Search effort: A* settles ~1/4 of the nodes"); ax.legend()
save(fig, "settled_nodes.png")

# 3. linear-time analyses vs E (real)
fig, ax = plt.subplots(figsize=(8, 5.4))
E = [e/1000 for e in col(s, "edges", int)]
ax.plot(E, col(s, "tarjan_ms"), "o-", color=ORANGE, lw=3, label="Tarjan bridges/APs")
ax.plot(E, col(s, "dijkstra_dist_ms"), "o-", color=BLUE, lw=2.5, label="Dijkstra query")
ax.plot(E, col(s, "bfs_conn_ms"), "o-", color=TEAL, lw=2.5, label="BFS connectivity")
ax.plot(E, col(s, "dfs_conn_ms"), "o--", color=GREY, lw=2.5, label="DFS connectivity")
ax.set_xlabel("Directed edges E (thousands) — real Colombo"); ax.set_ylabel("Time, ms")
ax.set_title("Core operations grow ~linearly with E"); ax.legend(fontsize=13)
save(fig, "linear_ops.png")

# 3b. synthetic extrapolation to 250k
t = read("scaling_sparse.csv"); tv = col(t, "nodes", int)
fig, ax = plt.subplots(figsize=(8, 5.4))
ax.loglog(tv, col(t, "dijkstra_ms"), "o-", color=BLUE, lw=2.5, label="Dijkstra")
ax.loglog(tv, col(t, "astar_ms"), "o-", color=ORANGE, lw=3, label="A*")
ax.loglog(tv, col(t, "tarjan_ms"), "s--", color=TEAL, lw=2.5, label="Tarjan")
ax.set_xlabel("Nodes V (log) — synthetic cities"); ax.set_ylabel("ms (log)"); ax.set_title("Beyond the district: synthetic cities to 250k"); ax.legend()
save(fig, "synthetic_scaling.png")

# 4. memory list vs matrix
r = read("representation.csv"); rv = col(r, "nodes", int)
fig, ax = plt.subplots(figsize=(8, 5.4))
x = range(len(rv)); w = 0.38
ax.bar([i - w/2 for i in x], col(r, "list_mb"), w, color=ORANGE, label="Adjacency list  O(V+E)")
ax.bar([i + w/2 for i in x], col(r, "matrix_mb"), w, color=GREY, label="Adjacency matrix  Θ(V²)")
ax.set_yscale("log"); ax.set_xticks(list(x)); ax.set_xticklabels([f"{v:,}" for v in rv])
for i, (a, b) in enumerate(zip(col(r, "list_mb"), col(r, "matrix_mb"))):
    ax.text(i + w/2, b * 1.15, f"{b:.0f} MB", ha="center", fontsize=12, color=INK)
    ax.text(i - w/2, a * 1.15, f"{a:.1f}", ha="center", fontsize=12, color=ORANGE)
ax.set_xlabel("Junctions V (real Colombo sub-networks)"); ax.set_ylabel("Memory, MB (log)"); ax.set_title("Measured memory: list vs matrix"); ax.legend(loc="upper left")
save(fig, "memory.png")

# 5. heap vs array, sparse vs dense
d = read("dense_vs_sparse.csv")
fig, axs = plt.subplots(1, 2, figsize=(11, 4.8))
for ax, kind, title in [(axs[0], "sparse", "Sparse road network (E≈3.7V)"), (axs[1], "dense", "Dense transit network (E≈V²/2)")]:
    rows = [q for q in d if q["graph"] == kind]; n = col(rows, "nodes", int)
    ax.plot(n, col(rows, "heap_dijkstra_ms"), "o-", color=BLUE, lw=2.5, label="Heap  O((V+E)logV)")
    ax.plot(n, col(rows, "array_dijkstra_ms"), "o-", color=ORANGE, lw=2.5, label="Array  O(V²)")
    ax.set_title(title, fontsize=15); ax.set_xlabel("Nodes V"); ax.set_ylabel("ms")
axs[0].legend(fontsize=12)
save(fig, "heap_vs_array.png")

# 6. reachability budget
b = read("reachability_budget.csv")
fig, ax = plt.subplots(figsize=(8, 5.4))
ax.plot(col(b, "budget_min"), col(b, "nodes_reached", int), "o-", color=ORANGE, lw=3, label="Nodes reached")
ax.set_xlabel("Travel-time budget, minutes"); ax.set_ylabel("Nodes reached", color=ORANGE)
ax2 = ax.twinx(); ax2.plot(col(b, "budget_min"), col(b, "time_ms"), "s--", color=BLUE, lw=2.5, label="Query time")
ax2.set_ylabel("Query time, ms", color=BLUE); ax2.spines["right"].set_visible(True); ax2.grid(False)
ax.set_title("Reachability from Town Hall (real network)")
save(fig, "reachability.png")

# 7. Real Colombo map + critical zones
from matplotlib.collections import LineCollection
D = os.path.join(ROOT, "data")
N = {}
for r in csv.DictReader(open(os.path.join(D, "colombo_nodes_osm.csv"))): N[r["osmid"]] = (float(r["x"]), float(r["y"]))
cls = {"motorway": "hw", "trunk": "art", "primary": "art", "secondary": "sec", "tertiary": "sec"}
segs = {"hw": [], "art": [], "sec": [], "res": []}
for r in csv.DictReader(open(os.path.join(D, "colombo_edges_osm.csv"))):
    if r["u"] not in N or r["v"] not in N or r["u"] > r["v"] and r["oneway"] == "False": continue
    h = r["highway"].strip("[]'\" ").split("'")[0].split(",")[0].replace("_link", "")
    segs[cls.get(h, "res")].append([N[r["u"]], N[r["v"]]])
def net(ax, lw=1.0):
    for k, c, w in [("res", "#c9c4b8", 0.35), ("sec", "#7d9bbb", 0.8), ("art", BLUE, 1.5), ("hw", TEAL, 2.2)]:
        ax.add_collection(LineCollection(segs[k], colors=c, linewidths=w * lw, zorder=1))
    ax.set_aspect(1 / math.cos(math.radians(6.9))); ax.grid(False); ax.set_xticks([]); ax.set_yticks([])
    for s_ in ["left", "bottom"]: ax.spines[s_].set_visible(False)
zones = read("critical_zones.csv"); crit = read("critical_roads.csv")
fig, ax = plt.subplots(figsize=(9, 9.6)); net(ax)
ax.scatter([float(z["lon"]) for z in zones], [float(z["lat"]) for z in zones], s=6, color="#b00020", zorder=3)
for c in crit[:5]:
    ax.plot([float(c["lon_a"]), float(c["lon_b"])], [float(c["lat_a"]), float(c["lat_b"])], color="#b00020", lw=5, zorder=4)
LM = {"Colombo Fort": (6.9344, 79.8428), "Moratuwa": (6.7730, 79.8820), "Malabe": (6.9040, 79.9580), "Homagama": (6.8440, 80.0020),
      "Mattakkuliya": (6.9740, 79.8790), "Avissawella": (6.9530, 80.2100)}
for nme, (la, lo) in LM.items():
    ax.scatter(lo, la, s=70, color=INK, zorder=5); ax.text(lo + 0.006, la + 0.004, nme, fontsize=13, fontweight="bold", color=INK, zorder=6)
ax.set_xlim(79.83, 80.24); ax.set_ylim(6.69, 7.0)
from matplotlib.lines import Line2D
ax.legend(handles=[Line2D([0],[0],color=TEAL,lw=3,label="Expressway"), Line2D([0],[0],color=BLUE,lw=3,label="Trunk / primary"),
    Line2D([0],[0],color="#7d9bbb",lw=2,label="Secondary / tertiary"), Line2D([0],[0],color="#c9c4b8",lw=2,label="Residential"),
    Line2D([0],[0],color="#b00020",lw=0,marker="o",label="Isolated if a top-5 critical road fails")], loc="lower right", fontsize=11)
save(fig, "colombo_map.png")

# 8. Galle Road jam re-route (zoom)
rt = read("demo_routes.csv")
fig, ax = plt.subplots(figsize=(7.6, 9.6)); net(ax, 1.4)
for lab, c, w, z, txt in [("before_jam", GREY, 7, 2, "Before jam: Galle Road, 35.6 min"), ("after_jam", ORANGE, 4, 3, "After jam: inland re-route, 47.5 min")]:
    pts = [(float(r["lon"]), float(r["lat"])) for r in rt if r["route"] == lab]
    ax.plot([p[0] for p in pts], [p[1] for p in pts], color=c, lw=w, zorder=z, solid_capstyle="round", label=txt)
for nme, dx, ha in [("Colombo Fort", 0.004, "left"), ("Moratuwa", -0.004, "right")]:
    la, lo = LM[nme]; ax.scatter(lo, la, s=90, color=INK, zorder=5); ax.text(lo + dx, la, nme, fontsize=14, fontweight="bold", ha=ha, zorder=6)
ax.set_xlim(79.83, 79.93); ax.set_ylim(6.735, 6.945)
leg = ax.legend(loc="lower right", fontsize=12, frameon=True, facecolor=BG, edgecolor=LINE if False else GRID, framealpha=1)
save(fig, "jam_reroute.png")
print("charts ->", OUT)
