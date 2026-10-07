# Intelligent Urban Routing Engine — Greater Colombo
**COMP50065 Data Structures & Algorithms — Scenario 01**
Author: Yaqoob Sadikeen · Language: **C# 12 / .NET 8**

A routing-engine prototype for a smart-mobility platform: optimal paths (distance / time / weighted),
constrained routing (heavy vehicles, closures, congestion), time-budget reachability, connectivity and
critical-infrastructure detection, plus an empirical benchmarking suite.

## 1. Requirements
| Item | Version |
|---|---|
| .NET SDK | **8.0** or later (`dotnet --version`) — download: https://dotnet.microsoft.com/download |
| OS | Windows, macOS or Linux |
| External packages | **None** (only the .NET base class library) |
| Optional | Python 3 + matplotlib, only to redraw charts (`tools/plot_results.py`) |

## 2. How to run
From this folder:
```bash
dotnet run -c Release -- demo        # real Colombo District walkthrough (default)
dotnet run -c Release -- test        # 64 automated correctness & worst-case tests
dotnet run -c Release -- bench       # full scaling benchmarks → results/*.csv (~2–4 min)
dotnet run -c Release -- bench --quick
dotnet run -c Release -- bench sparse   # one section: real | sparse | dense | repr | reach
dotnet run -c Release -- generate 5000 data/generated   # write a synthetic city as CSV
dotnet run -c Release -- all         # test + demo + bench
```
Always use `-c Release` for timings (Debug builds disable JIT optimisations).
Visual Studio / Rider: open `UrbanRouting.csproj`, set command-line arguments (e.g. `demo`) in the run profile.

## 3. Project structure
```
UrbanRouting.csproj
Program.cs                         CLI entry point + demo scenario
src/Models/      Node, Edge (derived travel time), enums
src/Graph/       RoadNetwork (CHOSEN: adjacency list + hash index)
                 AdjacencyMatrixNetwork (rejected alternative, benchmark only)
src/Routing/     CostModel (metric + admissible heuristic), RouteConstraints, PathResult
                 DijkstraRouter (binary heap)  ← general engine
                 AStarRouter                   ← point-to-point queries (chosen)
                 BellmanFordRouter, ArrayDijkstraRouter  ← compared alternatives
src/Analysis/    ReachabilityAnalyzer (bounded Dijkstra; BFS for hop budgets)
                 ConnectivityAnalyzer (BFS vs DFS, weak & strong connectivity)
                 BottleneckAnalyzer (iterative Tarjan: articulation points & bridges)
src/Data/        OsmLoader (real OpenStreetMap CSVs), DatasetLoader (generic CSV), SyntheticGenerator (seeded)
src/Benchmark/   Benchmarker → results/*.csv
src/Tests/       SelfTests (no test framework needed)
data/            colombo_nodes_osm.csv, colombo_edges_osm.csv (OpenStreetMap, Colombo District)
results/         benchmark CSVs + charts used in the presentation
tools/           plot_results.py (optional chart generator)
```

## 4. Datasets
### 4.1 Baseline — REAL Colombo District road network (`data/colombo_*_osm.csv`)
* **Source:** OpenStreetMap contributors (ODbL licence), drivable network of "Colombo District, Sri Lanka",
  extracted with OSMnx (`graph_from_place(..., network_type="drive")`, `add_edge_speeds`, `add_edge_travel_times`),
  exported to CSV in Google Colab.
* `colombo_nodes_osm.csv` — `osmid, y (lat), x (lon), street_count` → **43,744 junctions**.
* `colombo_edges_osm.csv` — `u, v, length (m), highway, oneway, speed_kph, travel_time, name, bridge` → 98,439 rows.
* After cleaning (`OsmLoader`): **51,078 roads = 98,233 directed edges** (3,923 one-way), average out-degree 2.25,
  1 weakly connected component.

| Cleaning step | Count | Why |
|---|---|---|
| Self-loops removed (u = v) | 206 | can never be on a shortest path |
| Reverse rows merged (u→v + v→u = one two-way road) | 47,155 | bridge detection must see one physical road |
| Lengths raised to straight-line distance | 9,055 roads (median 4 cm, max 0.7 m) | keeps the A* heuristic provably admissible |
| Rows rejected | 0 | — |

Attribute mapping: `highway` motorway → Highway; trunk/primary → Arterial; secondary/tertiary → Secondary;
residential/living_street/unclassified → Residential; `bridge` = yes/viaduct → Bridge. `speed_kph` is OSM
`maxspeed` where tagged, otherwise OSMnx's mean for that road type. **Congestion is not in OpenStreetMap**, so it
is an assumed weekday-peak level per class (Highway 0.15, Arterial/Bridge 0.60, Secondary 0.45, Residential 0.25).
Travel time = `distance / (speed × (1 − 0.75 × congestion))`. Landmarks (Fort, Town Hall, hospitals…) use approximate
coordinates snapped to the nearest main-road junction.

### 4.2 Scaling
* **Real:** nested sub-networks of the OSM graph (the k junctions nearest Colombo Fort, k = 1,000 … 43,744).
* **Synthetic (seeded, `SyntheticGenerator`)** for sizes beyond the district: grid city to 250,000 nodes,
  dense transit graph (E ≈ V²/2), 300,000-node chain (worst-case depth). `generate <n>` writes one as CSV.

## 5. Key results (reproduce with `bench` / `demo`; raw data in `results/`)
| Finding | Measured |
|---|---|
| A* vs Dijkstra, 300 random real trips | 0 cost mismatches; A* settles 27 % of the nodes (distance), 80 % (time) |
| Full network query (43,744 junctions) | Dijkstra 14.9 ms, A* 4.0 ms (distance) |
| Real scaling 1k → 43.7k junctions (E ×50) | Dijkstra ×62, Tarjan ×45, memory ×43 → near-linear |
| Bellman-Ford vs Dijkstra, 20k real junctions | 138 ms vs 5.1 ms |
| List vs matrix, 8,000 real junctions | 3.5 MB vs 488 MB (full district matrix ≈ 15.3 GB) |
| Tarjan on the full network | 15,720 articulation points, 18,922 bridges in 22 ms (warm median; ~40–140 ms on a cold first run); 18 bridges isolate ≥ 50 junctions |
| Top critical roads | High Level Road near Avissawella (484 junctions), Mattakkuliya Church Road (86) |
| Galle Road jam | re-route 47.5 min vs 62.5 min if the old route were still driven |
| Hash-index fix (SplitMix64 comparer) | building a 40k-node network: 10.9 s → 47 ms |

## 6. Worst cases handled
Invalid/negative node ids · source = destination · unreachable destination · blocked source/destination ·
constraints that disconnect the graph · negative/zero/NaN distances · self-loops · parallel roads ·
congestion outside range (clamped so speed > 0) · empty graph · 300,000-node chain (iterative DFS/Tarjan, no stack
overflow) · malformed/missing CSV input · quoted OSM fields & tag lists · reverse-row duplicates · hash-collision degradation of the edge index (custom hash).
