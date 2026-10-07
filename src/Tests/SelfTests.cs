using UrbanRouting.Analysis;
using UrbanRouting.Data;
using UrbanRouting.Graph;
using UrbanRouting.Models;
using UrbanRouting.Routing;

namespace UrbanRouting.Tests;

/// <summary>
/// Dependency-free test suite (no NuGet needed). Covers correctness AND the worst/edge cases the brief asks for.
/// Run: dotnet run -c Release -- test
/// </summary>
public static class SelfTests
{
    private static int _pass, _fail;
    private static readonly List<string> Failures = new();

    private static void Check(string name, bool ok, string detail = "")
    {
        if (ok) { _pass++; Console.WriteLine($"  PASS  {name}"); }
        else { _fail++; Failures.Add(name); Console.WriteLine($"  FAIL  {name} {detail}"); }
    }

    private static void Throws<T>(string name, Action a) where T : Exception
    {
        try { a(); Check(name, false, "(no exception)"); }
        catch (T) { Check(name, true); }
        catch (Exception ex) { Check(name, false, $"(wrong exception {ex.GetType().Name})"); }
    }

    private static readonly IPathFinder Dij = new DijkstraRouter(), Ast = new AStarRouter(),
                                        Bf = new BellmanFordRouter(), Arr = new ArrayDijkstraRouter();

    /// <summary>Hand-checkable graph:
    ///   A --1-- B --1-- C          A→D→C = 1.5+1.5 = 3.0 (Residential)
    ///   |               |          A→B→C = 2.0      (Secondary)   ← shortest
    ///   +--1.5-- D -1.5-+          E isolated
    /// </summary>
    private static RoadNetwork Small()
    {
        var g = new RoadNetwork();
        g.AddNode("A", 0, 0, 0, 0); g.AddNode("B", 0, 0, 1, 0); g.AddNode("C", 0, 0, 2, 0);
        g.AddNode("D", 0, 0, 1, -1); g.AddNode("E", 0, 0, 9, 9);
        g.AddRoad(0, 1, 1.0, 40, RoadType.Secondary, 0, true);   // road 0
        g.AddRoad(1, 2, 1.0, 40, RoadType.Secondary, 0, true);   // road 1
        g.AddRoad(0, 3, 1.5, 40, RoadType.Residential, 0, true); // road 2
        g.AddRoad(3, 2, 1.5, 40, RoadType.Residential, 0, true); // road 3
        return g;
    }

    public static bool RunAll(string dataDir)
    {
        _pass = _fail = 0; Failures.Clear();
        Console.WriteLine("== 1. Core shortest-path correctness ==");
        var g = Small();
        foreach (var alg in new[] { Dij, Ast, Bf, Arr })
        {
            var r = alg.FindPath(g, 0, 2, CostModel.ShortestDistance);
            Check($"{alg.Name}: A→C cost 2.0 via B", r.Found && Math.Abs(r.TotalCost - 2.0) < 1e-9 && r.Nodes.SequenceEqual(new[] { 0, 1, 2 }));
        }

        Console.WriteLine("== 2. Edge / worst cases ==");
        var same = Dij.FindPath(g, 1, 1, CostModel.ShortestDistance);
        Check("source == destination → cost 0, single-node path", same.Found && same.TotalCost == 0 && same.Nodes.Count == 1);
        var unreach = Dij.FindPath(g, 0, 4, CostModel.ShortestDistance);
        Check("isolated destination → graceful 'not found'", !unreach.Found);
        Check("unreachable search settles only its own component", unreach.NodesSettled == 4);
        Throws<ArgumentOutOfRangeException>("invalid node id rejected", () => Dij.FindPath(g, 0, 99, CostModel.ShortestDistance));
        Throws<ArgumentOutOfRangeException>("negative node id rejected", () => Dij.FindPath(g, -1, 2, CostModel.ShortestDistance));
        Throws<ArgumentOutOfRangeException>("negative distance rejected at insert", () => g.AddRoad(0, 1, -5, 40, RoadType.Secondary, 0, true));
        Throws<ArgumentOutOfRangeException>("zero distance rejected", () => g.AddRoad(0, 1, 0, 40, RoadType.Secondary, 0, true));
        Throws<ArgumentException>("self-loop rejected", () => g.AddRoad(2, 2, 1, 40, RoadType.Secondary, 0, true));
        Throws<ArgumentOutOfRangeException>("negative reachability budget rejected", () => ReachabilityAnalyzer.WithinBudget(g, 0, -1, CostModel.FastestTime));
        var e1 = new Edge(0, 1, 0, 1, 40, RoadType.Secondary, 5.0);
        Check("congestion clamped to 0.95 (speed never 0 → no ∞/NaN times)", e1.Congestion == Edge.MaxCongestion && double.IsFinite(e1.TravelTimeMin));
        var empty = new RoadNetwork();
        Check("empty network counts as connected (no crash)", ConnectivityAnalyzer.IsWeaklyConnected(empty));
        Check("empty network: no bottlenecks (no crash)", BottleneckAnalyzer.Analyze(empty).Bridges.Count == 0);

        var par = new RoadNetwork();
        par.AddNode("P", 0, 0, 0, 0); par.AddNode("Q", 0, 0, 1, 0);
        par.AddRoad(0, 1, 5, 40, RoadType.Secondary, 0, true);
        par.AddRoad(0, 1, 2, 40, RoadType.Secondary, 0, true);
        Check("parallel roads: cheaper one chosen", Math.Abs(Dij.FindPath(par, 0, 1, CostModel.ShortestDistance).TotalCost - 2) < 1e-9);
        Check("parallel roads: NOT reported as a bridge", BottleneckAnalyzer.Analyze(par).Bridges.Count == 0);

        Console.WriteLine("== 3. Constrained routing ==");
        var heavy = Dij.FindPath(g, 0, 2, CostModel.ShortestDistance, new RouteConstraints().Exclude(RoadType.Secondary));
        Check("excluding Secondary forces A→D→C (3.0)", heavy.Found && Math.Abs(heavy.TotalCost - 3.0) < 1e-9);
        Check("result contains no excluded road type", heavy.Edges.All(e => e.Type != RoadType.Secondary));
        var blockedB = Ast.FindPath(g, 0, 2, CostModel.ShortestDistance, new RouteConstraints().BlockNode(1));
        Check("blocked node B is avoided (A*)", blockedB.Found && !blockedB.Nodes.Contains(1));
        var blockedRoad = Dij.FindPath(g, 0, 2, CostModel.ShortestDistance, new RouteConstraints().BlockRoad(1));
        Check("closed road (B–C) is avoided", blockedRoad.Found && blockedRoad.Edges.All(e => e.RoadId != 1));
        var impossible = Dij.FindPath(g, 0, 2, CostModel.ShortestDistance, new RouteConstraints().BlockNode(1).BlockNode(3));
        Check("constraints that disconnect → 'not found', no crash", !impossible.Found);
        Check("blocked destination → 'not found'", !Dij.FindPath(g, 0, 2, CostModel.ShortestDistance, new RouteConstraints().BlockNode(2)).Found);
        Check("blocked source → 'not found'", !Dij.FindPath(g, 0, 2, CostModel.ShortestDistance, new RouteConstraints().BlockNode(0)).Found);

        var ow = new RoadNetwork();
        ow.AddNode("X", 0, 0, 0, 0); ow.AddNode("Y", 0, 0, 1, 0); ow.AddNode("Z", 0, 0, 1, 1);
        ow.AddRoad(0, 1, 1, 40, RoadType.Secondary, 0, twoWay: false);   // X→Y one-way
        ow.AddRoad(1, 2, 1, 40, RoadType.Secondary, 0, true);
        ow.AddRoad(2, 0, 3, 40, RoadType.Secondary, 0, true);
        Check("one-way respected: X→Y = 1", Math.Abs(Dij.FindPath(ow, 0, 1, CostModel.ShortestDistance).TotalCost - 1) < 1e-9);
        Check("one-way respected: Y→X must detour = 4", Math.Abs(Dij.FindPath(ow, 1, 0, CostModel.ShortestDistance).TotalCost - 4) < 1e-9);

        Console.WriteLine("== 4. Metrics & dynamic congestion ==");
        var cg = new RoadNetwork();
        cg.AddNode("S", 0, 0, 0, 0); cg.AddNode("M", 0, 0, 1, 0); cg.AddNode("T", 0, 0, 2, 0); cg.AddNode("W", 0, 0, 1, 2);
        cg.AddRoad(0, 1, 1, 50, RoadType.Arterial, 0, true);
        cg.AddRoad(1, 2, 1, 50, RoadType.Arterial, 0, true);
        cg.AddRoad(0, 3, 2.5, 100, RoadType.Highway, 0, true);
        cg.AddRoad(3, 2, 2.5, 100, RoadType.Highway, 0, true);
        Check("free-flow: short route is fastest", Dij.FindPath(cg, 0, 2, CostModel.FastestTime).Nodes.Contains(1));
        cg.UpdateCongestion(1, 2, 0.95);
        var jam = Dij.FindPath(cg, 0, 2, CostModel.FastestTime);
        Check("live congestion update re-routes to highway", jam.Nodes.Contains(3));
        var snap = Dij.FindPath(cg, 0, 2, CostModel.ShortestDistance);
        double t0 = snap.TotalTimeMin; cg.UpdateCongestion(0, 1, 0.9);
        Check("returned route's time is a snapshot (not re-priced by later updates)", snap.TotalTimeMin == t0);
        cg.UpdateCongestion(0, 1, 0);
        Check("distance metric ignores congestion", Dij.FindPath(cg, 0, 2, CostModel.ShortestDistance).Nodes.Contains(1));
        Check("weighted α=1 equals distance", Math.Abs(Dij.FindPath(cg, 0, 2, new CostModel(Metric.Weighted, 1)).TotalCost - 2) < 1e-9);
        Check("weighted α=0 equals time", Math.Abs(Dij.FindPath(cg, 0, 2, new CostModel(Metric.Weighted, 0)).TotalCost - jam.TotalCost) < 1e-9);
        Check("avoid-congestion constraint bypasses jammed road",
            Dij.FindPath(cg, 0, 2, CostModel.ShortestDistance, new RouteConstraints().AvoidCongestionAbove(0.6)).Nodes.Contains(3));

        Console.WriteLine("== 5. Cross-validation on 2,500-node synthetic city (300 random queries × 3 metrics) ==");
        var city = SyntheticGenerator.SparseCity(2500, seed: 1);
        var rnd = new Random(3);
        int mismatchA = 0, mismatchB = 0, mismatchArr = 0, fewer = 0, found = 0;
        var metrics = new[] { CostModel.ShortestDistance, CostModel.FastestTime, new CostModel(Metric.Weighted, 0.3) };
        for (int q = 0; q < 300; q++)
        {
            int s = rnd.Next(2500), t = rnd.Next(2500);
            var m = metrics[q % 3];
            var d = Dij.FindPath(city, s, t, m);
            var a = Ast.FindPath(city, s, t, m);
            if (d.Found != a.Found || (d.Found && Math.Abs(d.TotalCost - a.TotalCost) > 1e-6)) mismatchA++;
            if (d.Found) { found++; if (a.NodesSettled <= d.NodesSettled) fewer++; }
            if (q % 10 == 0)
            {
                var b = Bf.FindPath(city, s, t, m);
                var ar = Arr.FindPath(city, s, t, m);
                if (d.Found != b.Found || (d.Found && Math.Abs(d.TotalCost - b.TotalCost) > 1e-6)) mismatchB++;
                if (d.Found != ar.Found || (d.Found && Math.Abs(d.TotalCost - ar.TotalCost) > 1e-6)) mismatchArr++;
            }
        }
        Check("A* optimal cost == Dijkstra on all 300 queries", mismatchA == 0, $"({mismatchA} mismatches)");
        Check("Bellman-Ford == Dijkstra (30 queries)", mismatchB == 0);
        Check("Array-Dijkstra == heap Dijkstra (30 queries)", mismatchArr == 0);
        Check("A* never settles more nodes than Dijkstra", fewer == found, $"({fewer}/{found})");

        Console.WriteLine("== 6. Reachability ==");
        var r0 = ReachabilityAnalyzer.WithinBudget(city, 0, 0, CostModel.FastestTime);
        Check("budget 0 → only the source", r0.Count == 1 && r0.CostTo.ContainsKey(0));
        var r5 = ReachabilityAnalyzer.WithinBudget(city, 1200, 5, CostModel.FastestTime);
        var r10 = ReachabilityAnalyzer.WithinBudget(city, 1200, 10, CostModel.FastestTime);
        Check("monotonic: 5-min set ⊆ 10-min set", r5.CostTo.Keys.All(r10.CostTo.ContainsKey) && r10.Count > r5.Count);
        Check("every reported cost ≤ budget", r10.CostTo.Values.All(v => v <= 10 + 1e-9));
        bool agree = r10.CostTo.Take(40).All(kv => Math.Abs(Dij.FindPath(city, 1200, kv.Key, CostModel.FastestTime).TotalCost - kv.Value) < 1e-6);
        Check("bounded-Dijkstra costs == full Dijkstra costs", agree);
        int outside = Enumerable.Range(0, 2500).Where(v => !r10.CostTo.ContainsKey(v)).Take(40)
            .Count(v => { var p = Dij.FindPath(city, 1200, v, CostModel.FastestTime); return p.Found && p.TotalCost <= 10; });
        Check("no node within budget is missed", outside == 0);
        var hops = ReachabilityAnalyzer.WithinHops(city, 1200, 6);
        bool differs = !hops.CostTo.Keys.ToHashSet().SetEquals(ReachabilityAnalyzer.WithinBudget(city, 1200, 4, CostModel.FastestTime).CostTo.Keys);
        Check("BFS-by-hops ≠ time budget (why BFS is wrong for weighted budgets)", differs);

        Console.WriteLine("== 7. Connectivity & bottlenecks ==");
        Check("BFS and DFS agree on connectivity", ConnectivityAnalyzer.IsWeaklyConnected(city) == ConnectivityAnalyzer.IsWeaklyConnected(city, useDfs: true));
        Check("isolated node E → network not connected", !ConnectivityAnalyzer.IsWeaklyConnected(g));
        ConnectivityAnalyzer.ComponentLabels(g, out int comps);
        Check("component count = 2 (A-B-C-D, E)", comps == 2);
        Check("one-way X→Y network: strongly connected via detour", ConnectivityAnalyzer.IsStronglyConnected(ow, out _, out _));
        var path3 = SyntheticGenerator.Chain(3);
        var rep3 = BottleneckAnalyzer.Analyze(path3);
        Check("chain a-b-c: b is articulation point, 2 bridges", rep3.ArticulationPoints.Count == 1 && rep3.ArticulationPoints[0].NodeId == 1 && rep3.Bridges.Count == 2);
        var cyc = new RoadNetwork();
        for (int i = 0; i < 5; i++) cyc.AddNode($"c{i}", 0, 0, i, 0);
        for (int i = 0; i < 5; i++) cyc.AddRoad(i, (i + 1) % 5, 1, 40, RoadType.Secondary, 0, true);
        var repC = BottleneckAnalyzer.Analyze(cyc);
        Check("ring road: no single point of failure", repC.ArticulationPoints.Count == 0 && repC.Bridges.Count == 0);
        Check("Tarjan APs, bridges AND impact scores == brute-force oracle on 40 random graphs", TarjanMatchesBruteForce(40));

        Console.WriteLine("== 8. Scale / stack-safety worst case ==");
        var chain = SyntheticGenerator.Chain(300_000);
        bool okDeep;
        try
        {
            var rep = BottleneckAnalyzer.Analyze(chain, computeImpact: false);
            okDeep = rep.Bridges.Count == 299_999 && ConnectivityAnalyzer.IsWeaklyConnected(chain, useDfs: true);
        }
        catch (Exception) { okDeep = false; }
        Check("300,000-node chain: iterative DFS/Tarjan, no stack overflow", okDeep);
        var far = Dij.FindPath(chain, 0, 299_999, CostModel.ShortestDistance);
        Check("300,000-segment shortest path found", far.Found && far.Nodes.Count == 300_000);

        Console.WriteLine("== 9. Dataset loading ==");
        var colombo = Program.LoadColombo(dataDir, out var osmRep);
        Check($"OSM Colombo loads: {colombo.NodeCount:N0} nodes, {colombo.EdgeCount:N0} directed edges, 0 bad rows",
              colombo.NodeCount == 43_744 && colombo.EdgeCount > 90_000 && osmRep.BadRows == 0);
        Check("OSM: every edge length ≥ straight-line distance (A* admissible on real data)",
              colombo.Nodes.All(n => colombo.OutEdges(n.Id).All(e => e.DistanceKm >= colombo.StraightLineKm(e.From, e.To) - 1e-12)));
        Check("OSM: no self-loops survive cleaning", colombo.Nodes.All(n => colombo.OutEdges(n.Id).All(e => e.To != e.From)));
        var rr = new Random(9); int mm = 0;
        for (int q = 0; q < 60; q++)
        {
            int s = rr.Next(colombo.NodeCount), t = rr.Next(colombo.NodeCount);
            foreach (var m in new[] { CostModel.ShortestDistance, CostModel.FastestTime })
            {
                var x = Dij.FindPath(colombo, s, t, m); var y = Ast.FindPath(colombo, s, t, m);
                if (x.Found != y.Found || (x.Found && Math.Abs(x.TotalCost - y.TotalCost) > 1e-9)) mm++;
            }
        }
        Check("A* == Dijkstra on 120 real Colombo queries", mm == 0, $"({mm})");
        var tmpo = Path.Combine(Path.GetTempPath(), "ur_osm"); Directory.CreateDirectory(tmpo);
        File.WriteAllText(Path.Combine(tmpo, "n.csv"), "osmid,y,x,street_count\n900000000001,6.90,79.85,3\n900000000002,6.901,79.85,3\n900000000003,6.902,79.85,1\n");
        File.WriteAllText(Path.Combine(tmpo, "e.csv"), "u,v,length,highway,oneway,speed_kph,travel_time,name,bridge\n" +
            "900000000001,900000000002,120.5,\"['residential', 'tertiary']\",False,30,1,\"Road, With Comma\",\n" +
            "900000000002,900000000001,120.5,\"['residential', 'tertiary']\",False,30,1,\"Road, With Comma\",\n" +
            "900000000002,900000000003,150,primary,True,50,1,Main,yes\n" +
            "900000000003,900000000003,10,residential,False,30,1,Loop,\n" +
            "900000000002,900000000003,5,residential,True,30,1,TooShort,\n" +
            "900000000001,999,10,residential,False,30,1,Ghost,\n");
        var mini = Data.OsmLoader.Load(Path.Combine(tmpo, "n.csv"), Path.Combine(tmpo, "e.csv"), out var mr);
        bool paired = mini.TryGetEdge(0, 1, out var e01) && mini.TryGetEdge(1, 0, out var e10) && e01.RoadId == e10.RoadId;
        Check("OSM parser: u→v + v→u rows merged into ONE two-way road", paired && mini.RoadCount == 3);
        Check("OSM parser: quoted names with commas and tag lists parsed", e01!.Name == "Road, With Comma" && e01.Type == RoadType.Residential);
        Check("OSM parser: bridge=yes → Bridge, one-way kept one-way", mini.OutEdges(1).Any(e => e.To == 2 && e.Type == RoadType.Bridge) && !mini.TryGetEdge(2, 1, out _));
        Check("OSM parser: self-loop removed, unknown node skipped", mr.SelfLoopsRemoved == 1 && mr.BadRows == 1);
        Check("OSM parser: length shorter than straight line is raised", mr.LengthsRaisedToStraightLine == 1);
        string tmp = Path.Combine(Path.GetTempPath(), "ur_bad");
        Directory.CreateDirectory(tmp);
        File.WriteAllText(Path.Combine(tmp, "n.csv"), "id,name,lat,lon,type\n0,A,6.9,79.8,Junction\n1,B,6.91,79.81,Junction\n");
        File.WriteAllText(Path.Combine(tmp, "e.csv"), "from,to,distance_km,speed_limit_kmh,road_type,congestion,one_way\n0,1,1.2,40,Secondary,0.2,0\n0,7,1,40,Secondary,0,0\n0,1,abc,40,Secondary,0,0\n1,1,1,40,Secondary,0,0\n0,1,-2,40,Secondary,0,0\n0,1,1,40,Spaceway,0,0\n");
        var w2 = new List<string>();
        var bad = DatasetLoader.Load(Path.Combine(tmp, "n.csv"), Path.Combine(tmp, "e.csv"), w2);
        Check("generic CSV: malformed rows skipped with warnings, valid row kept", bad.EdgeCount == 2 && w2.Count == 5);
        Throws<FileNotFoundException>("missing dataset file → clear error", () => DatasetLoader.Load("nope.csv", "nope2.csv"));

        Console.WriteLine($"\nRESULT: {_pass} passed, {_fail} failed");
        foreach (var f in Failures) Console.WriteLine($"  failed: {f}");
        return _fail == 0;
    }

    private static bool TarjanMatchesBruteForce(int trials)
    {
        var rnd = new Random(11);
        for (int t = 0; t < trials; t++)
        {
            int n = rnd.Next(4, 14);
            var g = new RoadNetwork();
            for (int i = 0; i < n; i++) g.AddNode($"r{i}", 0, 0, i, 0);
            int m = rnd.Next(n - 1, n * 2);
            for (int k = 0; k < m; k++) { int a = rnd.Next(n), b = rnd.Next(n); if (a != b) g.AddRoad(a, b, 1, 40, RoadType.Secondary, 0, true); }
            var rep = BottleneckAnalyzer.Analyze(g, computeImpact: false);
            var adj = BottleneckAnalyzer.BuildUndirected(g);
            int baseComps = Components(n, adj, -1, -1);
            var apBrute = Enumerable.Range(0, n).Where(v => Components(n, adj, v, -1) > baseComps).ToHashSet();
            var roads = adj.SelectMany(l => l.Select(x => x.road)).Distinct();
            var brBrute = roads.Where(r => Components(n, adj, -1, r) > baseComps).ToHashSet();
            if (!apBrute.SetEquals(rep.ArticulationPoints.Select(a => a.NodeId))) return false;
            var repI = BottleneckAnalyzer.Analyze(g, computeImpact: true);
            foreach (var ap in repI.ArticulationPoints)
                if (ap.NodesCutOff != CutOffBrute(n, adj, ap.NodeId)) return false;
            if (!brBrute.SetEquals(rep.Bridges.Select(b => b.RoadId))) return false;
        }
        return true;
    }

    /// <summary>Nodes outside the largest remaining piece of v's component when v is removed (oracle for O(V+E) impact).</summary>
    private static int CutOffBrute(int n, List<(int to, int road)>[] adj, int v)
    {
        var comp = new HashSet<int> { v }; var st = new Stack<int>(); st.Push(v);
        while (st.Count > 0) foreach (var (w, _) in adj[st.Pop()]) if (comp.Add(w)) st.Push(w);
        var seen = new HashSet<int> { v }; int largest = 0;
        foreach (int s in comp)
        {
            if (seen.Contains(s)) continue;
            int size = 0; seen.Add(s); st.Push(s);
            while (st.Count > 0) { size++; foreach (var (w, _) in adj[st.Pop()]) if (seen.Add(w)) st.Push(w); }
            largest = Math.Max(largest, size);
        }
        return comp.Count - 1 - largest;
    }

    /// <summary>Components after removing node skipV (not counted) or road skipRoad.</summary>
    private static int Components(int n, List<(int to, int road)>[] adj, int skipV, int skipRoad)
    {
        var seen = new bool[n]; int c = 0;
        for (int s = 0; s < n; s++)
        {
            if (seen[s] || s == skipV) continue;
            c++; var st = new Stack<int>(); st.Push(s); seen[s] = true;
            while (st.Count > 0)
                foreach (var (w, r) in adj[st.Pop()])
                    if (!seen[w] && w != skipV && r != skipRoad) { seen[w] = true; st.Push(w); }
        }
        return c;
    }
}
