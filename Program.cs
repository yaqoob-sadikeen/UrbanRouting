using UrbanRouting.Analysis;
using UrbanRouting.Benchmark;
using UrbanRouting.Data;
using UrbanRouting.Graph;
using UrbanRouting.Models;
using UrbanRouting.Routing;
using UrbanRouting.Tests;

namespace UrbanRouting;

public static class Program
{
    public static int Main(string[] args)
    {
        Console.OutputEncoding = System.Text.Encoding.UTF8;
        string mode = args.Length > 0 ? args[0].ToLowerInvariant() : "demo";
        string dataDir = FindDataDir();
        try
        {
            switch (mode)
            {
                case "demo": Demo(dataDir); return 0;
                case "test": return SelfTests.RunAll(dataDir) ? 0 : 1;
                case "bench":
                    string? only = args.Skip(1).FirstOrDefault(a => !a.StartsWith("--"));
                    Benchmarker.RunAll("results", quick: args.Contains("--quick"), only: only, dataDir: dataDir);
                    return 0;
                case "generate":
                    int n = args.Length > 1 && int.TryParse(args[1], out var v) && v > 0 ? v : 1000;
                    string outDir = args.Length > 2 ? args[2] : "data/generated";
                    Directory.CreateDirectory(outDir);
                    var g = SyntheticGenerator.SparseCity(n, 42);
                    DatasetLoader.Save(g, Path.Combine(outDir, $"city_{n}_nodes.csv"), Path.Combine(outDir, $"city_{n}_edges.csv"));
                    Console.WriteLine($"Generated {g.NodeCount} nodes / {g.EdgeCount} directed edges → {outDir}");
                    return 0;
                case "all":
                    bool ok = SelfTests.RunAll(dataDir);
                    Demo(dataDir);
                    Benchmarker.RunAll("results", dataDir: dataDir);
                    return ok ? 0 : 1;
                default:
                    Console.WriteLine("Usage: dotnet run -c Release -- [demo | test | bench [real|sparse|dense|repr|reach] [--quick] | generate <nodes> [dir] | all]");
                    return 2;
            }
        }
        catch (Exception ex)
        {
            Console.Error.WriteLine($"ERROR: {ex.Message}");
            return 1;
        }
    }

    private static string FindDataDir()
    {
        foreach (var c in new[] { "data", "../data", "../../../data", "../../../../data" })
            if (File.Exists(Path.Combine(c, "colombo_nodes_osm.csv"))) return c;
        return "data";
    }

    public static RoadNetwork LoadColombo(string dataDir, out Data.OsmLoadReport report) =>
        Data.OsmLoader.Load(Path.Combine(dataDir, "colombo_nodes_osm.csv"), Path.Combine(dataDir, "colombo_edges_osm.csv"), out report);

    /// <summary>Approximate landmark coordinates; each is snapped to the nearest junction on a non-residential road.</summary>
    public static readonly (string Name, double Lat, double Lon)[] Landmarks =
    {
        ("Colombo Fort", 6.9344, 79.8428), ("National Hospital", 6.9190, 79.8680), ("Town Hall", 6.9167, 79.8636),
        ("Borella Junction", 6.9147, 79.8778), ("Narahenpita", 6.8950, 79.8770), ("Kalubowila Hospital", 6.8670, 79.8770),
        ("Mount Lavinia", 6.8390, 79.8630), ("Moratuwa", 6.7730, 79.8820), ("Malabe", 6.9040, 79.9580),
        ("Homagama", 6.8440, 80.0020), ("Mattakkuliya", 6.9740, 79.8790)
    };

    private static void H(string t) => Console.WriteLine($"\n━━ {t} ━━");

    private static void Demo(string dataDir)
    {
        var sw = System.Diagnostics.Stopwatch.StartNew();
        var g = LoadColombo(dataDir, out var rep);
        double loadMs = sw.Elapsed.TotalMilliseconds;
        bool OnMainRoad(int v) => g.OutEdges(v).Any(e => e.Type != RoadType.Residential) && g.InEdges(v).Any(e => e.Type != RoadType.Residential);
        var L = Landmarks.ToDictionary(l => l.Name, l => g.FindNearestNode(l.Lat, l.Lon, OnMainRoad));
        IPathFinder dij = new DijkstraRouter(), ast = new AStarRouter();
        string Nm(int v) => L.FirstOrDefault(kv => kv.Value == v).Key ?? $"({g.GetNode(v).Lat:0.0000}, {g.GetNode(v).Lon:0.0000})";
        string Roads(PathResult r, int k = 4) => !r.Found ? "" : "      via " + string.Join(" → ",
            r.Edges.Where(e => e.Name != "").GroupBy(e => e.Name).OrderByDescending(x => x.Sum(e => e.DistanceKm)).Take(k).Select(x => $"{x.Key} ({x.Sum(e => e.DistanceKm):0.0} km)"));
        void Show(string label, PathResult r)
        {
            Console.WriteLine(r.Found
                ? $"  {label,-34} {r.TotalDistanceKm,6:0.00} km {r.TotalTimeMin,6:0.0} min {r.Edges.Count,4} segs  settled {r.NodesSettled,6:N0}  [{r.Algorithm}]"
                : $"  {label,-34} NO ROUTE — {r.Note}");
        }

        H("Network: Colombo District drivable roads (OpenStreetMap via OSMnx)");
        Console.WriteLine($"Loaded in {loadMs:0} ms: {g.NodeCount:N0} junctions, {g.RoadCount:N0} roads, {g.EdgeCount:N0} directed edges, avg out-degree {(double)g.EdgeCount / g.NodeCount:0.00}");
        Console.WriteLine($"Cleaning: {rep.SelfLoopsRemoved} self-loops removed · {rep.ReverseRowsMerged:N0} reverse rows merged into two-way roads · {rep.OneWayRoads:N0} one-way roads · {rep.LengthsRaisedToStraightLine:N0} lengths raised to straight-line · {rep.BadRows} bad rows");
        foreach (var t in Enum.GetValues<RoadType>())
            Console.WriteLine($"  {t,-12} {g.Nodes.Sum(n => g.OutEdges(n.Id).Count(e => e.Type == t)),7:N0} directed edges");
        Console.WriteLine("Landmarks snapped to nearest main-road junction:");
        foreach (var l in Landmarks)
        {
            var n = g.GetNode(L[l.Name]);
            double off = Math.Sqrt(Math.Pow((n.Lat - l.Lat) * 110.574, 2) + Math.Pow((n.Lon - l.Lon) * 110.4, 2)) * 1000;
            Console.WriteLine($"  {l.Name,-20} → node {n.Id,5} ({off:0} m away)");
        }

        H("1. Optimal path: Moratuwa → National Hospital");
        foreach (var m in new[] { CostModel.ShortestDistance, CostModel.FastestTime, new CostModel(Metric.Weighted, 0.5) })
        {
            var d = dij.FindPath(g, L["Moratuwa"], L["National Hospital"], m);
            var a = ast.FindPath(g, L["Moratuwa"], L["National Hospital"], m);
            Console.WriteLine($"Metric = {m}");
            Show("Dijkstra", d); Show("A*", a);
            Console.WriteLine($"  same cost: {Math.Abs(d.TotalCost - a.TotalCost) < 1e-9}; A* settled {100.0 * a.NodesSettled / d.NodesSettled:0}% of Dijkstra's nodes");
            if (m.Metric == Metric.Time) Console.WriteLine(Roads(d));
        }

        H("2. Constrained routing");
        int fort = L["Colombo Fort"], nara = L["Narahenpita"], mal = L["Malabe"], bor = L["Borella Junction"];
        foreach (var (s, t) in new[] { (fort, nara), (fort, mal), (L["Mount Lavinia"], L["Kalubowila Hospital"]) })
        {
            var car = dij.FindPath(g, s, t, CostModel.FastestTime);
            var hv = dij.FindPath(g, s, t, CostModel.FastestTime, RouteConstraints.HeavyVehicle());
            Console.WriteLine($"{Nm(s)} → {Nm(t)}");
            Show("car", car);
            Console.WriteLine($"    residential share of car route: {car.Edges.Where(e => e.Type == RoadType.Residential).Sum(e => e.DistanceKm) / Math.Max(1e-9, car.TotalDistanceKm):P0}");
            Show("heavy vehicle (no residential)", hv);
        }
        var bn = g.GetNode(bor);
        var zone = new RouteConstraints();
        for (int v = 0; v < g.NodeCount; v++) if (g.StraightLineKm(v, bor) <= 0.4) zone.BlockNode(v);
        Console.WriteLine($"Incident: {zone.BlockedNodes.Count} junctions within 400 m of Borella Junction closed");
        Show("Fort → Malabe, normal", dij.FindPath(g, fort, mal, CostModel.FastestTime));
        Show("Fort → Malabe, Borella zone closed", dij.FindPath(g, fort, mal, CostModel.FastestTime, zone));
        Console.WriteLine("Edge case — avoid every road with congestion > 0.5 (all arterials), but Fort itself sits on an arterial:");
        Show("Fort → Malabe", dij.FindPath(g, fort, mal, CostModel.FastestTime, new RouteConstraints().AvoidCongestionAbove(0.5)));

        H("3. Live congestion: every 'Galle Road' segment jams to 0.95");
        int mor = L["Moratuwa"];
        var before = dij.FindPath(g, mor, fort, CostModel.FastestTime);
        var galle = g.Nodes.SelectMany(n => g.OutEdges(n.Id)).Where(e => e.Name.Equals("Galle Road", StringComparison.OrdinalIgnoreCase)).ToList();
        var saved = galle.Select(e => e.Congestion).ToList();
        foreach (var e in galle) e.Congestion = 0.95;
        var stale = before; // route computed before the jam keeps its original price (snapshot)
        var after = dij.FindPath(g, mor, fort, CostModel.FastestTime);
        double galleKmBefore = before.Edges.Where(e => e.Name == "Galle Road").Sum(e => e.DistanceKm);
        double galleKmAfter = after.Edges.Where(e => e.Name == "Galle Road").Sum(e => e.DistanceKm);
        double oldRouteNow = before.Edges.Sum(e => e.TravelTimeMin);
        Console.WriteLine($"  {galle.Count:N0} Galle Road edges updated (O(1) each)");
        Show("before jam", before); Console.WriteLine($"    Galle Road km on route: {galleKmBefore:0.0}");
        Console.WriteLine($"  old route if still driven now: {oldRouteNow:0.0} min");
        Show("after jam (re-routed)", after); Console.WriteLine($"    Galle Road km on route: {galleKmAfter:0.0}");
        Console.WriteLine(Roads(after));
        for (int i = 0; i < galle.Count; i++) galle[i].Congestion = saved[i];
        Directory.CreateDirectory("results");
        using (var w = new StreamWriter("results/demo_routes.csv"))
        {
            w.WriteLine("route,seq,lat,lon");
            foreach (var (label, r) in new[] { ("before_jam", before), ("after_jam", after) })
                for (int i = 0; i < r.Nodes.Count; i++)
                    w.WriteLine(FormattableString.Invariant($"{label},{i},{g.GetNode(r.Nodes[i]).Lat},{g.GetNode(r.Nodes[i]).Lon}"));
        }

        H("4. Reachability: emergency depot at Town Hall (time budget, peak congestion)");
        foreach (double b in new double[] { 3, 5, 8, 12, 20 })
        {
            var r = ReachabilityAnalyzer.WithinBudget(g, L["Town Hall"], b, CostModel.FastestTime);
            var hosp = new[] { "National Hospital", "Kalubowila Hospital" }.Where(h => r.CostTo.ContainsKey(L[h])).Select(h => $"{h} {r.CostTo[L[h]]:0.0}");
            Console.WriteLine($"  ≤{b,2} min: {r.Count,6:N0} junctions ({100.0 * r.Count / g.NodeCount,4:0.0}% of district)  {string.Join(", ", hosp)}");
        }
        var t8 = ReachabilityAnalyzer.WithinBudget(g, L["Town Hall"], 8, CostModel.FastestTime);
        int h = 1; ReachResult hop = ReachabilityAnalyzer.WithinHops(g, L["Town Hall"], 1);
        while (hop.Count < t8.Count) hop = ReachabilityAnalyzer.WithinHops(g, L["Town Hall"], ++h);   // BFS set of similar size
        int both = hop.CostTo.Keys.Count(t8.CostTo.ContainsKey);
        Console.WriteLine($"  BFS ≤{h} hops gives {hop.Count:N0} junctions (similar size to the 8-min set), but only {100.0 * both / t8.Count:0}% of them are truly ≤8 min → hops ≠ minutes");
        var late = hop.CostTo.Keys.Where(v => !t8.CostTo.ContainsKey(v)).Select(v => dij.FindPath(g, L["Town Hall"], v, CostModel.FastestTime).TotalCost).DefaultIfEmpty(0).Max();
        Console.WriteLine($"  worst BFS-included junction actually needs {late:0.0} min");

        H("5. Connectivity");
        ConnectivityAnalyzer.ComponentLabels(g, out int comps);
        Console.WriteLine($"  Weakly connected: {ConnectivityAnalyzer.IsWeaklyConnected(g)} (BFS) / {ConnectivityAnalyzer.IsWeaklyConnected(g, true)} (DFS), {comps} component(s)");
        bool strong = ConnectivityAnalyzer.IsStronglyConnected(g, out var unreachable, out var trapped);
        Console.WriteLine($"  Strongly connected: {strong} — {unreachable.Count} junctions cannot be reached from the rest, {trapped.Count} cannot get back out");
        if (unreachable.Count > 0)
        {
            var u = g.GetNode(unreachable[0]);
            Console.WriteLine($"  e.g. junction at ({u.Lat:0.0000}, {u.Lon:0.0000}): in-roads {g.InEdges(u.Id).Count}, out-roads {g.OutEdges(u.Id).Count} — one-way streets clipped at the district boundary");
        }

        H("6. Critical infrastructure (Tarjan)");
        sw.Restart();
        var bot = BottleneckAnalyzer.Analyze(g);
        Console.WriteLine($"  {bot.ArticulationPoints.Count:N0} articulation points, {bot.Bridges.Count:N0} bridges in {sw.ElapsedMilliseconds} ms");
        Console.WriteLine($"  cutting off ≥1 node: all of them; ≥10 nodes: {bot.ArticulationPoints.Count(a => a.NodesCutOff >= 10)} APs, {bot.Bridges.Count(b => b.NodesCutOff >= 10)} bridges; ≥50: {bot.ArticulationPoints.Count(a => a.NodesCutOff >= 50)} APs, {bot.Bridges.Count(b => b.NodesCutOff >= 50)} bridges");
        Console.WriteLine("  Top critical roads (bridges) by junctions isolated:");
        foreach (var br in bot.Bridges.Take(6))
        {
            g.TryGetEdge(br.A, br.B, out var e);
            var n = g.GetNode(br.A);
            Console.WriteLine($"    • {(e?.Name is { Length: > 0 } nm ? nm : "(unnamed)"),-28} {e?.Type,-11} at ({n.Lat:0.0000}, {n.Lon:0.0000}) isolates {br.NodesCutOff}");
        }
        using (var w = new StreamWriter("results/critical_roads.csv"))
        {
            w.WriteLine("rank,name,lat_a,lon_a,lat_b,lon_b,cut_off");
            int rank = 0;
            foreach (var br in bot.Bridges.Take(20))
            {
                g.TryGetEdge(br.A, br.B, out var e); var a = g.GetNode(br.A); var b = g.GetNode(br.B);
                w.WriteLine(FormattableString.Invariant($"{++rank},\"{e?.Name}\",{a.Lat},{a.Lon},{b.Lat},{b.Lon},{br.NodesCutOff}"));
            }
        }
        using (var w = new StreamWriter("results/critical_zones.csv"))
        {
            // the junctions each of the top-5 bridges would isolate (BFS on the side that is cut off)
            w.WriteLine("rank,lat,lon");
            int rank = 0;
            foreach (var br in bot.Bridges.Take(5))
            {
                rank++;
                var c = new RouteConstraints().BlockRoad(br.RoadId);
                var sideA = ReachabilityAnalyzer.WithinHops(g, br.A, int.MaxValue, c);
                var zoneSet = sideA.Count <= g.NodeCount / 2 ? sideA : ReachabilityAnalyzer.WithinHops(g, br.B, int.MaxValue, c);
                foreach (int v in zoneSet.CostTo.Keys)
                    w.WriteLine(FormattableString.Invariant($"{rank},{g.GetNode(v).Lat},{g.GetNode(v).Lon}"));
            }
        }
        var top = bot.Bridges.First(b => { g.TryGetEdge(b.A, b.B, out var e); return e!.Name.Contains("Mattakkuliya"); });
        var close = new RouteConstraints().BlockRoad(top.RoadId);
        int side = dij.FindPath(g, fort, top.A, CostModel.FastestTime, close).Found ? top.B : top.A;
        Console.WriteLine("  Proof — close that Mattakkuliya road, then route Fort → the cut-off side:");
        Show("  open", dij.FindPath(g, fort, side, CostModel.FastestTime));
        Show("  closed", dij.FindPath(g, fort, side, CostModel.FastestTime, close));

        H("7. A* vs Dijkstra on 300 random origin–destination pairs");
        var rnd = new Random(2026);
        foreach (var m in new[] { CostModel.ShortestDistance, CostModel.FastestTime })
        {
            long sd = 0, sa = 0; int mism = 0, found = 0; double td = 0, ta = 0;
            for (int q = 0; q < 300; q++)
            {
                int s = rnd.Next(g.NodeCount), t = rnd.Next(g.NodeCount);
                var t0 = System.Diagnostics.Stopwatch.StartNew(); var x = dij.FindPath(g, s, t, m); td += t0.Elapsed.TotalMilliseconds;
                t0.Restart(); var y = ast.FindPath(g, s, t, m); ta += t0.Elapsed.TotalMilliseconds;
                if (x.Found != y.Found || (x.Found && Math.Abs(x.TotalCost - y.TotalCost) > 1e-9)) mism++;
                if (x.Found) { found++; sd += x.NodesSettled; sa += y.NodesSettled; }
            }
            Console.WriteLine($"  {m,-9}: {found} routable, cost mismatches {mism}; A* settles {100.0 * sa / sd:0}% of Dijkstra's nodes; mean time Dijkstra {td / 300:0.00} ms vs A* {ta / 300:0.00} ms");
        }
    }
}
