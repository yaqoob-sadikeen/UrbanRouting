using System.Diagnostics;
using System.Globalization;
using UrbanRouting.Analysis;
using UrbanRouting.Data;
using UrbanRouting.Graph;
using UrbanRouting.Routing;

namespace UrbanRouting.Benchmark;

/// <summary>
/// Empirical evaluation. All graphs are seeded → results are reproducible.
/// Each timing = median of several runs after a JIT warm-up. Writes CSVs into results/.
/// </summary>
public static class Benchmarker
{
    private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

    /// <param name="only">null = all sections, else one of: sparse, dense, repr, reach</param>
    public static void RunAll(string outDir, bool quick = false, string? only = null, string dataDir = "data")
    {
        Directory.CreateDirectory(outDir);
        Warmup();
        if (only is null or "real") RealScaling(outDir, dataDir);
        if (only is null or "sparse") ScalingSparse(outDir, quick);
        if (only is null or "dense") DenseVsSparse(outDir, quick);
        if (only is null or "repr") Representation(outDir, quick, dataDir);
        if (only is null or "reach") ReachabilityBudgets(outDir, dataDir);
        Console.WriteLine($"\nCSV results written to {Path.GetFullPath(outDir)}");
    }

    private static void Warmup()
    {
        var g = SyntheticGenerator.SparseCity(2000, 99);
        for (int i = 0; i < 30; i++)
        {
            new DijkstraRouter().FindPath(g, i, 1999 - i, CostModel.FastestTime);
            new AStarRouter().FindPath(g, i, 1999 - i, CostModel.FastestTime);
        }
        BottleneckAnalyzer.Analyze(g, false);
    }

    private static double MedianMs(Action a, int reps)
    {
        var t = new double[reps];
        for (int i = 0; i < reps; i++) { var sw = Stopwatch.StartNew(); a(); t[i] = sw.Elapsed.TotalMilliseconds; }
        Array.Sort(t); return t[reps / 2];
    }

    private static long MeasureBytes(Func<object> build, out object built)
    {
        GC.Collect(); GC.WaitForPendingFinalizers(); GC.Collect();
        long before = GC.GetTotalMemory(true);
        built = build();
        long after = GC.GetTotalMemory(true);
        return after - before;
    }

    /// <summary>Nested real sub-networks: the k junctions closest (in hops) to Colombo Fort, k = 1k … all 43,744.</summary>
    public static List<int> BallAround(RoadNetwork g, int centre, int k)
    {
        var seen = new bool[g.NodeCount]; var order = new List<int>(k); var q = new Queue<int>();
        seen[centre] = true; q.Enqueue(centre);
        while (q.Count > 0 && order.Count < k)
        {
            int u = q.Dequeue(); order.Add(u);
            foreach (var e in g.OutEdges(u)) if (!seen[e.To]) { seen[e.To] = true; q.Enqueue(e.To); }
            foreach (var e in g.InEdges(u)) if (!seen[e.From]) { seen[e.From] = true; q.Enqueue(e.From); }
        }
        return order;
    }

    private static void RealScaling(string dir, string dataDir)
    {
        var full = Program.LoadColombo(dataDir, out _);
        int fort = full.FindNearestNode(6.9344, 79.8428);
        using var w = new StreamWriter(Path.Combine(dir, "scaling_real.csv"));
        w.WriteLine("nodes,edges,memory_mb,dijkstra_dist_ms,astar_dist_ms,dijkstra_time_ms,astar_time_ms,bellmanford_ms,dijkstra_settled,astar_settled,bfs_conn_ms,dfs_conn_ms,tarjan_ms,articulation_points,bridges");
        Console.WriteLine("\n[0] REAL Colombo sub-networks grown from Colombo Fort");
        Console.WriteLine($"{"V",8} {"E",8} {"Dij-d",8} {"A*-d",8} {"Dij-t",8} {"A*-t",8} {"BF",8} {"set D",7} {"set A*",7} {"Tarjan",8} {"MB",6}");
        var dij = new DijkstraRouter(); var ast = new AStarRouter(); var bf = new BellmanFordRouter();
        foreach (int k in new[] { 1_000, 2_500, 5_000, 10_000, 20_000, full.NodeCount })
        {
            var ball = BallAround(full, fort, k);
            long bytes = MeasureBytes(() => full.Subgraph(ball), out var o);
            var g = (RoadNetwork)o; int n = g.NodeCount;
            var rnd = new Random(7); int Q = 25;
            var pairs = Enumerable.Range(0, Q).Select(_ => (rnd.Next(n), rnd.Next(n))).ToArray();
            double dd = 0, ad = 0, dt = 0, at = 0, b = 0; long ds = 0, as_ = 0; int bq = 0;
            foreach (var (s, t) in pairs)
            {
                PathResult? x = null, y = null;
                dd += MedianMs(() => x = dij.FindPath(g, s, t, CostModel.ShortestDistance), 3);
                ad += MedianMs(() => y = ast.FindPath(g, s, t, CostModel.ShortestDistance), 3);
                ds += x!.NodesSettled; as_ += y!.NodesSettled;
                dt += MedianMs(() => dij.FindPath(g, s, t, CostModel.FastestTime), 3);
                at += MedianMs(() => ast.FindPath(g, s, t, CostModel.FastestTime), 3);
                if (n <= 20_000 && bq < 3) { b += MedianMs(() => bf.FindPath(g, s, t, CostModel.ShortestDistance), 1); bq++; }
            }
            double bfsMs = MedianMs(() => ConnectivityAnalyzer.BfsUndirected(g, 0, new bool[n]), 5);
            double dfsMs = MedianMs(() => ConnectivityAnalyzer.DfsUndirected(g, 0, new bool[n]), 5);
            BottleneckReport? rep = null;
            double tj = MedianMs(() => rep = BottleneckAnalyzer.Analyze(g), 3);
            string bcol = bq > 0 ? (b / bq).ToString("0.###", Inv) : "";
            w.WriteLine(string.Join(',', n, g.EdgeCount, (bytes / 1048576.0).ToString("0.###", Inv),
                (dd / Q).ToString("0.####", Inv), (ad / Q).ToString("0.####", Inv), (dt / Q).ToString("0.####", Inv), (at / Q).ToString("0.####", Inv),
                bcol, ds / Q, as_ / Q, bfsMs.ToString("0.###", Inv), dfsMs.ToString("0.###", Inv), tj.ToString("0.###", Inv),
                rep!.ArticulationPoints.Count, rep.Bridges.Count));
            Console.WriteLine($"{n,8} {g.EdgeCount,8} {dd / Q,8:0.00} {ad / Q,8:0.00} {dt / Q,8:0.00} {at / Q,8:0.00} {(bq > 0 ? (b / bq).ToString("0.0") : "-"),8} {ds / Q,7} {as_ / Q,7} {tj,8:0.0} {bytes / 1048576.0,6:0.0}");
        }
    }

    // ---- 1. How every operation scales with V on realistic sparse road networks ----
    private static void ScalingSparse(string dir, bool quick)
    {
        int[] sizes = quick ? new[] { 1_000, 5_000, 20_000 } : new[] { 1_000, 2_500, 5_000, 10_000, 25_000, 50_000, 100_000, 250_000 };
        using var w = new StreamWriter(Path.Combine(dir, "scaling_sparse.csv"));
        w.WriteLine("nodes,edges,build_ms,memory_mb,dijkstra_ms,astar_ms,bellmanford_ms,dijkstra_settled,astar_settled,reach15_ms,reach15_nodes,bfs_conn_ms,dfs_conn_ms,bfs_peak,dfs_peak,tarjan_ms,articulation_points,bridges");
        Console.WriteLine("\n[1] Sparse city scaling");
        Console.WriteLine($"{"V",8} {"E",9} {"Dijk ms",9} {"A* ms",8} {"BF ms",9} {"Dijk set",9} {"A* set",8} {"Tarjan",8} {"MB",7}");
        var dij = new DijkstraRouter(); var ast = new AStarRouter(); var bf = new BellmanFordRouter();
        foreach (int n in sizes)
        {
            var sw = Stopwatch.StartNew();
            long bytes = MeasureBytes(() => SyntheticGenerator.SparseCity(n, 42), out var obj);
            double buildMs = sw.Elapsed.TotalMilliseconds;
            var g = (RoadNetwork)obj;
            var rnd = new Random(5);
            int Q = 15;
            var pairs = Enumerable.Range(0, Q).Select(_ => (rnd.Next(n), rnd.Next(n))).ToArray();
            double dMs = 0, aMs = 0, bMs = 0; long dSet = 0, aSet = 0; int bfq = 0;
            foreach (var (s, t) in pairs)
            {
                PathResult? rd = null, ra = null;
                dMs += MedianMs(() => rd = dij.FindPath(g, s, t, CostModel.FastestTime), 3);
                aMs += MedianMs(() => ra = ast.FindPath(g, s, t, CostModel.FastestTime), 3);
                dSet += rd!.NodesSettled; aSet += ra!.NodesSettled;
                if (n <= 25_000 && bfq < 5) { bMs += MedianMs(() => bf.FindPath(g, s, t, CostModel.FastestTime), 1); bfq++; }
            }
            ReachResult? rr = null;
            double reachMs = MedianMs(() => rr = ReachabilityAnalyzer.WithinBudget(g, n / 2, 15, CostModel.FastestTime), 5);
            TraversalStats bs = default!, ds = default!;
            double bfsMs = MedianMs(() => bs = ConnectivityAnalyzer.BfsUndirected(g, 0, new bool[n]), 3);
            double dfsMs = MedianMs(() => ds = ConnectivityAnalyzer.DfsUndirected(g, 0, new bool[n]), 3);
            BottleneckReport? rep = null;
            double tarjanMs = MedianMs(() => rep = BottleneckAnalyzer.Analyze(g, computeImpact: false), 3);
            string bfCol = bfq > 0 ? (bMs / bfq).ToString("0.###", Inv) : "";
            w.WriteLine(string.Join(',', n, g.EdgeCount, buildMs.ToString("0.##", Inv), (bytes / 1048576.0).ToString("0.###", Inv),
                (dMs / Q).ToString("0.####", Inv), (aMs / Q).ToString("0.####", Inv), bfCol, dSet / Q, aSet / Q,
                reachMs.ToString("0.####", Inv), rr!.Count, bfsMs.ToString("0.###", Inv), dfsMs.ToString("0.###", Inv),
                bs.PeakAuxiliarySize, ds.PeakAuxiliarySize, tarjanMs.ToString("0.###", Inv), rep!.ArticulationPoints.Count, rep.Bridges.Count));
            Console.WriteLine($"{n,8} {g.EdgeCount,9} {dMs / Q,9:0.000} {aMs / Q,8:0.000} {(bfq > 0 ? (bMs / bfq).ToString("0.0") : "-"),9} {dSet / Q,9} {aSet / Q,8} {tarjanMs,8:0.00} {bytes / 1048576.0,7:0.0}");
        }
    }

    // ---- 2. Sparse road graph vs dense transit graph: heap vs array Dijkstra ----
    private static void DenseVsSparse(string dir, bool quick)
    {
        int[] sizes = quick ? new[] { 250, 1000 } : new[] { 250, 500, 1000, 1500, 2000 };
        using var w = new StreamWriter(Path.Combine(dir, "dense_vs_sparse.csv"));
        w.WriteLine("graph,nodes,edges,heap_dijkstra_ms,array_dijkstra_ms");
        Console.WriteLine("\n[2] Heap vs array Dijkstra: sparse road network vs dense transit network (point-to-point, 5 random pairs)");
        var heap = new DijkstraRouter(); var arr = new ArrayDijkstraRouter();
        foreach (int n in sizes)
            foreach (var (name, g) in new[] { ("sparse", SyntheticGenerator.SparseCity(n, 3)), ("dense", SyntheticGenerator.DenseTransit(n, 0.5, 3)) })
            {
                var rnd = new Random(1); double h = 0, a = 0; int Q = 5;
                for (int q = 0; q < Q; q++)
                {
                    int s = rnd.Next(n), t = rnd.Next(n);
                    h += MedianMs(() => heap.FindPath(g, s, t, CostModel.ShortestDistance), 3);
                    a += MedianMs(() => arr.FindPath(g, s, t, CostModel.ShortestDistance), 3);
                }
                w.WriteLine($"{name},{n},{g.EdgeCount},{(h / Q).ToString("0.####", Inv)},{(a / Q).ToString("0.####", Inv)}");
                Console.WriteLine($"  {name,-6} V={n,5} E={g.EdgeCount,9}  heap {h / Q,9:0.000} ms   array {a / Q,9:0.000} ms");
            }
    }

    // ---- 3. Adjacency list vs adjacency matrix ----
    private static void Representation(string dir, bool quick, string dataDir)
    {
        int[] sizes = quick ? new[] { 500, 2000 } : new[] { 500, 1000, 2000, 4000, 8000 };
        using var w = new StreamWriter(Path.Combine(dir, "representation.csv"));
        w.WriteLine("nodes,edges,list_mb,matrix_mb,list_iterate_ms,matrix_iterate_ms,hash_lookup_ns,scan_lookup_ns,matrix_lookup_ns");
        Console.WriteLine("\n[3] Adjacency list vs adjacency matrix (REAL Colombo sub-networks)");
        var full = Program.LoadColombo(dataDir, out _);
        int fort = full.FindNearestNode(6.9344, 79.8428);
        foreach (int n in sizes)
        {
            var ball = BallAround(full, fort, n);
            long listBytes = MeasureBytes(() => full.Subgraph(ball), out var o);
            var g = (RoadNetwork)o;
            long matBytes = MeasureBytes(() => new AdjacencyMatrixNetwork(g), out var mo);
            var m = (AdjacencyMatrixNetwork)mo;
            double listIter = MedianMs(() =>
            {
                double s = 0;
                for (int u = 0; u < g.NodeCount; u++) foreach (var e in g.OutEdges(u)) s += e.DistanceKm;
                if (s < 0) Console.Write("");
            }, 5);
            double matIter = MedianMs(() => { if (m.SumAllNeighbourWeights() < 0) Console.Write(""); }, 3);
            var rnd = new Random(2); const int L = 200_000;
            var qs = Enumerable.Range(0, L).Select(_ => { int u = rnd.Next(n); var oe = g.OutEdges(u); int v = oe.Count > 0 && rnd.Next(2) == 0 ? oe[0].To : rnd.Next(n); return (u, v); }).ToArray();
            int hits = 0;
            double hashMs = MedianMs(() => { foreach (var (u, v) in qs) if (g.TryGetEdge(u, v, out _)) hits++; }, 3);
            double scanMs = MedianMs(() => { foreach (var (u, v) in qs) if (g.FindEdgeByScan(u, v) != null) hits++; }, 3);
            double matMs = MedianMs(() => { foreach (var (u, v) in qs) if (m.HasEdge(u, v)) hits++; }, 3);
            w.WriteLine(string.Join(',', n, g.EdgeCount, (listBytes / 1048576.0).ToString("0.###", Inv), (matBytes / 1048576.0).ToString("0.###", Inv),
                listIter.ToString("0.####", Inv), matIter.ToString("0.####", Inv),
                (hashMs * 1e6 / L).ToString("0.#", Inv), (scanMs * 1e6 / L).ToString("0.#", Inv), (matMs * 1e6 / L).ToString("0.#", Inv)));
            Console.WriteLine($"  V={n,5}: list {listBytes / 1048576.0,7:0.00} MB vs matrix {matBytes / 1048576.0,8:0.00} MB | iterate all neighbours: list {listIter,7:0.000} ms vs matrix {matIter,8:0.000} ms | lookup ns: hash {hashMs * 1e6 / L:0} scan {scanMs * 1e6 / L:0} matrix {matMs * 1e6 / L:0}");
        }
    }

    // ---- 4. Reachability with varying budgets on a 50k-node city ----
    private static void ReachabilityBudgets(string dir, string dataDir)
    {
        var g = Program.LoadColombo(dataDir, out _);
        using var w = new StreamWriter(Path.Combine(dir, "reachability_budget.csv"));
        w.WriteLine("budget_min,nodes_reached,time_ms,peak_heap");
        Console.WriteLine("\n[4] Reachability vs travel-time budget (REAL Colombo, source = Town Hall)");
        int src = g.FindNearestNode(6.9167, 79.8636, v => g.OutEdges(v).Any(e => e.Type != Models.RoadType.Residential));
        foreach (double b in new double[] { 1, 2, 5, 10, 15, 20, 30, 45, 60 })
        {
            ReachResult? r = null;
            double ms = MedianMs(() => r = ReachabilityAnalyzer.WithinBudget(g, src, b, CostModel.FastestTime), 5);
            w.WriteLine($"{b},{r!.Count},{ms.ToString("0.####", Inv)},{r.PeakFrontier}");
            Console.WriteLine($"  budget {b,3} min → {r.Count,6} nodes in {ms,8:0.000} ms");
        }
    }
}
