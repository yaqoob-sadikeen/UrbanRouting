using UrbanRouting.Graph;

namespace UrbanRouting.Analysis;

public sealed record CriticalNode(int NodeId, int NodesCutOff);
public sealed record CriticalRoad(int RoadId, int A, int B, int NodesCutOff);

public sealed record BottleneckReport(IReadOnlyList<CriticalNode> ArticulationPoints, IReadOnlyList<CriticalRoad> Bridges);

/// <summary>
/// Critical-infrastructure detection with TARJAN'S LOW-LINK algorithm (one DFS, O(V+E)).
///   disc[v] = DFS discovery time; low[v] = earliest discovery time reachable from v's subtree using ≤1 back edge.
///   • Road (p,v) is a BRIDGE           iff low[v] >  disc[p]   (subtree of v has no other way out)
///   • Node p is an ARTICULATION POINT  iff low[v] ≥ disc[p] for some child v (non-root), or root with ≥2 children
/// Implemented ITERATIVELY (explicit stack) so large networks cannot overflow the call stack.
/// Works on the undirected road multigraph: parent is skipped by ROAD ID, not by node, so two parallel roads
/// between the same junctions are correctly NOT reported as a bridge.
/// Impact of each critical node/road is derived from the same DFS counters (subtree sizes) → whole analysis O(V+E).
/// Rejected alternative: brute force (remove each node/edge, re-check connectivity) = O(V·(V+E)) — used only in tests
/// as an oracle to verify Tarjan.
/// </summary>
public static class BottleneckAnalyzer
{
    public static BottleneckReport Analyze(RoadNetwork g, bool computeImpact = true)
    {
        int n = g.NodeCount;
        var adj = BuildUndirected(g);
        var disc = new int[n]; Array.Fill(disc, -1);
        var low = new int[n];
        var parentRoad = new int[n]; Array.Fill(parentRoad, -1);
        var parentNode = new int[n]; Array.Fill(parentNode, -1);
        var iter = new int[n];
        var subtree = new int[n];
        var isAp = new bool[n];
        var rootOf = new int[n];
        var cutSum = new int[n];   // total size of subtrees that would be severed from p if p failed
        var cutMax = new int[n];   // largest such subtree
        var bridges = new List<(int road, int p, int v)>();
        int timer = 0;
        var stack = new Stack<int>();

        for (int root = 0; root < n; root++)
        {
            if (disc[root] != -1) continue;
            int rootChildren = 0;
            disc[root] = low[root] = timer++;
            rootOf[root] = root;
            stack.Push(root);
            while (stack.Count > 0)
            {
                int v = stack.Peek();
                if (iter[v] < adj[v].Count)
                {
                    var (w, road) = adj[v][iter[v]++];
                    if (road == parentRoad[v]) continue;          // don't reuse the tree edge we came in on
                    if (disc[w] == -1)
                    {
                        parentRoad[w] = road; parentNode[w] = v; rootOf[w] = root;
                        disc[w] = low[w] = timer++;
                        if (v == root) rootChildren++;
                        stack.Push(w);
                    }
                    else low[v] = Math.Min(low[v], disc[w]);      // back edge
                }
                else
                {
                    stack.Pop();
                    subtree[v]++;                                   // count v itself
                    int p = parentNode[v];
                    if (p == -1) continue;
                    subtree[p] += subtree[v];
                    low[p] = Math.Min(low[p], low[v]);
                    if (low[v] > disc[p]) bridges.Add((parentRoad[v], p, v));
                    if (p == root || low[v] >= disc[p])
                    {
                        if (p != root) isAp[p] = true;
                        cutSum[p] += subtree[v]; cutMax[p] = Math.Max(cutMax[p], subtree[v]);
                    }
                }
            }
            if (rootChildren > 1) isAp[root] = true;
        }

        // Impact in O(1) per node from the DFS counters: if p fails, its component (size C) splits into the severed
        // child subtrees plus the "rest" (C - 1 - cutSum). Nodes cut off = everything outside the largest piece.
        var aps = new List<CriticalNode>();
        for (int v = 0; v < n; v++)
        {
            if (!isAp[v]) continue;
            int comp = subtree[rootOf[v]];
            int rest = comp - 1 - cutSum[v];
            int largest = Math.Max(cutMax[v], rest);
            aps.Add(new CriticalNode(v, computeImpact ? comp - 1 - largest : -1));
        }
        // cut-off = smaller side of the split component
        var brs = bridges.Select(b => new CriticalRoad(b.road, b.p, b.v,
            Math.Min(subtree[b.v], subtree[rootOf[b.v]] - subtree[b.v]))).ToList();
        return new BottleneckReport(
            aps.OrderByDescending(a => a.NodesCutOff).ToList(),
            brs.OrderByDescending(b => b.NodesCutOff).ToList());
    }

    /// <summary>Undirected multigraph: one entry per physical road (RoadId), both endpoints. O(V+E).</summary>
    internal static List<(int to, int road)>[] BuildUndirected(RoadNetwork g)
    {
        int n = g.NodeCount;
        var adj = new List<(int, int)>[n];
        for (int i = 0; i < n; i++) adj[i] = new List<(int, int)>();
        var seenRoad = new HashSet<int>();
        for (int u = 0; u < n; u++)
            foreach (var e in g.OutEdges(u))
                if (seenRoad.Add(e.RoadId)) { adj[e.From].Add((e.To, e.RoadId)); adj[e.To].Add((e.From, e.RoadId)); }
        return adj;
    }
}
