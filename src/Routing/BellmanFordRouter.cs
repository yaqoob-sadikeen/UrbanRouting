using UrbanRouting.Graph;
using UrbanRouting.Models;

namespace UrbanRouting.Routing;

/// <summary>
/// REJECTED ALTERNATIVE (benchmarked for comparison). Relaxes every edge up to V-1 times.
///   Time O(V·E) worst case, Space O(V). Only advantage: tolerates negative weights, which road
///   distances/times never have. Includes early termination (stop when a pass changes nothing)
///   and negative-cycle detection for completeness.
/// </summary>
public sealed class BellmanFordRouter : PathFinderBase
{
    public override string Name => "Bellman-Ford";

    protected override PathResult Search(RoadNetwork g, int s, int t, CostModel cost, RouteConstraints c)
    {
        int n = g.NodeCount;
        var dist = new double[n];
        Array.Fill(dist, double.PositiveInfinity);
        var prev = new Edge?[n];
        dist[s] = 0;
        long relaxed = 0;
        bool changed = true;
        for (int pass = 0; pass < n - 1 && changed; pass++)
        {
            changed = false;
            for (int u = 0; u < n; u++)
            {
                if (double.IsPositiveInfinity(dist[u]) || !c.IsNodeAllowed(u)) continue;
                foreach (var e in g.OutEdges(u))
                {
                    if (!c.IsEdgeAllowed(e)) continue;
                    relaxed++;
                    double nd = dist[u] + cost.Cost(e);
                    if (nd < dist[e.To]) { dist[e.To] = nd; prev[e.To] = e; changed = true; }
                }
            }
        }
        if (changed) // a V-th improving pass would mean a negative cycle
            for (int u = 0; u < n; u++)
                if (!double.IsPositiveInfinity(dist[u]))
                    foreach (var e in g.OutEdges(u))
                        if (c.IsEdgeAllowed(e) && dist[u] + cost.Cost(e) < dist[e.To])
                            return PathResult.NotFound(Name, n, relaxed, "negative cycle detected");

        return double.IsPositiveInfinity(dist[t])
            ? PathResult.NotFound(Name, n, relaxed, "destination unreachable under current constraints")
            : PathResult.Build(Name, s, t, prev, dist[t], n, relaxed);
    }
}
