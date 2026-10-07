using UrbanRouting.Graph;
using UrbanRouting.Models;

namespace UrbanRouting.Routing;

/// <summary>
/// Dijkstra WITHOUT a heap: linear scan for the minimum unsettled node.
///   Time O(V² + E), Space O(V). Worse on sparse road graphs, but on a DENSE transit network
///   (E ≈ V²) it matches/beats the heap version because it avoids E·log V heap pushes.
/// </summary>
public sealed class ArrayDijkstraRouter : PathFinderBase
{
    public override string Name => "Dijkstra (array, O(V²))";

    protected override PathResult Search(RoadNetwork g, int s, int t, CostModel cost, RouteConstraints c)
    {
        int n = g.NodeCount;
        var dist = new double[n];
        Array.Fill(dist, double.PositiveInfinity);
        var prev = new Edge?[n];
        var settled = new bool[n];
        dist[s] = 0;
        int settledCount = 0; long relaxed = 0;

        for (int iter = 0; iter < n; iter++)
        {
            int u = -1; double best = double.PositiveInfinity;
            for (int i = 0; i < n; i++)
                if (!settled[i] && dist[i] < best) { best = dist[i]; u = i; }
            if (u == -1) break; // remaining nodes unreachable
            settled[u] = true; settledCount++;
            if (u == t) return PathResult.Build(Name, s, t, prev, best, settledCount, relaxed);
            foreach (var e in g.OutEdges(u))
            {
                if (settled[e.To] || !c.IsEdgeAllowed(e)) continue;
                relaxed++;
                double nd = best + CheckedCost(cost, e);
                if (nd < dist[e.To]) { dist[e.To] = nd; prev[e.To] = e; }
            }
        }
        return PathResult.NotFound(Name, settledCount, relaxed, "destination unreachable under current constraints");
    }
}
