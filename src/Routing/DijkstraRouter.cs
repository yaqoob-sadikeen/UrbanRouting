using UrbanRouting.Graph;
using UrbanRouting.Models;

namespace UrbanRouting.Routing;

/// <summary>
/// Dijkstra with a BINARY HEAP (System.Collections.Generic.PriorityQueue) and LAZY DELETION.
/// .NET's PriorityQueue has no decrease-key, so an improved distance is pushed again and stale
/// entries are skipped on pop (settled[] check). Heap holds ≤ E entries.
///   Time  O((V + E) log V)     Space O(V + E) worst case (dist/prev arrays + heap)
/// Early exit when the target is settled — its distance is final at that moment.
/// </summary>
public sealed class DijkstraRouter : PathFinderBase
{
    public override string Name => "Dijkstra (binary heap)";

    protected override PathResult Search(RoadNetwork g, int s, int t, CostModel cost, RouteConstraints c)
    {
        int n = g.NodeCount;
        var dist = new double[n];
        Array.Fill(dist, double.PositiveInfinity);
        var prev = new Edge?[n];
        var settled = new bool[n];
        var pq = new PriorityQueue<int, double>();
        dist[s] = 0;
        pq.Enqueue(s, 0);
        int settledCount = 0; long relaxed = 0;

        while (pq.TryDequeue(out int u, out double d))
        {
            if (settled[u]) continue;                 // stale heap entry (lazy deletion)
            settled[u] = true; settledCount++;
            if (u == t) return PathResult.Build(Name, s, t, prev, d, settledCount, relaxed);

            foreach (var e in g.OutEdges(u))
            {
                int v = e.To;
                if (settled[v] || !c.IsEdgeAllowed(e)) continue;
                relaxed++;
                double nd = d + CheckedCost(cost, e);
                if (nd < dist[v]) { dist[v] = nd; prev[v] = e; pq.Enqueue(v, nd); }
            }
        }
        return PathResult.NotFound(Name, settledCount, relaxed, "destination unreachable under current constraints");
    }
}
