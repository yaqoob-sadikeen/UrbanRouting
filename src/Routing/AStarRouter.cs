using UrbanRouting.Graph;
using UrbanRouting.Models;

namespace UrbanRouting.Routing;

/// <summary>
/// A* = Dijkstra ordered by f(v) = g(v) + h(v), where h is the straight-line lower bound from CostModel.
/// Because h is admissible AND consistent, the first time a node is popped its g is optimal, so the
/// same settled[] logic as Dijkstra is correct. Worst case equals Dijkstra: O((V+E) log V), O(V+E) space;
/// in practice it settles far fewer nodes because the search is pulled toward the destination.
/// </summary>
public sealed class AStarRouter : PathFinderBase
{
    public override string Name => "A* (straight-line heuristic)";

    protected override PathResult Search(RoadNetwork g, int s, int t, CostModel cost, RouteConstraints c)
    {
        int n = g.NodeCount;
        var gScore = new double[n];
        Array.Fill(gScore, double.PositiveInfinity);
        var prev = new Edge?[n];
        var settled = new bool[n];
        var pq = new PriorityQueue<int, double>();
        gScore[s] = 0;
        pq.Enqueue(s, cost.Heuristic(g, s, t));
        int settledCount = 0; long relaxed = 0;

        while (pq.TryDequeue(out int u, out _))
        {
            if (settled[u]) continue;
            settled[u] = true; settledCount++;
            if (u == t) return PathResult.Build(Name, s, t, prev, gScore[t], settledCount, relaxed);

            double gu = gScore[u];
            foreach (var e in g.OutEdges(u))
            {
                int v = e.To;
                if (settled[v] || !c.IsEdgeAllowed(e)) continue;
                relaxed++;
                double ng = gu + CheckedCost(cost, e);
                if (ng < gScore[v])
                {
                    gScore[v] = ng; prev[v] = e;
                    pq.Enqueue(v, ng + cost.Heuristic(g, v, t));
                }
            }
        }
        return PathResult.NotFound(Name, settledCount, relaxed, "destination unreachable under current constraints");
    }
}
