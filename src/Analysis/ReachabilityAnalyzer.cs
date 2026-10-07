using UrbanRouting.Graph;
using UrbanRouting.Routing;

namespace UrbanRouting.Analysis;

public sealed record ReachResult(IReadOnlyDictionary<int, double> CostTo, int PeakFrontier)
{
    public int Count => CostTo.Count;
}

/// <summary>
/// Decision Area 3 — traversal for non-optimal queries.
///  • WithinBudget  : BOUNDED DIJKSTRA. A weighted budget (minutes / km) needs costs in increasing order,
///                    so we stop the moment the popped cost exceeds the budget. Output-sensitive:
///                    O((V'+E') log V') where V',E' = the part of the city inside the budget, not the whole city.
///  • WithinHops    : BFS. Correct only when every edge costs 1 ("within k intersections"). BFS explores by
///                    hop count, NOT by minutes — using it for a time budget gives wrong answers (see tests).
/// </summary>
public static class ReachabilityAnalyzer
{
    public static ReachResult WithinBudget(RoadNetwork g, int source, double budget, CostModel cost, RouteConstraints? c = null)
    {
        g.ValidateNode(source);
        if (budget < 0 || double.IsNaN(budget)) throw new ArgumentOutOfRangeException(nameof(budget), "Budget must be ≥ 0.");
        c ??= RouteConstraints.None;
        var result = new Dictionary<int, double>();
        if (!c.IsNodeAllowed(source)) return new ReachResult(result, 0);

        var best = new Dictionary<int, double> { [source] = 0 };  // sparse map: memory ∝ explored area
        var pq = new PriorityQueue<int, double>();
        pq.Enqueue(source, 0);
        int peak = 1;
        while (pq.TryDequeue(out int u, out double d))
        {
            if (d > budget) break;                    // every remaining entry is ≥ d  → done
            if (result.ContainsKey(u)) continue;      // stale entry
            result[u] = d;
            foreach (var e in g.OutEdges(u))
            {
                if (result.ContainsKey(e.To) || !c.IsEdgeAllowed(e)) continue;
                double nd = d + cost.Cost(e);
                if (nd <= budget && (!best.TryGetValue(e.To, out var old) || nd < old))
                {
                    best[e.To] = nd; pq.Enqueue(e.To, nd);
                }
            }
            peak = Math.Max(peak, pq.Count);
        }
        return new ReachResult(result, peak);
    }

    public static ReachResult WithinHops(RoadNetwork g, int source, int maxHops, RouteConstraints? c = null)
    {
        g.ValidateNode(source);
        if (maxHops < 0) throw new ArgumentOutOfRangeException(nameof(maxHops));
        c ??= RouteConstraints.None;
        var hops = new Dictionary<int, double>();
        if (!c.IsNodeAllowed(source)) return new ReachResult(hops, 0);
        var q = new Queue<int>();
        hops[source] = 0; q.Enqueue(source);
        int peak = 1;
        while (q.Count > 0)
        {
            int u = q.Dequeue();
            int h = (int)hops[u];
            if (h == maxHops) continue;
            foreach (var e in g.OutEdges(u))
                if (!hops.ContainsKey(e.To) && c.IsEdgeAllowed(e)) { hops[e.To] = h + 1; q.Enqueue(e.To); }
            peak = Math.Max(peak, q.Count);
        }
        return new ReachResult(hops, peak);
    }
}
