using UrbanRouting.Graph;
using UrbanRouting.Models;

namespace UrbanRouting.Routing;

public sealed class PathResult
{
    public bool Found { get; init; }
    public IReadOnlyList<int> Nodes { get; init; } = Array.Empty<int>();
    public IReadOnlyList<Edge> Edges { get; init; } = Array.Empty<Edge>();
    public double TotalCost { get; init; } = double.PositiveInfinity;
    /// <summary>Snapshotted when the route is built: a later live-congestion update must not
    /// silently re-price a route that was already returned to a user.</summary>
    public double TotalDistanceKm { get; init; }
    public double TotalTimeMin { get; init; }
    /// <summary>Nodes permanently settled — the key empirical measure of search effort.</summary>
    public int NodesSettled { get; init; }
    public long EdgesRelaxed { get; init; }
    public string Algorithm { get; init; } = "";
    public string? Note { get; init; }

    public static PathResult NotFound(string algo, int settled, long relaxed, string note) =>
        new() { Found = false, Algorithm = algo, NodesSettled = settled, EdgesRelaxed = relaxed, Note = note };

    /// <summary>Rebuilds the path by walking predecessor edges back from the target. O(path length).</summary>
    public static PathResult Build(string algo, int source, int target, Edge?[] prevEdge, double cost, int settled, long relaxed)
    {
        var edges = new List<Edge>();
        int cur = target;
        while (cur != source)
        {
            var e = prevEdge[cur] ?? throw new InvalidOperationException("Broken predecessor chain.");
            edges.Add(e);
            cur = e.From;
        }
        edges.Reverse();
        var nodes = new List<int>(edges.Count + 1) { source };
        nodes.AddRange(edges.Select(e => e.To));
        return new PathResult
        {
            Found = true, Nodes = nodes, Edges = edges, TotalCost = cost, NodesSettled = settled, EdgesRelaxed = relaxed, Algorithm = algo,
            TotalDistanceKm = edges.Sum(e => e.DistanceKm), TotalTimeMin = edges.Sum(e => e.TravelTimeMin)
        };
    }

    public string Describe(RoadNetwork g)
    {
        if (!Found) return $"[{Algorithm}] NO ROUTE — {Note}";
        return $"[{Algorithm}] {string.Join(" → ", Nodes.Select(n => g.GetNode(n).Name))}\n" +
               $"      {TotalDistanceKm:0.00} km, {TotalTimeMin:0.0} min, {Edges.Count} segments, settled {NodesSettled} nodes";
    }
}
