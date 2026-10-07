using UrbanRouting.Graph;

namespace UrbanRouting.Routing;

public interface IPathFinder
{
    string Name { get; }
    PathResult FindPath(RoadNetwork g, int source, int target, CostModel cost, RouteConstraints? constraints = null);
}

/// <summary>Shared input validation — every worst case is rejected or answered before the search starts.</summary>
public abstract class PathFinderBase : IPathFinder
{
    public abstract string Name { get; }

    public PathResult FindPath(RoadNetwork g, int source, int target, CostModel cost, RouteConstraints? constraints = null)
    {
        ArgumentNullException.ThrowIfNull(g);
        ArgumentNullException.ThrowIfNull(cost);
        g.ValidateNode(source);
        g.ValidateNode(target);
        constraints ??= RouteConstraints.None;
        if (!constraints.IsNodeAllowed(source)) return PathResult.NotFound(Name, 0, 0, "source is blocked");
        if (!constraints.IsNodeAllowed(target)) return PathResult.NotFound(Name, 0, 0, "destination is blocked");
        if (source == target)
            return new PathResult { Found = true, Nodes = new[] { source }, TotalCost = 0, NodesSettled = 1, Algorithm = Name };
        return Search(g, source, target, cost, constraints);
    }

    protected abstract PathResult Search(RoadNetwork g, int s, int t, CostModel cost, RouteConstraints c);

    protected static double CheckedCost(CostModel cost, Models.Edge e)
    {
        double w = cost.Cost(e);
        if (w < 0 || double.IsNaN(w)) throw new InvalidOperationException($"Negative/NaN cost on {e}; Dijkstra/A* invalid.");
        return w;
    }
}
