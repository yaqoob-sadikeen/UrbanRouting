using UrbanRouting.Models;

namespace UrbanRouting.Routing;

/// <summary>
/// Constrained routing is implemented as an EDGE/NODE FILTER applied during relaxation.
/// This keeps every algorithm's complexity unchanged (each check is O(1) with hash sets)
/// and avoids copying the graph per query (which would cost O(V+E) extra time and space).
/// </summary>
public sealed class RouteConstraints
{
    public HashSet<int> BlockedNodes { get; } = new();
    public HashSet<int> BlockedRoads { get; } = new();          // by RoadId (closes both directions)
    public HashSet<long> BlockedDirectedEdges { get; } = new(); // single direction (u,v)
    public HashSet<RoadType> ExcludedRoadTypes { get; } = new();
    /// <summary>Edges with congestion strictly above this are avoided (null = no limit).</summary>
    public double? MaxCongestion { get; set; }

    public static RouteConstraints None => new();

    public bool IsEmpty => BlockedNodes.Count == 0 && BlockedRoads.Count == 0 && BlockedDirectedEdges.Count == 0
                           && ExcludedRoadTypes.Count == 0 && MaxCongestion is null;

    public RouteConstraints BlockNode(int v) { BlockedNodes.Add(v); return this; }
    public RouteConstraints BlockRoad(int roadId) { BlockedRoads.Add(roadId); return this; }
    public RouteConstraints BlockDirected(int u, int v) { BlockedDirectedEdges.Add(((long)u << 32) | (uint)v); return this; }
    public RouteConstraints Exclude(RoadType t) { ExcludedRoadTypes.Add(t); return this; }
    public RouteConstraints AvoidCongestionAbove(double level) { MaxCongestion = level; return this; }

    /// <summary>Heavy goods vehicles: no residential streets.</summary>
    public static RouteConstraints HeavyVehicle() => new RouteConstraints().Exclude(RoadType.Residential);

    public bool IsNodeAllowed(int v) => !BlockedNodes.Contains(v);

    public bool IsEdgeAllowed(Edge e) =>
        !BlockedRoads.Contains(e.RoadId)
        && !ExcludedRoadTypes.Contains(e.Type)
        && (MaxCongestion is null || e.Congestion <= MaxCongestion.Value)
        && !BlockedDirectedEdges.Contains(((long)e.From << 32) | (uint)e.To)
        && !BlockedNodes.Contains(e.To);
}
