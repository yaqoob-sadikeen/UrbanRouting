namespace UrbanRouting.Models;

/// <summary>
/// A location in the network (intersection, hub, hospital...).
/// X/Y are planar kilometre coordinates used by the A* heuristic; Lat/Lon are kept for display.
/// </summary>
public sealed record Node(int Id, string Name, double Lat, double Lon, double X, double Y, NodeType Type)
{
    public override string ToString() => $"{Name} (#{Id})";
}
