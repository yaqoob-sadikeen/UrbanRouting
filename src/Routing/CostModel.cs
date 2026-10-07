using UrbanRouting.Graph;
using UrbanRouting.Models;

namespace UrbanRouting.Routing;

/// <summary>
/// Turns a metric into (1) an edge cost and (2) an ADMISSIBLE, CONSISTENT A* heuristic.
///   Distance : cost = km,                heuristic = straight-line km           (road km ≥ straight-line km)
///   Time     : cost = minutes,           heuristic = straight-line km / vmax    (can't beat fastest road in a straight line)
///   Weighted : cost = α·km + (1-α)·min,  heuristic = α·h_dist + (1-α)·h_time    (sum of two lower bounds is a lower bound)
/// </summary>
public sealed class CostModel
{
    public Metric Metric { get; }
    public double Alpha { get; }

    public CostModel(Metric metric, double alpha = 0.5)
    {
        if (alpha < 0 || alpha > 1) throw new ArgumentOutOfRangeException(nameof(alpha), "Alpha must be in [0,1].");
        Metric = metric; Alpha = alpha;
    }

    public static readonly CostModel ShortestDistance = new(Metric.Distance);
    public static readonly CostModel FastestTime = new(Metric.Time);

    public double Cost(Edge e) => Metric switch
    {
        Metric.Distance => e.DistanceKm,
        Metric.Time => e.TravelTimeMin,
        _ => Alpha * e.DistanceKm + (1 - Alpha) * e.TravelTimeMin
    };

    public double Heuristic(RoadNetwork g, int v, int target)
    {
        double km = g.StraightLineKm(v, target);
        double minutes = km / g.MaxSpeedLimitKmh * 60.0;
        return Metric switch
        {
            Metric.Distance => km,
            Metric.Time => minutes,
            _ => Alpha * km + (1 - Alpha) * minutes
        };
    }

    public override string ToString() => Metric == Metric.Weighted ? $"Weighted(α={Alpha:0.00})" : Metric.ToString();
}
