namespace UrbanRouting.Models;

/// <summary>
/// A directed road segment. A two-way road is stored as two Edge objects sharing one RoadId,
/// which lets connectivity analysis treat them as ONE physical road.
/// Attributes kept (Decision Area 4): distance, speed limit, road type, congestion.
/// Travel time is DERIVED, so a congestion update automatically changes travel time.
/// </summary>
public sealed class Edge
{
    /// <summary>Congestion is clamped to this so effective speed never reaches zero.</summary>
    public const double MaxCongestion = 0.95;
    /// <summary>At full congestion (1.0) a road runs at 25% of its speed limit.</summary>
    public const double CongestionSlowdown = 0.75;

    public int From { get; }
    public int To { get; }
    public int RoadId { get; }
    public double DistanceKm { get; }
    public int SpeedLimitKmh { get; }
    public RoadType Type { get; }
    /// <summary>Street name from OpenStreetMap (may be empty).</summary>
    public string Name { get; }
    private double _congestion;

    public Edge(int from, int to, int roadId, double distanceKm, int speedLimitKmh, RoadType type, double congestion, string name = "")
    {
        if (from == to) throw new ArgumentException($"Self-loop {from}->{to} is not a valid road segment.");
        if (!(distanceKm > 0) || double.IsInfinity(distanceKm))
            throw new ArgumentOutOfRangeException(nameof(distanceKm), "Distance must be a finite positive number (Dijkstra/A* require non-negative weights).");
        if (speedLimitKmh <= 0) throw new ArgumentOutOfRangeException(nameof(speedLimitKmh), "Speed limit must be positive.");
        From = from; To = to; RoadId = roadId; DistanceKm = distanceKm; SpeedLimitKmh = speedLimitKmh; Type = type; Name = name ?? "";
        Congestion = congestion;
    }

    /// <summary>Real-time congestion level in [0, 0.95]. Mutable to support live traffic feeds.</summary>
    public double Congestion
    {
        get => _congestion;
        set => _congestion = double.IsNaN(value) ? 0 : Math.Clamp(value, 0, MaxCongestion);
    }

    public double EffectiveSpeedKmh => SpeedLimitKmh * (1 - CongestionSlowdown * _congestion);

    /// <summary>Minutes to traverse this segment under current congestion.</summary>
    public double TravelTimeMin => DistanceKm / EffectiveSpeedKmh * 60.0;

    public override string ToString() => $"{From}->{To} [{Type}, {DistanceKm:0.00} km, {TravelTimeMin:0.0} min]";
}
