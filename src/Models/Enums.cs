namespace UrbanRouting.Models;

/// <summary>Road classification. Drives speed limits and constrained-routing rules
/// (e.g. heavy vehicles may not use Residential roads).</summary>
public enum RoadType { Highway, Arterial, Secondary, Residential, Bridge }

/// <summary>What a location represents. Used for emergency-response queries.</summary>
public enum NodeType { Junction, TransitHub, Hospital, FireStation }

/// <summary>Which quantity a route should minimise.</summary>
public enum Metric { Distance, Time, Weighted }
