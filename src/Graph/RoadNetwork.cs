using UrbanRouting.Models;

namespace UrbanRouting.Graph;

/// <summary>
/// CHOSEN REPRESENTATION: directed, weighted adjacency list.
///   - _out[u]  : outgoing edges of u   -> neighbour iteration O(deg(u))   (hot path of every algorithm)
///   - _in[v]   : incoming edges of v   -> reverse traversal for strong-connectivity checks
///   - _index   : hash map (u,v)->edge  -> O(1) average edge lookup / live congestion update
/// Space: O(V + E). A road network is sparse (E ≈ 2–4·V), so this beats a V×V matrix by orders of magnitude.
/// </summary>
public sealed class RoadNetwork
{
    private readonly List<Node> _nodes = new();
    private readonly List<List<Edge>> _out = new();
    private readonly List<List<Edge>> _in = new();
    private readonly Dictionary<long, Edge> _index = new(EdgeKeyComparer.Instance);
    private int _edgeCount;
    private int _nextRoadId;

    public int NodeCount => _nodes.Count;
    public int EdgeCount => _edgeCount;
    /// <summary>Number of physical roads (a two-way road counts once).</summary>
    public int RoadCount => _nextRoadId;
    public int MaxSpeedLimitKmh { get; private set; } = 1;
    public IReadOnlyList<Node> Nodes => _nodes;

    private static long Key(int u, int v) => ((long)u << 32) | (uint)v;

    public int AddNode(string name, double lat, double lon, double x, double y, NodeType type = NodeType.Junction)
    {
        int id = _nodes.Count;
        _nodes.Add(new Node(id, name, lat, lon, x, y, type));
        _out.Add(new List<Edge>());
        _in.Add(new List<Edge>());
        return id;
    }

    /// <summary>Adds a road. twoWay=true stores both directions with a shared RoadId. O(1) amortised.</summary>
    public void AddRoad(int from, int to, double distanceKm, int speedLimitKmh, RoadType type, double congestion, bool twoWay, string name = "")
    {
        ValidateNode(from); ValidateNode(to);
        int roadId = _nextRoadId;
        // Build both edges first: if validation throws, the network is left unchanged.
        var fwd = new Edge(from, to, roadId, distanceKm, speedLimitKmh, type, congestion, name);
        var back = twoWay ? new Edge(to, from, roadId, distanceKm, speedLimitKmh, type, congestion, name) : null;
        _nextRoadId++;
        AddDirected(fwd);
        if (back != null) AddDirected(back);
        MaxSpeedLimitKmh = Math.Max(MaxSpeedLimitKmh, speedLimitKmh);
    }

    private void AddDirected(Edge e)
    {
        _out[e.From].Add(e);
        _in[e.To].Add(e);
        _edgeCount++;
        // Parallel roads between the same pair are allowed (the algorithms handle them naturally);
        // the index keeps the shortest one for O(1) lookups.
        long k = Key(e.From, e.To);
        if (!_index.TryGetValue(k, out var existing) || e.DistanceKm < existing.DistanceKm) _index[k] = e;
    }

    public Node GetNode(int id) { ValidateNode(id); return _nodes[id]; }
    public IReadOnlyList<Edge> OutEdges(int u) => _out[u];
    public IReadOnlyList<Edge> InEdges(int v) => _in[v];

    /// <summary>O(1) average edge-existence lookup via hash index.</summary>
    public bool TryGetEdge(int u, int v, out Edge edge) => _index.TryGetValue(Key(u, v), out edge!);

    /// <summary>O(deg(u)) lookup by scanning the list — used only to benchmark why the index exists.</summary>
    public Edge? FindEdgeByScan(int u, int v)
    {
        foreach (var e in _out[u]) if (e.To == v) return e;
        return null;
    }

    /// <summary>Live traffic update on u->v (and v->u if the road is two-way). O(1) average.</summary>
    public bool UpdateCongestion(int u, int v, double congestion, bool bothDirections = true)
    {
        bool found = false;
        if (TryGetEdge(u, v, out var e)) { e.Congestion = congestion; found = true; }
        if (bothDirections && TryGetEdge(v, u, out var r)) { r.Congestion = congestion; found = true; }
        return found;
    }

    public int? FindNodeByName(string name)
    {
        foreach (var n in _nodes)
            if (n.Name.Equals(name, StringComparison.OrdinalIgnoreCase)) return n.Id;
        return null;
    }

    /// <summary>Straight-line (Euclidean) distance in km between two nodes — lower bound for A*.</summary>
    public double StraightLineKm(int a, int b)
    {
        var p = _nodes[a]; var q = _nodes[b];
        double dx = p.X - q.X, dy = p.Y - q.Y;
        return Math.Sqrt(dx * dx + dy * dy);
    }

    /// <summary>
    /// Snaps a GPS coordinate to the nearest node (optionally only nodes passing a filter, e.g. "on a main road").
    /// Linear scan O(V) — fine for occasional look-ups; a production system would use a k-d tree / R-tree (O(log V)).
    /// </summary>
    public int FindNearestNode(double lat, double lon, Func<int, bool>? filter = null)
    {
        int best = -1; double bestD = double.PositiveInfinity;
        double cos = Math.Cos(lat * Math.PI / 180);
        for (int i = 0; i < _nodes.Count; i++)
        {
            if (filter != null && !filter(i)) continue;
            double dLat = _nodes[i].Lat - lat, dLon = (_nodes[i].Lon - lon) * cos;
            double d = dLat * dLat + dLon * dLon;
            if (d < bestD) { bestD = d; best = i; }
        }
        if (best < 0) throw new InvalidOperationException("No node satisfies the filter.");
        return best;
    }

    /// <summary>
    /// Induced sub-network on the given nodes (ids re-numbered 0..k-1), keeping two-way roads paired.
    /// Used to benchmark the REAL network at several sizes. O(V + E).
    /// </summary>
    public RoadNetwork Subgraph(IReadOnlyList<int> keep)
    {
        var map = new Dictionary<int, int>(keep.Count);
        var sub = new RoadNetwork();
        foreach (int v in keep)
        {
            if (map.ContainsKey(v)) continue;
            var n = _nodes[v];
            map[v] = sub.AddNode(n.Name, n.Lat, n.Lon, n.X, n.Y, n.Type);
        }
        var done = new HashSet<int>();
        foreach (int v in keep)
            foreach (var e in _out[v])
            {
                if (!map.TryGetValue(e.To, out int b) || !done.Add(e.RoadId)) continue;
                bool twoWay = _out[e.To].Any(r => r.To == e.From && r.RoadId == e.RoadId);
                sub.AddRoad(map[e.From], b, e.DistanceKm, e.SpeedLimitKmh, e.Type, e.Congestion, twoWay, e.Name);
            }
        return sub;
    }

    public void ValidateNode(int id)
    {
        if ((uint)id >= (uint)_nodes.Count)
            throw new ArgumentOutOfRangeException(nameof(id), $"Node {id} does not exist (network has {_nodes.Count} nodes).");
    }
}

/// <summary>
/// Why this exists: .NET's long.GetHashCode() is (low 32 bits XOR high 32 bits). For key (u,v) that is u^v,
/// so neighbouring junctions (u, u+1) all hash to tiny values → massive collisions → O(1) lookup degrades to O(n)
/// and building a 40k-node network took 11 s. SplitMix64 finaliser spreads the bits; build time became linear again.
/// </summary>
internal sealed class EdgeKeyComparer : IEqualityComparer<long>
{
    public static readonly EdgeKeyComparer Instance = new();
    public bool Equals(long a, long b) => a == b;
    public int GetHashCode(long k)
    {
        ulong z = (ulong)k + 0x9E3779B97F4A7C15UL;
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9UL;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBUL;
        return (int)(z ^ (z >> 31));
    }
}
