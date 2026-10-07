namespace UrbanRouting.Graph;

/// <summary>
/// REJECTED ALTERNATIVE, kept only for the empirical comparison.
/// V×V matrix of minimum distances (+∞ = no road). Edge lookup O(1), but neighbour iteration is O(V)
/// per node and memory is Θ(V²) regardless of how many roads exist.
/// </summary>
public sealed class AdjacencyMatrixNetwork
{
    private readonly double[,] _w;
    public int NodeCount { get; }

    public AdjacencyMatrixNetwork(RoadNetwork g)
    {
        NodeCount = g.NodeCount;
        _w = new double[NodeCount, NodeCount];
        for (int i = 0; i < NodeCount; i++)
            for (int j = 0; j < NodeCount; j++) _w[i, j] = double.PositiveInfinity;
        for (int u = 0; u < NodeCount; u++)
            foreach (var e in g.OutEdges(u))
                if (e.DistanceKm < _w[u, e.To]) _w[u, e.To] = e.DistanceKm;
    }

    public bool HasEdge(int u, int v) => !double.IsPositiveInfinity(_w[u, v]);
    public double Weight(int u, int v) => _w[u, v];

    /// <summary>Sum of all edge weights by visiting every neighbour of every node: Θ(V²).</summary>
    public double SumAllNeighbourWeights()
    {
        double s = 0;
        for (int u = 0; u < NodeCount; u++)
            for (int v = 0; v < NodeCount; v++)
            {
                double w = _w[u, v];
                if (!double.IsPositiveInfinity(w)) s += w;
            }
        return s;
    }
}
