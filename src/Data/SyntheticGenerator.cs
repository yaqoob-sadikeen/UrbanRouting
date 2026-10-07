using UrbanRouting.Graph;
using UrbanRouting.Models;

namespace UrbanRouting.Data;

/// <summary>
/// Reproducible (seeded) synthetic cities for scaling tests.
///  • SparseCity : jittered grid street pattern, block = 400 m. ~8% of streets missing, 3% one-way,
///                 every 5th row/column is an arterial, sparse diagonal shortcuts. E ≈ 3.7·V (like real roads).
///  • DenseTransit : random complete-ish network (each pair linked with probability p) — models a dense
///                 transit/flight-style graph where E = Θ(V²), to show where the rankings flip.
/// Road distance = straight-line × U(1.00, 1.25) so the A* heuristic stays admissible.
/// </summary>
public static class SyntheticGenerator
{
    public static RoadNetwork SparseCity(int nodes, int seed = 42)
    {
        if (nodes < 1) throw new ArgumentOutOfRangeException(nameof(nodes));
        var rnd = new Random(seed);
        var g = new RoadNetwork();
        int side = (int)Math.Ceiling(Math.Sqrt(nodes));
        const double block = 0.4;
        for (int i = 0; i < nodes; i++)
        {
            int r = i / side, c = i % side;
            double x = c * block + (rnd.NextDouble() - 0.5) * 0.15, y = r * block + (rnd.NextDouble() - 0.5) * 0.15;
            g.AddNode($"N{i}", 6.9 + y / 110.574, 79.85 + x / 110.5, x, y);
        }
        for (int i = 0; i < nodes; i++)
        {
            int r = i / side, c = i % side;
            if (c + 1 < side && i + 1 < nodes) Street(g, rnd, i, i + 1, arterial: r % 5 == 0);
            if (i + side < nodes) Street(g, rnd, i, i + side, arterial: c % 5 == 0);
            if (rnd.NextDouble() < 0.05 && c + 1 < side && i + side + 1 < nodes) Street(g, rnd, i, i + side + 1, arterial: false);
        }
        return g;
    }

    private static void Street(RoadNetwork g, Random rnd, int a, int b, bool arterial)
    {
        if (!arterial && rnd.NextDouble() < 0.08) return;                   // missing street
        double d = Math.Max(0.01, g.StraightLineKm(a, b) * (1.0 + rnd.NextDouble() * 0.25));
        var type = arterial ? RoadType.Arterial : (rnd.NextDouble() < 0.4 ? RoadType.Residential : RoadType.Secondary);
        int speed = type switch { RoadType.Arterial => 50, RoadType.Secondary => 40, _ => 30 };
        bool oneWay = !arterial && rnd.NextDouble() < 0.03;
        g.AddRoad(a, b, d, speed, type, rnd.NextDouble() * 0.8, twoWay: !oneWay);
    }

    public static RoadNetwork DenseTransit(int nodes, double p, int seed = 7)
    {
        var rnd = new Random(seed);
        var g = new RoadNetwork();
        double size = Math.Sqrt(nodes) * 0.5;
        for (int i = 0; i < nodes; i++)
        {
            double x = rnd.NextDouble() * size, y = rnd.NextDouble() * size;
            g.AddNode($"T{i}", 6.9 + y / 110.574, 79.85 + x / 110.5, x, y, NodeType.TransitHub);
        }
        for (int i = 0; i < nodes; i++)
            for (int j = i + 1; j < nodes; j++)
                if (rnd.NextDouble() < p)
                {
                    double d = Math.Max(0.01, g.StraightLineKm(i, j) * (1.0 + rnd.NextDouble() * 0.25));
                    g.AddRoad(i, j, d, 60, RoadType.Arterial, rnd.NextDouble() * 0.5, twoWay: true);
                }
        return g;
    }

    /// <summary>A single long chain — worst case for recursive DFS depth.</summary>
    public static RoadNetwork Chain(int nodes)
    {
        var g = new RoadNetwork();
        for (int i = 0; i < nodes; i++) g.AddNode($"C{i}", 0, 0, i * 0.1, 0);
        for (int i = 0; i + 1 < nodes; i++) g.AddRoad(i, i + 1, 0.1, 50, RoadType.Secondary, 0, true);
        return g;
    }
}
