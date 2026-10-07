using System.Globalization;
using UrbanRouting.Graph;
using UrbanRouting.Models;

namespace UrbanRouting.Data;

/// <summary>
/// Loads nodes.csv / edges.csv. Defensive against malformed input: bad rows are SKIPPED with a warning
/// (not a crash) — unknown node ids, non-numeric fields, zero/negative distances, self-loops, bad enums.
/// Lat/Lon are projected to planar km (equirectangular; error &lt; 0.1% over a city) for the A* heuristic.
/// </summary>
public static class DatasetLoader
{
    private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

    public static RoadNetwork Load(string nodesCsv, string edgesCsv, List<string>? warnings = null)
    {
        warnings ??= new List<string>();
        if (!File.Exists(nodesCsv)) throw new FileNotFoundException("Nodes file not found", nodesCsv);
        if (!File.Exists(edgesCsv)) throw new FileNotFoundException("Edges file not found", edgesCsv);

        var g = new RoadNetwork();
        var rows = File.ReadAllLines(nodesCsv).Skip(1).Where(l => !string.IsNullOrWhiteSpace(l)).ToList();
        var parsed = new List<(int id, string name, double lat, double lon, NodeType type)>();
        foreach (var (line, i) in rows.Select((l, i) => (l, i + 2)))
        {
            var f = line.Split(',');
            if (f.Length < 5 || !int.TryParse(f[0], out int id) ||
                !double.TryParse(f[2], NumberStyles.Float, Inv, out double lat) ||
                !double.TryParse(f[3], NumberStyles.Float, Inv, out double lon) ||
                !Enum.TryParse(f[4].Trim(), true, out NodeType type))
            { warnings.Add($"nodes line {i}: malformed, skipped"); continue; }
            parsed.Add((id, f[1].Trim(), lat, lon, type));
        }
        // ids must be 0..n-1 so they can index arrays directly (O(1) access instead of dictionary lookups)
        parsed.Sort((a, b) => a.id.CompareTo(b.id));
        for (int k = 0; k < parsed.Count; k++)
            if (parsed[k].id != k) throw new InvalidDataException($"Node ids must be contiguous from 0; expected {k}, found {parsed[k].id}.");

        double lat0 = parsed.Count > 0 ? parsed.Average(p => p.lat) : 0, lon0 = parsed.Count > 0 ? parsed.Average(p => p.lon) : 0;
        double kmPerDegLon = 111.320 * Math.Cos(lat0 * Math.PI / 180), kmPerDegLat = 110.574;
        foreach (var p in parsed)
            g.AddNode(p.name, p.lat, p.lon, (p.lon - lon0) * kmPerDegLon, (p.lat - lat0) * kmPerDegLat, p.type);

        int ln = 1;
        foreach (var line in File.ReadAllLines(edgesCsv).Skip(1))
        {
            ln++;
            if (string.IsNullOrWhiteSpace(line)) continue;
            var f = line.Split(',');
            try
            {
                if (f.Length < 7) throw new FormatException("expected 7 columns");
                int a = int.Parse(f[0], Inv), b = int.Parse(f[1], Inv);
                double d = double.Parse(f[2], Inv);
                int speed = int.Parse(f[3], Inv);
                var type = Enum.Parse<RoadType>(f[4].Trim(), true);
                double cong = double.Parse(f[5], Inv);
                bool oneWay = f[6].Trim() == "1";
                g.AddRoad(a, b, d, speed, type, cong, twoWay: !oneWay);
            }
            catch (Exception ex) { warnings.Add($"edges line {ln}: {ex.Message} — skipped"); }
        }
        return g;
    }

    public static void Save(RoadNetwork g, string nodesCsv, string edgesCsv)
    {
        using (var w = new StreamWriter(nodesCsv))
        {
            w.WriteLine("id,name,lat,lon,type");
            foreach (var n in g.Nodes) w.WriteLine(string.Join(',', n.Id, n.Name, n.Lat.ToString("0.000000", Inv), n.Lon.ToString("0.000000", Inv), n.Type));
        }
        using (var w = new StreamWriter(edgesCsv))
        {
            w.WriteLine("from,to,distance_km,speed_limit_kmh,road_type,congestion,one_way");
            var written = new HashSet<int>();
            for (int u = 0; u < g.NodeCount; u++)
                foreach (var e in g.OutEdges(u))
                {
                    if (!written.Add(e.RoadId)) continue;
                    bool twoWay = g.TryGetEdge(e.To, e.From, out var r) && r.RoadId == e.RoadId;
                    w.WriteLine(string.Join(',', e.From, e.To, e.DistanceKm.ToString("0.###", Inv), e.SpeedLimitKmh, e.Type,
                        e.Congestion.ToString("0.##", Inv), twoWay ? 0 : 1));
                }
        }
    }
}
