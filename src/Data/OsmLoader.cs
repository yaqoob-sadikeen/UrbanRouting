using System.Globalization;
using System.Text;
using UrbanRouting.Graph;
using UrbanRouting.Models;

namespace UrbanRouting.Data;

public sealed record OsmLoadReport(int NodesRead, int EdgeRowsRead, int SelfLoopsRemoved, int ReverseRowsMerged,
    int LengthsRaisedToStraightLine, int BadRows, int OneWayRoads, int TwoWayRoads);

/// <summary>
/// Loads the REAL Colombo District drivable road network exported from OpenStreetMap with OSMnx
/// (colombo_nodes_osm.csv: osmid,y,x,street_count · colombo_edges_osm.csv: u,v,length,highway,oneway,speed_kph,travel_time,name,bridge).
///
/// Cleaning / mapping decisions (all counted in the report):
///  • OSM ids (64-bit) are re-numbered 0..V-1 so every algorithm can use plain arrays (O(1) access).
///  • OSMnx stores a two-way street as TWO rows (u→v and v→u). We keep the u&lt;v row and add it as one two-way
///    road (shared RoadId), so bridge detection treats it as ONE physical road. One-way rows are added directly.
///  • Self-loops (u = v, e.g. turning circles) are removed: they never lie on a shortest path.
///  • length (m) → km. If a curved road's recorded length is shorter than the straight line between its end
///    nodes (rounding), it is raised to the straight line, guaranteeing the A* heuristic stays admissible.
///  • highway tag → RoadType; bridge=yes/viaduct → Bridge. Lists like "['residential','tertiary']" use the first tag.
///  • speed_kph: OSM maxspeed where tagged, otherwise OSMnx's per-road-type mean (imputed in the export).
///  • congestion: NOT in OpenStreetMap → assumed peak-hour level per road class (documented assumption).
/// </summary>
public static class OsmLoader
{
    private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

    /// <summary>Assumed weekday-peak congestion by road class (OSM has no traffic data).</summary>
    public static double AssumedCongestion(RoadType t) => t switch
    {
        RoadType.Highway => 0.15, RoadType.Arterial => 0.60, RoadType.Bridge => 0.60,
        RoadType.Secondary => 0.45, _ => 0.25
    };

    public static RoadType MapHighway(string highway, string bridge)
    {
        string b = bridge.Trim().ToLowerInvariant();
        if (b.Contains("yes") || b.Contains("viaduct")) return RoadType.Bridge;
        string h = highway.Trim().TrimStart('[').Trim('\'', '"', ' ');
        int comma = h.IndexOf(',');
        if (comma >= 0) h = h[..comma].Trim('\'', '"', ' ', ']');
        h = h.Replace("_link", "");
        return h switch
        {
            "motorway" => RoadType.Highway,
            "trunk" or "primary" => RoadType.Arterial,
            "secondary" or "tertiary" => RoadType.Secondary,
            _ => RoadType.Residential   // residential, living_street, unclassified, road
        };
    }

    public static RoadNetwork Load(string nodesCsv, string edgesCsv, out OsmLoadReport report)
    {
        if (!File.Exists(nodesCsv)) throw new FileNotFoundException("OSM nodes file not found", nodesCsv);
        if (!File.Exists(edgesCsv)) throw new FileNotFoundException("OSM edges file not found", edgesCsv);
        var g = new RoadNetwork();
        var idMap = new Dictionary<long, int>();
        var raw = new List<(long id, double lat, double lon)>();
        int bad = 0;

        foreach (var f in ReadCsv(nodesCsv, out var nh))
        {
            try { raw.Add((long.Parse(f[nh["osmid"]], Inv), double.Parse(f[nh["y"]], Inv), double.Parse(f[nh["x"]], Inv))); }
            catch { bad++; }
        }
        double lat0 = raw.Average(r => r.lat), lon0 = raw.Average(r => r.lon);
        double kx = 111.320 * Math.Cos(lat0 * Math.PI / 180), ky = 110.574;
        foreach (var r in raw)
        {
            if (idMap.ContainsKey(r.id)) { bad++; continue; }
            idMap[r.id] = g.AddNode($"OSM {r.id}", r.lat, r.lon, (r.lon - lon0) * kx, (r.lat - lat0) * ky);
        }

        int rows = 0, loops = 0, merged = 0, raised = 0, oneWay = 0, twoWay = 0;
        // Pass 1: parse. Two-way rows are grouped by (min id, max id, length) so the u→v / v→u pair of the same
        // physical street is detected even if only one direction happened to be exported.
        var twoWayGroups = new Dictionary<(int, int, long), (int count, string[] row)>();
        var oneWayRows = new List<(int a, int b, string[] row)>();
        foreach (var f in ReadCsv(edgesCsv, out var eh))
        {
            rows++;
            try
            {
                if (!idMap.TryGetValue(long.Parse(f[eh["u"]], Inv), out int a) ||
                    !idMap.TryGetValue(long.Parse(f[eh["v"]], Inv), out int b)) { bad++; continue; }
                if (a == b) { loops++; continue; }
                double.Parse(f[eh["length"]], Inv); double.Parse(f[eh["speed_kph"]], Inv); // validate numerics now
                if (f[eh["oneway"]].Trim().Equals("True", StringComparison.OrdinalIgnoreCase)) { oneWayRows.Add((a, b, f)); continue; }
                var key = (Math.Min(a, b), Math.Max(a, b), (long)Math.Round(double.Parse(f[eh["length"]], Inv) * 100));
                twoWayGroups[key] = twoWayGroups.TryGetValue(key, out var gr) ? (gr.count + 1, gr.row) : (1, f);
            }
            catch { bad++; }
        }
        var hdr = ReadCsvHeader(edgesCsv);
        void Add(int a, int b, string[] f, bool two)
        {
            double km = double.Parse(f[hdr["length"]], Inv) / 1000.0;
            double straight = g.StraightLineKm(a, b);
            if (km < straight) { km = straight; raised++; }
            if (km <= 0) km = 1e-4;
            var type = MapHighway(f[hdr["highway"]], hdr.ContainsKey("bridge") ? f[hdr["bridge"]] : "");
            int speed = Math.Max(5, (int)Math.Round(double.Parse(f[hdr["speed_kph"]], Inv)));
            string name = hdr.ContainsKey("name") ? CleanName(f[hdr["name"]]) : "";
            g.AddRoad(a, b, km, speed, type, AssumedCongestion(type), twoWay: two, name);
        }
        // Pass 2: build. A group of k rows = ceil(k/2) two-way roads (normally k = 2 → one road).
        foreach (var ((a, b, _), (count, row)) in twoWayGroups)
        {
            int roads = (count + 1) / 2;
            merged += count - roads;
            for (int i = 0; i < roads; i++) { Add(a, b, row, true); twoWay++; }
        }
        foreach (var (a, b, row) in oneWayRows) { Add(a, b, row, false); oneWay++; }
        report = new OsmLoadReport(raw.Count, rows, loops, merged, raised, bad, oneWay, twoWay);
        return g;
    }

    private static string CleanName(string s)
    {
        s = s.Trim();
        if (s.StartsWith('[')) { s = s.TrimStart('[').TrimEnd(']'); int c = s.IndexOf(','); if (c >= 0) s = s[..c]; }
        return s.Trim('\'', '"', ' ');
    }

    private static Dictionary<string, int> ReadCsvHeader(string path) =>
        Split(File.ReadLines(path).First()).Select((n, i) => (n.Trim(), i)).ToDictionary(x => x.Item1, x => x.i);

    /// <summary>Minimal RFC-4180 CSV reader (quoted fields may contain commas). Yields rows after the header.</summary>
    public static IEnumerable<string[]> ReadCsv(string path, out Dictionary<string, int> header)
    {
        var lines = File.ReadLines(path).GetEnumerator();
        if (!lines.MoveNext()) throw new InvalidDataException($"{path} is empty");
        var h = Split(lines.Current);
        header = h.Select((name, i) => (name.Trim(), i)).ToDictionary(x => x.Item1, x => x.i);
        return Rows(lines, h.Length);
    }

    private static IEnumerable<string[]> Rows(IEnumerator<string> lines, int width)
    {
        while (lines.MoveNext())
        {
            if (string.IsNullOrWhiteSpace(lines.Current)) continue;
            var f = Split(lines.Current);
            if (f.Length < width) Array.Resize(ref f, width);
            for (int i = 0; i < f.Length; i++) f[i] ??= "";
            yield return f;
        }
    }

    public static string[] Split(string line)
    {
        var res = new List<string>(); var sb = new StringBuilder(); bool q = false;
        for (int i = 0; i < line.Length; i++)
        {
            char c = line[i];
            if (q) { if (c == '"') { if (i + 1 < line.Length && line[i + 1] == '"') { sb.Append('"'); i++; } else q = false; } else sb.Append(c); }
            else if (c == '"') q = true;
            else if (c == ',') { res.Add(sb.ToString()); sb.Clear(); }
            else sb.Append(c);
        }
        res.Add(sb.ToString());
        return res.ToArray();
    }
}
