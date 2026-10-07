using UrbanRouting.Graph;

namespace UrbanRouting.Analysis;

public sealed record TraversalStats(int Visited, int PeakAuxiliarySize);

/// <summary>
/// Connectivity checks. Both BFS (queue) and DFS (explicit stack — never recursion, so a 1,000,000-node
/// chain cannot overflow the call stack) are O(V+E) time. They differ in AUXILIARY SPACE:
/// BFS holds a whole frontier "ring", DFS holds one long path. Both are measured empirically.
///  • Weak connectivity   : treat roads as undirected (use out + in edges).
///  • Strong connectivity : every node reaches every other respecting one-way streets
///                          ⇔ BFS from any node reaches all on G AND on reverse(G)   (Kosaraju's insight). O(V+E).
/// </summary>
public static class ConnectivityAnalyzer
{
    public static TraversalStats BfsUndirected(RoadNetwork g, int start, bool[] visited)
    {
        var q = new Queue<int>();
        visited[start] = true; q.Enqueue(start);
        int count = 0, peak = 1;
        while (q.Count > 0)
        {
            int u = q.Dequeue(); count++;
            foreach (var e in g.OutEdges(u)) if (!visited[e.To]) { visited[e.To] = true; q.Enqueue(e.To); }
            foreach (var e in g.InEdges(u)) if (!visited[e.From]) { visited[e.From] = true; q.Enqueue(e.From); }
            if (q.Count > peak) peak = q.Count;
        }
        return new TraversalStats(count, peak);
    }

    // Stack-based depth-first traversal (nodes marked when pushed, so each node enters the stack once: O(V) stack bound).
    public static TraversalStats DfsUndirected(RoadNetwork g, int start, bool[] visited)
    {
        var st = new Stack<int>();
        visited[start] = true; st.Push(start);
        int count = 0, peak = 1;
        while (st.Count > 0)
        {
            int u = st.Pop(); count++;
            foreach (var e in g.OutEdges(u)) if (!visited[e.To]) { visited[e.To] = true; st.Push(e.To); }
            foreach (var e in g.InEdges(u)) if (!visited[e.From]) { visited[e.From] = true; st.Push(e.From); }
            if (st.Count > peak) peak = st.Count;
        }
        return new TraversalStats(count, peak);
    }

    /// <summary>Labels every node with a (weakly) connected component id. O(V+E).</summary>
    public static int[] ComponentLabels(RoadNetwork g, out int componentCount)
    {
        int n = g.NodeCount;
        var label = new int[n];
        Array.Fill(label, -1);
        componentCount = 0;
        var q = new Queue<int>();
        for (int s = 0; s < n; s++)
        {
            if (label[s] != -1) continue;
            label[s] = componentCount; q.Enqueue(s);
            while (q.Count > 0)
            {
                int u = q.Dequeue();
                foreach (var e in g.OutEdges(u)) if (label[e.To] == -1) { label[e.To] = componentCount; q.Enqueue(e.To); }
                foreach (var e in g.InEdges(u)) if (label[e.From] == -1) { label[e.From] = componentCount; q.Enqueue(e.From); }
            }
            componentCount++;
        }
        return label;
    }

    public static bool IsWeaklyConnected(RoadNetwork g, bool useDfs = false)
    {
        if (g.NodeCount <= 1) return true;                 // empty / single node: trivially connected
        var visited = new bool[g.NodeCount];
        var stats = useDfs ? DfsUndirected(g, 0, visited) : BfsUndirected(g, 0, visited);
        return stats.Visited == g.NodeCount;
    }

    /// <summary>Returns true if every location can reach every other obeying one-way rules.
    /// unreachableFrom0 / cannotReach0 list the offending nodes for diagnostics.</summary>
    public static bool IsStronglyConnected(RoadNetwork g, out List<int> unreachableFrom0, out List<int> cannotReach0)
    {
        unreachableFrom0 = new(); cannotReach0 = new();
        if (g.NodeCount <= 1) return true;
        var fwd = Directed(g, 0, reverse: false);
        var bwd = Directed(g, 0, reverse: true);
        for (int v = 0; v < g.NodeCount; v++)
        {
            if (!fwd[v]) unreachableFrom0.Add(v);
            if (!bwd[v]) cannotReach0.Add(v);
        }
        return unreachableFrom0.Count == 0 && cannotReach0.Count == 0;
    }

    private static bool[] Directed(RoadNetwork g, int s, bool reverse)
    {
        var seen = new bool[g.NodeCount];
        var q = new Queue<int>();
        seen[s] = true; q.Enqueue(s);
        while (q.Count > 0)
        {
            int u = q.Dequeue();
            if (!reverse) { foreach (var e in g.OutEdges(u)) if (!seen[e.To]) { seen[e.To] = true; q.Enqueue(e.To); } }
            else { foreach (var e in g.InEdges(u)) if (!seen[e.From]) { seen[e.From] = true; q.Enqueue(e.From); } }
        }
        return seen;
    }
}
