using UnityEngine;
using System.Collections.Generic;

public class PathFinder : MonoBehaviour
{
    private class AStarEntry
    {
        public GraphNode node;
        public float g;              // cost from start to this entry
        public float f;              // g + heuristic
        public Vector3 position;     // last waypoint (start center or wall midpoint)
        public AStarEntry parent;    // previous entry in chain

        public AStarEntry(GraphNode node, float g, float f, Vector3 position, AStarEntry parent)
        {
            this.node = node;
            this.g = g;
            this.f = f;
            this.position = position;
            this.parent = parent;
        }
    }
    
    //private int inf = -1;

    /*
    private float getCost(GraphNode node, GraphNode start, GraphNode destination, Vector3 target)
    {
        Vector3 dirToTarget = target - node.GetCenter();
        Vector3 dirToStart = start.GetCenter() - node.GetCenter();
        return dirToTarget.Normalize() + dirToStart.Normalize()
    } */

    // Assignment 2: Implement AStar
    //
    // DO NOT CHANGE THIS SIGNATURE (parameter types + return type)
    // AStar will be given the start node, destination node and the target position, and should return 
    // a path as a list of positions the agent has to traverse to reach its destination, as well as the
    // number of nodes that were expanded to find this path
    // The last entry of the path will be the target position, and you can also use it to calculate the heuristic
    // value of nodes you add to your search frontier; the number of expanded nodes tells us if your search was
    // efficient
    //
    // Take a look at StandaloneTests.cs for some test cases
    public static (List<Vector3>, int) AStar(GraphNode start, GraphNode destination, Vector3 target)
    {
        // Implement A* here
        // Frontier sorted by f-value
        List<AStarEntry> frontier = new List<AStarEntry>();
        HashSet<int> closed = new HashSet<int>();
        Dictionary<int, float> bestG = new Dictionary<int, float>();
        int expansions = 0;

        // Initialize with start node
        Vector3 startPos = start.GetCenter();
        float h0 = Vector3.Distance(startPos, target);
        AStarEntry startEntry = new AStarEntry(start, 0f, h0, startPos, null);
        frontier.Insert(0, startEntry);
        bestG[start.GetID()] = 0f;

        while (frontier.Count > 0)
        {
            // Pop lowest-f
            AStarEntry current = frontier[0];
            frontier.RemoveAt(0);

            // Goal check
            if (current.node.GetID() == destination.GetID())
            {
                // Reconstruct path via helper
                return (ReconstructPath(current, target), expansions);
            }

            closed.Add(current.node.GetID());
            expansions++;

            // Expand neighbors
            foreach (GraphNeighbor nbr in current.node.GetNeighbors())
            {
                GraphNode nextNode = nbr.GetNode();
                int nextID = nextNode.GetID();
                if (closed.Contains(nextID)) continue;

                // Midpoint of shared wall
                Vector3 mid = nbr.GetWall().midpoint;
                float gNew = current.g + Vector3.Distance(current.position, mid);

                // Skip if not an improvement
                if (bestG.TryGetValue(nextID, out float prevG) && prevG <= gNew)
                    continue;

                bestG[nextID] = gNew;
                float fNew = gNew + Vector3.Distance(mid, target);
                AStarEntry entry = new AStarEntry(nextNode, gNew, fNew, mid, current);
                InsertSorted(frontier, entry);
            }
        }

        // No path found: return only target
        return (new List<Vector3> { target }, expansions);
    }

    private static List<Vector3> ReconstructPath(AStarEntry endEntry, Vector3 target)
    {
        List<Vector3> path = new List<Vector3>();
        AStarEntry e = endEntry;
        while (e.parent != null)
        {
            path.Add(e.position);
            e = e.parent;
        }
        path.Reverse();
        path.Add(target);
        return path;
    }

    // Keep frontier sorted by f ascending
    private static void InsertSorted(List<AStarEntry> list, AStarEntry entry)
    {
        int low = 0;
        int high = list.Count;
        while (low < high)
        {
            int mid = (low + high) / 2;
            if (entry.f < list[mid].f) high = mid;
            else low = mid + 1;
        }
        list.Insert(low, entry);
    }

    public Graph graph;
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        EventBus.OnTarget += PathFind;
        EventBus.OnSetGraph += SetGraph;
    }

    // Update is called once per frame
    void Update()
    { }

    public void SetGraph(Graph g)
    {
        graph = g;
    }

    // entry point
    public void PathFind(Vector3 target)
    {
        if (graph == null) return;

        // find start and destination nodes in graph
        GraphNode start = null;
        GraphNode destination = null;
        foreach (var n in graph.all_nodes)
        {
            if (Util.PointInPolygon(transform.position, n.GetPolygon()))
            {
                start = n;
            }
            if (Util.PointInPolygon(target, n.GetPolygon()))
            {
                destination = n;
            }
        }
        if (destination != null)
        {
            // only find path if destination is inside graph
            EventBus.ShowTarget(target);
            (List<Vector3> path, int expanded) = PathFinder.AStar(start, destination, target);

            Debug.Log("found path of length " + path.Count + " expanded " + expanded + " nodes, out of: " + graph.all_nodes.Count);
            EventBus.SetPath(path);
        }
    }

    

 
}
