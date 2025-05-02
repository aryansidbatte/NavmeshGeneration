using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.InputSystem;

public class NavMesh : MonoBehaviour
{
    // implement NavMesh generation here:
    //    the outline are Walls in counterclockwise order
    //    iterate over them, and if you find a reflex angle
    //    you have to split the polygon into two
    //    then perform the same operation on both parts
    //    until no more reflex angles are present
    //
    //    when you have a number of polygons, you will have
    //    to convert them into a graph: each polygon is a node
    //    you can find neighbors by finding shared edges between
    //    different polygons (or you can keep track of this while 
    //    you are splitting)

    // Polygon helper for convex decomposition
    private class Polygon
    {
        public List<Wall> walls;
        public Polygon(List<Wall> walls) { this.walls = walls; }

        // first reflex angle or -1 if none found
        public int FindNonConvexCorner()
        {
            int n = walls.Count;
            var verts = walls.Select(w => w.start).ToList();
            for (int i = 0; i < n; i++)
            {
                int prev = (i + n - 1) % n;
                int next = (i + 1) % n;
                Vector3 a = verts[prev] - verts[i];
                Vector3 b = verts[next] - verts[i];
                float cross = a.x * b.z - a.z * b.x;
                if (cross > Mathf.Epsilon) return i;
            }
            return -1;
        }

        // find valid split point or -1 if none found
        // inside class Polygon
        public int FindSplitPoint(int reflex)
        {
            int n = walls.Count;
            var verts = walls.Select(w => w.start).ToList();
            Vector3 r = verts[reflex];

            // try every non‑adjacent vertex
            for (int offset = 2; offset < n - 1; offset++)
            {
                int j = (reflex + offset) % n;
                Vector3 s = verts[j];
                var chord = new Wall(r, s);

                // skip match existing edge
                if (walls.Any(w => w.Same(chord)))
                    continue;

                // skip crosses any non‑adjacent wall
                bool crosses = false;
                foreach (var w in walls)
                {
                    // ignore walls that share an endpoint with the chord
                    if ((w.start - r).sqrMagnitude < Mathf.Epsilon ||
                        (w.end - r).sqrMagnitude < Mathf.Epsilon ||
                        (w.start - s).sqrMagnitude < Mathf.Epsilon ||
                        (w.end - s).sqrMagnitude < Mathf.Epsilon)
                        continue;

                    if (w.Crosses(r, s))
                    {
                        crosses = true;
                        break;
                    }
                }
                if (crosses)
                    continue;

                // skip if chord midpoint outside polygon
                Vector3 mid = (r + s) * 0.5f;
                if (!Util.PointInPolygon(mid, walls))
                    continue;

                // accepted!
                return j;
            }

            // no valid split found
            return -1;
        }

        // split polygon at reflex and split
        public (Polygon a, Polygon b) Split(int reflex, int split)
        {
            if (split < reflex) return Split(split, reflex);
            var verts = walls.Select(w => w.start).ToList();
            var v1 = new List<Vector3>();
            int idx = reflex;
            while (true)
            {
                v1.Add(verts[idx]);
                if (idx == split) break;
                idx = (idx + 1) % verts.Count;
            }
            v1.Add(verts[reflex]);
            var v2 = new List<Vector3>();
            idx = split;
            while (true)
            {
                v2.Add(verts[idx]);
                if (idx == reflex) break;
                idx = (idx + 1) % verts.Count;
            }
            v2.Add(verts[split]);

            var wallsA = new List<Wall>();
            for (int i = 0; i < v1.Count - 1; i++)
                wallsA.Add(new Wall(v1[i], v1[i + 1]));

            var wallsB = new List<Wall>();
            for (int i = 0; i < v2.Count - 1; i++)
                wallsB.Add(new Wall(v2[i], v2[i + 1]));

            return (new Polygon(wallsA), new Polygon(wallsB));
        }
    }

    // Main navmesh builder
    public Graph MakeNavMesh(List<Wall> outline)
    {
        // CCW
        var walls = new List<Wall>(outline);
        float area = walls.Sum(w => w.start.x * w.end.z - w.end.x * w.start.z);
        if (area < 0)
            walls = walls.Select(w => new Wall(w.end, w.start)).Reverse().ToList();

        // convex decomposition
        var polys = new List<Polygon> { new Polygon(walls) };
        for (int i = 0; i < polys.Count; )
        {
            int r = polys[i].FindNonConvexCorner();
            if (r < 0)
            {
                i++;
                continue;
            }
            int s = polys[i].FindSplitPoint(r);
            if (s < 0)
            {
                i++;
                continue;
            }
            var (a, b) = polys[i].Split(r, s);
            polys.RemoveAt(i);
            polys.Add(a);
            polys.Add(b);
        }

        // build graph
        var graph = new Graph { outline = new List<Wall>(walls), all_nodes = new List<GraphNode>() };
        for (int id = 0; id < polys.Count; id++)
            graph.all_nodes.Add(new GraphNode(id, polys[id].walls));
        DefineNeighbors(graph.all_nodes);
        Debug.Log($"got navmesh: {graph.all_nodes.Count} convex cells");
        return graph;
    }
    
    private static void DefineNeighbors(List<GraphNode> nodes)
    {
        for (int i = 0; i < nodes.Count; i++)
            for (int j = i + 1; j < nodes.Count; j++)
            {
                var p1 = nodes[i].GetPolygon();
                var p2 = nodes[j].GetPolygon();
                for (int e1 = 0; e1 < p1.Count; e1++)
                    for (int e2 = 0; e2 < p2.Count; e2++)
                        if (p1[e1].Same(p2[e2]))
                        {
                            nodes[i].AddNeighbor(nodes[j], e1);
                            nodes[j].AddNeighbor(nodes[i], e2);
                        }
            }
    }

    private void SetMap(List<Wall> outline)
    {
        var navmesh = MakeNavMesh(outline);
        EventBus.SetGraph(navmesh);
    }

    void Start() => EventBus.OnSetMap += SetMap;
    void Update() { }
}
