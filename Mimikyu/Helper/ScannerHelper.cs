using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;
using MIConvexHull;
using Rhino;
using Rhino.Geometry;
using Rhino.Geometry.Collections;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Mimikyu.Helper
{
    internal class ScannerHelper
    {
        public class ScanFace
        {

            public Point3d Center;
            public Vector3d Normal;
            public Vector3d UAxis;
            public Vector3d VAxis;
            public double Width;
            public double Height;

            public Point3d PointAt(double u, double v)
            {
                double uu = (u - 0.5) * Width;
                double vv = (v - 0.5) * Height;

                return Center + UAxis * uu + VAxis * vv;
            }
        }

        internal class RawDefectBranch
        {
            public int Index;
            public GH_Path Path;
            public List<Point3d> Points;
            public BoundingBox BoundingBox;
            public Point3d Center;
        }

        internal class MergedRegion
        {
            public List<int> RawBranchIndices = new List<int>();
            public List<Point3d> Points = new List<Point3d>();
        }

        public class HullVertex : IVertex
        {
            public double[] Position { get; set; }

            public int Index { get; set; }

            public HullVertex(Point3d p, int index)
            {
                Position = new[]
                {
            p.X,
            p.Y,
            p.Z
        };

                Index = index;
            }
        }

        public static Mesh BrepToSingleMesh(Brep brep)
        {
            Mesh[] meshes = Mesh.CreateFromBrep(brep, MeshingParameters.Default);

            Mesh joined = new Mesh();

            if (meshes == null || meshes.Length == 0)
                return joined;

            foreach (Mesh m in meshes)
            {
                if (m != null)
                    joined.Append(m);
            }

            joined.Vertices.CombineIdentical(true, true);
            joined.Vertices.CullUnused();
            joined.UnifyNormals();
            joined.Normals.ComputeNormals();
            joined.Compact();

            return joined;
        }

        public static Box GetMinimumBoundingBox3D(Mesh inputMesh)
        {

            // Note: The inputMesh is already a convex hull

            MeshFaceList faces = inputMesh.Faces;

            List<Plane> planes = new List<Plane>();

            // Get all the possible planes
            foreach (MeshFace face in faces)
            {
                List<Point3d> pts = new List<Point3d>();
                pts.Add(inputMesh.Vertices[face.A]);
                pts.Add(inputMesh.Vertices[face.B]);
                pts.Add(inputMesh.Vertices[face.C]);
                Plane tempPlane = new Plane();
                if (Plane.FitPlaneToPoints(pts, out tempPlane) == PlaneFitResult.Success)
                    planes.Add(tempPlane);
            }

            List<Box> orientedBoxes = new List<Box>();

            foreach (Plane pln in planes)
            {
                Box bb = new Box();
                inputMesh.GetBoundingBox(pln, out bb);
                orientedBoxes.Add(bb);
            }

            // Sort the bounding boxes by volume
            List<Box> SortedBoundingBoxes = orientedBoxes.OrderBy(o => o.Volume).ToList();

            // Return the smallest one
            return SortedBoundingBoxes[0];
        }

        public static Plane CreateCameraPlane(Point3d target, Vector3d direction, double distance, Vector3d preferredXAxis)
        {
            direction.Unitize();

            Point3d camPos =
                target + direction * distance;

            // TCP/camera Z points away from object
            Vector3d zAxis = direction;
            zAxis.Unitize();

            // Try to keep plane X axis along the scan length
            Vector3d xAxis =
                preferredXAxis
                - zAxis * Vector3d.Multiply(preferredXAxis, zAxis);

            if (!xAxis.Unitize())
            {
                xAxis =
                    Vector3d.CrossProduct(
                        Vector3d.ZAxis,
                        zAxis);

                if (!xAxis.Unitize())
                    xAxis = Vector3d.XAxis;
            }

            // Ensure Plane.ZAxis = zAxis
            Vector3d yAxis =
                Vector3d.CrossProduct(
                    zAxis,
                    xAxis);

            yAxis.Unitize();

            xAxis =
                Vector3d.CrossProduct(
                    yAxis,
                    zAxis);

            xAxis.Unitize();

            return new Plane(
                camPos,
                -xAxis,
                -yAxis);
        }

        // ====================================================================
        // Step 1: Read defect branches
        // ====================================================================

        internal static List<RawDefectBranch> ReadRawDefectBranches(
            GH_Structure<GH_Point> defectTree)
        {
            List<RawDefectBranch> branches =
                new List<RawDefectBranch>();

            for (int i = 0; i < defectTree.PathCount; i++)
            {
                GH_Path path = defectTree.Paths[i];

                List<Point3d> pts =
                    GetPointsFromBranch(
                        defectTree,
                        i);

                if (pts.Count == 0)
                    continue;

                BoundingBox bb =
                    new BoundingBox(pts);

                if (!bb.IsValid)
                    continue;

                RawDefectBranch branch =
                    new RawDefectBranch();

                branch.Index = i;
                branch.Path = path;
                branch.Points = pts;
                branch.BoundingBox = bb;
                branch.Center = bb.Center;

                branches.Add(branch);
            }

            return branches;
        }

        private static List<Point3d> GetPointsFromBranch(
            GH_Structure<GH_Point> tree,
            int branchIndex)
        {
            List<Point3d> pts =
                new List<Point3d>();

            IList branch =
                tree.get_Branch(branchIndex);

            if (branch == null)
                return pts;

            foreach (object obj in branch)
            {
                GH_Point ghPt = obj as GH_Point;

                if (ghPt == null)
                    continue;

                Point3d p = ghPt.Value;

                if (p.IsValid)
                    pts.Add(p);
            }

            return pts;
        }

        // ====================================================================
        // Step 2: Merge nearby branches
        // ====================================================================

        internal static List<MergedRegion> MergeNearbyBranches(
            List<RawDefectBranch> branches,
            double mergeDistance)
        {
            int n = branches.Count;

            UnionFind uf =
                new UnionFind(n);

            for (int i = 0; i < n; i++)
            {
                BoundingBox a =
                    branches[i].BoundingBox;

                a.Inflate(mergeDistance);



                for (int j = i + 1; j < n; j++)
                {
                    BoundingBox b =
                        branches[j].BoundingBox;

                    b.Inflate(mergeDistance);

                    double centerDistance =
                        branches[i].Center.DistanceTo(
                            branches[j].Center);

                    double diagonalA =
                        branches[i].BoundingBox.Diagonal.Length;

                    double diagonalB =
                        branches[j].BoundingBox.Diagonal.Length;

                    double threshold =
                        mergeDistance +
                        0.5 * Math.Min(diagonalA, diagonalB);

                    if (BoundingBoxesOverlap(a, b) &&
                        centerDistance < threshold)
                    {
                        uf.Union(i, j);
                    }
                }
            }

            Dictionary<int, MergedRegion> regionMap =
                new Dictionary<int, MergedRegion>();

            for (int i = 0; i < n; i++)
            {
                int root =
                    uf.Find(i);

                if (!regionMap.ContainsKey(root))
                {
                    regionMap[root] =
                        new MergedRegion();
                }

                regionMap[root].RawBranchIndices.Add(
                    branches[i].Index);

                regionMap[root].Points.AddRange(
                    branches[i].Points);
            }

            return regionMap.Values.ToList();
        }

        private static bool BoundingBoxesOverlap(
            BoundingBox a,
            BoundingBox b)
        {
            if (a.Max.X < b.Min.X || a.Min.X > b.Max.X)
                return false;

            if (a.Max.Y < b.Min.Y || a.Min.Y > b.Max.Y)
                return false;

            if (a.Max.Z < b.Min.Z || a.Min.Z > b.Max.Z)
                return false;

            return true;
        }

        private class UnionFind
        {
            private int[] parent;
            private int[] rank;

            public UnionFind(int count)
            {
                parent = new int[count];
                rank = new int[count];

                for (int i = 0; i < count; i++)
                {
                    parent[i] = i;
                    rank[i] = 0;
                }
            }

            public int Find(int x)
            {
                if (parent[x] != x)
                    parent[x] = Find(parent[x]);

                return parent[x];
            }

            public void Union(int a, int b)
            {
                int rootA = Find(a);
                int rootB = Find(b);

                if (rootA == rootB)
                    return;

                if (rank[rootA] < rank[rootB])
                {
                    parent[rootA] = rootB;
                }
                else if (rank[rootA] > rank[rootB])
                {
                    parent[rootB] = rootA;
                }
                else
                {
                    parent[rootB] = rootA;
                    rank[rootA]++;
                }
            }
        }

        internal static List<Point3d> RemoveDuplicatePoints(
            List<Point3d> pts,
            double tolerance)
        {
            List<Point3d> unique =
                new List<Point3d>();

            double tol2 =
                tolerance * tolerance;

            foreach (Point3d p in pts)
            {
                bool exists = false;

                foreach (Point3d q in unique)
                {
                    if (p.DistanceToSquared(q) <= tol2)
                    {
                        exists = true;
                        break;
                    }
                }

                if (!exists)
                    unique.Add(p);
            }

            return unique;
        }

        internal static bool CreateHullMeshMIConvexHull(
            List<Point3d> points,
            out Mesh hullMesh,
            out string debug)
        {
            hullMesh = new Mesh();
            debug = "";

            if (points == null || points.Count < 4)
            {
                debug = "Not enough points for 3D hull.";
                return false;
            }

            double tol =
                RhinoDoc.ActiveDoc != null
                ? RhinoDoc.ActiveDoc.ModelAbsoluteTolerance
                : 0.001;

            List<Point3d> cleanPoints =
                RemoveDuplicatePoints(
                    points,
                    tol);

            if (cleanPoints.Count < 4)
            {
                debug =
                    "After duplicate removal, fewer than 4 points remain. " +
                    "Input count = " + points.Count +
                    ", clean count = " + cleanPoints.Count;

                return false;
            }

            BoundingBox bb =
                new BoundingBox(cleanPoints);

            double dx = bb.Max.X - bb.Min.X;
            double dy = bb.Max.Y - bb.Min.Y;
            double dz = bb.Max.Z - bb.Min.Z;

            debug =
                "Input points = " + points.Count +
                ", clean points = " + cleanPoints.Count +
                ", bbox size = " +
                dx.ToString("F3") + ", " +
                dy.ToString("F3") + ", " +
                dz.ToString("F3");

            List<HullVertex> verts =
                cleanPoints
                .Select((p, i) => new HullVertex(p, i))
                .ToList();

            ConvexHullCreationResult<
                HullVertex,
                DefaultConvexFace<HullVertex>> hullResult;

            try
            {
                hullResult =
                    ConvexHull.Create<
                        HullVertex,
                        DefaultConvexFace<HullVertex>>(
                            verts,
                            1e-10);
            }
            catch (Exception ex)
            {
                debug +=
                    " | MIConvexHull exception: " +
                    ex.Message;

                return false;
            }

            if (hullResult == null || hullResult.Result == null)
            {
                debug +=
                    " | Hull result is null.";

                return false;
            }

            var hull =
                hullResult.Result;

            if (hull.Faces == null)
            {
                debug +=
                    " | Hull faces are null.";

                return false;
            }

            Dictionary<int, int> vertexIndexMap =
                new Dictionary<int, int>();

            int faceCount = 0;

            foreach (var face in hull.Faces)
            {
                if (face == null || face.Vertices == null)
                    continue;

                HullVertex[] fv =
                    face.Vertices;

                if (fv.Length != 3)
                    continue;

                int[] meshIds =
                    new int[3];

                for (int i = 0; i < 3; i++)
                {
                    int originalId =
                        fv[i].Index;

                    if (!vertexIndexMap.ContainsKey(originalId))
                    {
                        Point3d p =
                            cleanPoints[originalId];

                        int newMeshId =
                            hullMesh.Vertices.Add(p);

                        vertexIndexMap[originalId] =
                            newMeshId;
                    }

                    meshIds[i] =
                        vertexIndexMap[originalId];
                }

                if (meshIds[0] == meshIds[1] ||
                    meshIds[1] == meshIds[2] ||
                    meshIds[2] == meshIds[0])
                {
                    continue;
                }

                hullMesh.Faces.AddFace(
                    meshIds[0],
                    meshIds[1],
                    meshIds[2]);

                faceCount++;
            }

            hullMesh.Vertices.CombineIdentical(true, true);
            hullMesh.Vertices.CullUnused();

            hullMesh.FaceNormals.ComputeFaceNormals();
            hullMesh.Normals.ComputeNormals();
            hullMesh.UnifyNormals();
            hullMesh.Compact();

            debug +=
                " | hull vertices = " + hullMesh.Vertices.Count +
                ", hull faces = " + hullMesh.Faces.Count;

            if (!hullMesh.IsValid || hullMesh.Faces.Count == 0)
            {
                debug +=
                    " | Created hull mesh is invalid or empty.";

                return false;
            }

            if (hullMesh.Faces.Count <= 1)
            {
                debug +=
                    " | WARNING: Hull has only one face. Input is probably coplanar, collinear, collapsed, or only one face was returned by MIConvexHull.";

                return false;
            }

            return true;
        }

        internal static List<Point3d> DownsamplePointsEvenly(
            List<Point3d> pts,
            int maxCount)
        {
            if (pts == null)
                return new List<Point3d>();

            if (pts.Count <= maxCount)
                return new List<Point3d>(pts);

            List<Point3d> result =
                new List<Point3d>();

            double step =
                (double)(pts.Count - 1) /
                (double)(maxCount - 1);

            for (int i = 0; i < maxCount; i++)
            {
                int id =
                    (int)Math.Round(i * step);

                id =
                    Math.Max(
                        0,
                        Math.Min(
                            pts.Count - 1,
                            id));

                result.Add(pts[id]);
            }

            return result;
        }

        internal static bool IsPointSetPlanar(
            List<Point3d> pts,
            double planarThreshold,
            out double maxDistance)
        {
            maxDistance = 0.0;

            if (pts == null || pts.Count < 3)
                return true;

            Plane plane;

            PlaneFitResult fit =
                Plane.FitPlaneToPoints(
                    pts,
                    out plane);

            if (fit != PlaneFitResult.Success)
                return false;

            foreach (Point3d p in pts)
            {
                double d =
                    Math.Abs(
                        plane.DistanceTo(p));

                if (d > maxDistance)
                    maxDistance = d;
            }

            return maxDistance <= planarThreshold;
        }

        internal static bool TryCreatePlanarDefectBox(
            List<Point3d> pts,
            out Box box)
        {
            box = Box.Unset;

            if (pts == null || pts.Count < 3)
                return false;

            Plane plane;

            PlaneFitResult fit =
                Plane.FitPlaneToPoints(
                    pts,
                    out plane);

            if (fit != PlaneFitResult.Success)
                return false;

            // Make plane normal stable if possible.
            if (plane.ZAxis * Vector3d.ZAxis < 0.0)
                plane.Flip();

            double minX = double.MaxValue;
            double maxX = double.MinValue;

            double minY = double.MaxValue;
            double maxY = double.MinValue;

            double minZ = double.MaxValue;
            double maxZ = double.MinValue;

            foreach (Point3d p in pts)
            {
                Vector3d q =
                    p - plane.Origin;

                double x =
                    q * plane.XAxis;

                double y =
                    q * plane.YAxis;

                double z =
                    q * plane.ZAxis;

                if (x < minX) minX = x;
                if (x > maxX) maxX = x;

                if (y < minY) minY = y;
                if (y > maxY) maxY = y;

                if (z < minZ) minZ = z;
                if (z > maxZ) maxZ = z;
            }

            double tol =
                RhinoDoc.ActiveDoc != null
                ? RhinoDoc.ActiveDoc.ModelAbsoluteTolerance
                : 0.001;

            // Give the planar box a tiny thickness so Rhino Box remains valid.
            if (Math.Abs(maxZ - minZ) < tol)
            {
                minZ = -tol;
                maxZ = tol;
            }

            if (Math.Abs(maxX - minX) < tol ||
                Math.Abs(maxY - minY) < tol)
            {
                return false;
            }

            box =
                new Box(
                    plane,
                    new Interval(minX, maxX),
                    new Interval(minY, maxY),
                    new Interval(minZ, maxZ));

            return box.IsValid;
        }

        // ====================================================================
        // Pose generation helpers
        // ====================================================================

        internal static void GetBoxAxesAndSizes(
            Box box,
            out Vector3d[] axes,
            out double[] sizes)
        {
            axes = new Vector3d[3];
            sizes = new double[3];

            axes[0] = box.Plane.XAxis;
            axes[1] = box.Plane.YAxis;
            axes[2] = box.Plane.ZAxis;

            sizes[0] = Math.Abs(box.X.Length);
            sizes[1] = Math.Abs(box.Y.Length);
            sizes[2] = Math.Abs(box.Z.Length);
        }

        internal static void GetScanAxisIds(
            double[] sizes,
            out int uId,
            out int vId,
            out int nId)
        {
            int[] ids =
                Enumerable.Range(0, 3)
                .OrderByDescending(i => sizes[i])
                .ToArray();

            uId = ids[0];
            vId = ids[1];
            nId = ids[2];
        }

        internal static bool ProjectPointToMeshSurface(
            Mesh mesh,
            Point3d approximatePoint,
            Vector3d preferredNormal,
            out Point3d surfacePoint,
            out Vector3d surfaceNormal)
        {
            surfacePoint = Point3d.Unset;
            surfaceNormal = Vector3d.Unset;

            if (mesh == null || !mesh.IsValid)
                return false;

            MeshPoint mp =
                mesh.ClosestMeshPoint(
                    approximatePoint,
                    double.MaxValue);

            if (mp == null)
                return false;

            surfacePoint =
                mp.Point;

            int faceId =
                mp.FaceIndex;

            if (faceId >= 0 && faceId < mesh.Faces.Count)
            {
                mesh.FaceNormals.ComputeFaceNormals();

                surfaceNormal =
                    mesh.FaceNormals[faceId];

                surfaceNormal.Unitize();

                if (surfaceNormal * preferredNormal < 0.0)
                    surfaceNormal = -surfaceNormal;
            }
            else
            {
                surfaceNormal =
                    preferredNormal;
            }

            return true;
        }




    }
}
