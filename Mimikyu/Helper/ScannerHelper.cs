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
        public enum Sides
        {
            posX = 0,
            posY = 1,
            negX = 2,
            negY = 3,
            posZ = 4,
            negZ = 5
        }

        public class ScanObject
        {
            public Mesh Mesh;
            public Box BoundingBox;
            public Plane ObjectPlane;

            public Point3d Center;

            public double SizeX;
            public double SizeY;
            public double SizeZ;

            public Dictionary<int, ScanFace> Faces =
                new Dictionary<int, ScanFace>();

            public static ScanObject FromBrep(Brep brep)
            {
                Mesh mesh =
                    BrepToSingleMesh(brep);

                Box box =
                    GetMinimumBoundingBox3D(mesh);

                return FromMeshAndBox(
                    mesh,
                    box);
            }

            public static ScanObject FromMeshAndBox(
                Mesh mesh,
                Box box)
            {
                if (mesh == null || !mesh.IsValid)
                    throw new Exception("Cannot create ScanObject: mesh is null or invalid.");

                if (!box.IsValid)
                    throw new Exception("Cannot create ScanObject: box is invalid.");

                mesh.FaceNormals.ComputeFaceNormals();
                mesh.Normals.ComputeNormals();
                mesh.Compact();

                ScanObject obj =
                    new ScanObject();

                obj.Mesh =
                    mesh;

                obj.BoundingBox =
                    box;

                obj.ObjectPlane =
                    box.Plane;

                obj.Center =
                    box.Center;

                obj.SizeX =
                    Math.Abs(box.X.Length);

                obj.SizeY =
                    Math.Abs(box.Y.Length);

                obj.SizeZ =
                    Math.Abs(box.Z.Length);

                obj.BuildFaces();

                return obj;
            }

            private void BuildFaces()
            {
                Faces.Clear();

                Vector3d x =
                    ObjectPlane.XAxis;

                Vector3d y =
                    ObjectPlane.YAxis;

                Vector3d z =
                    ObjectPlane.ZAxis;

                x.Unitize();
                y.Unitize();
                z.Unitize();

                Point3d posXCenter =
                    BoundingBox.PointAt(
                        1.0,
                        0.5,
                        0.5);

                Point3d negXCenter =
                    BoundingBox.PointAt(
                        0.0,
                        0.5,
                        0.5);

                Point3d posYCenter =
                    BoundingBox.PointAt(
                        0.5,
                        1.0,
                        0.5);

                Point3d negYCenter =
                    BoundingBox.PointAt(
                        0.5,
                        0.0,
                        0.5);

                Point3d posZCenter =
                    BoundingBox.PointAt(
                        0.5,
                        0.5,
                        1.0);

                Point3d negZCenter =
                    BoundingBox.PointAt(
                        0.5,
                        0.5,
                        0.0);

                // RIGHT / posX
                Faces[(int)Sides.posX] =
                    new ScanFace()
                    {
                        Center = posXCenter,
                        Normal = x,
                        UAxis = y,
                        VAxis = z,
                        Width = SizeY,
                        Height = SizeZ
                    };

                // LEFT / negX
                Faces[(int)Sides.negX] =
                    new ScanFace()
                    {
                        Center = negXCenter,
                        Normal = -x,
                        UAxis = -y,
                        VAxis = z,
                        Width = SizeY,
                        Height = SizeZ
                    };

                // BACK / posY
                Faces[(int)Sides.posY] =
                    new ScanFace()
                    {
                        Center = posYCenter,
                        Normal = y,
                        UAxis = -x,
                        VAxis = z,
                        Width = SizeX,
                        Height = SizeZ
                    };

                // FRONT / negY
                Faces[(int)Sides.negY] =
                    new ScanFace()
                    {
                        Center = negYCenter,
                        Normal = -y,
                        UAxis = x,
                        VAxis = z,
                        Width = SizeX,
                        Height = SizeZ
                    };

                // TOP / posZ
                Faces[(int)Sides.posZ] =
                    new ScanFace()
                    {
                        Center = posZCenter,
                        Normal = z,
                        UAxis = -x,
                        VAxis = y,
                        Width = SizeX,
                        Height = SizeY
                    };

                // BOTTOM / negZ
                Faces[(int)Sides.negZ] =
                    new ScanFace()
                    {
                        Center = negZCenter,
                        Normal = -z,
                        UAxis = x,
                        VAxis = -y,
                        Width = SizeX,
                        Height = SizeY
                    };
            }

            public ScanFace GetFace(int sideId)
            {
                if (Faces.ContainsKey(sideId))
                    return Faces[sideId];

                return Faces[(int)Sides.posZ];
            }

            public Vector3d GetSideNormal(int sideId)
            {
                Vector3d n =
                    GetFace(sideId).Normal;

                n.Unitize();

                return n;
            }

            public Vector3d GetSideUAxis(int sideId)
            {
                Vector3d u =
                    GetFace(sideId).UAxis;

                u.Unitize();

                return u;
            }

            public string SideIdToKey(int sideId)
            {
                switch (sideId)
                {
                    case 0: return "posX";
                    case 1: return "posY";
                    case 2: return "negX";
                    case 3: return "negY";
                    case 4: return "posZ";
                    case 5: return "negZ";
                }

                return "Unknown";
            }

            public int SideKeyToId(string key)
            {
                switch (key)
                {
                    case "posX": return (int)Sides.posX;
                    case "posY": return (int)Sides.posY;
                    case "negX": return (int)Sides.negX;
                    case "negY": return (int)Sides.negY;
                    case "posZ": return (int)Sides.posZ;
                    case "negZ": return (int)Sides.negZ;
                }

                return -1;
            }

            public int ClassifyPointToSideId(Point3d p)
            {
                Vector3d q =
                    p - ObjectPlane.Origin;

                Vector3d xAxis =
                    ObjectPlane.XAxis;

                Vector3d yAxis =
                    ObjectPlane.YAxis;

                Vector3d zAxis =
                    ObjectPlane.ZAxis;

                xAxis.Unitize();
                yAxis.Unitize();
                zAxis.Unitize();

                double x =
                    q * xAxis;

                double y =
                    q * yAxis;

                double z =
                    q * zAxis;

                double dPosX =
                    Math.Abs(x - BoundingBox.X.Max);

                double dNegX =
                    Math.Abs(x - BoundingBox.X.Min);

                double dPosY =
                    Math.Abs(y - BoundingBox.Y.Max);

                double dNegY =
                    Math.Abs(y - BoundingBox.Y.Min);

                double dPosZ =
                    Math.Abs(z - BoundingBox.Z.Max);

                double dNegZ =
                    Math.Abs(z - BoundingBox.Z.Min);

                double best =
                    dPosX;

                int side =
                    (int)Sides.posX;

                if (dPosY < best)
                {
                    best = dPosY;
                    side = (int)Sides.posY;
                }

                if (dNegX < best)
                {
                    best = dNegX;
                    side = (int)Sides.negX;
                }

                if (dNegY < best)
                {
                    best = dNegY;
                    side = (int)Sides.negY;
                }

                if (dPosZ < best)
                {
                    best = dPosZ;
                    side = (int)Sides.posZ;
                }

                if (dNegZ < best)
                {
                    best = dNegZ;
                    side = (int)Sides.negZ;
                }

                return side;
            }

            // This preserves your original GH_ObjectPose image/pose order:
            // 0 front, 1 back, 2 right, 3 left, 4 top, 5 bottom.
            public List<ScanFace> GetObjectPoseFacesInOriginalOrder()
            {
                return new List<ScanFace>()
                {
                    GetFace((int)Sides.negY), // front
                    GetFace((int)Sides.posY), // back
                    GetFace((int)Sides.posX), // right
                    GetFace((int)Sides.negX), // left
                    GetFace((int)Sides.posZ), // top
                    GetFace((int)Sides.negZ)  // bottom
                };
            }
        }

        internal class MergedRegion
        {
            public List<int> RawBranchIndices = new List<int>();

            public HashSet<int> SideIds = new HashSet<int>();

            public List<Point3d> Points = new List<Point3d>();

            public Dictionary<int, List<Point3d>> PointsBySide = new Dictionary<int, List<Point3d>>();
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
        public enum RegionType
        {
            Surface,
            Edge,
            Corner
        }
        internal static Vector3d SideToNormal(int side)
        {
            switch (side)
            {
                case 0: return Vector3d.XAxis;
                case 1: return Vector3d.YAxis;
                case 2: return -Vector3d.XAxis;
                case 3: return -Vector3d.YAxis;
                case 4: return Vector3d.ZAxis;
                case 5: return -Vector3d.ZAxis;
            }

            return Vector3d.ZAxis;
        }

        internal static RegionType GetRegionType(MergedRegion region)
        {
            int n = region.SideIds.Count;

            if (n <= 1)
                return RegionType.Surface;

            if (n == 2)
                return RegionType.Edge;

            return RegionType.Corner;
        }
        internal static List<Vector3d> GetScanDirections(
    MergedRegion region,
    ScanObject scanObject)
        {
            List<Vector3d> dirs =
                new List<Vector3d>();

            if (region == null ||
                scanObject == null ||
                region.SideIds == null ||
                region.SideIds.Count == 0)
            {
                dirs.Add(Vector3d.ZAxis);
                return dirs;
            }

            RegionType type =
                GetRegionType(region);

            List<int> sideIds =
                region.SideIds.ToList();

            List<Vector3d> sideNormals =
                new List<Vector3d>();

            foreach (int sideId in sideIds)
            {
                Vector3d n =
                    scanObject.GetSideNormal(sideId);

                if (n.Unitize())
                    sideNormals.Add(n);
            }

            if (sideNormals.Count == 0)
            {
                dirs.Add(Vector3d.ZAxis);
                return dirs;
            }

            if (type == RegionType.Surface)
            {
                dirs.Add(sideNormals[0]);
                return dirs;
            }

            if (type == RegionType.Edge &&
                sideNormals.Count >= 2)
            {
                Vector3d edgeDir =
                    sideNormals[0] +
                    sideNormals[1];

                if (edgeDir.Unitize())
                    dirs.Add(edgeDir);

                dirs.Add(sideNormals[0]);
                dirs.Add(sideNormals[1]);

                return dirs;
            }

            Vector3d cornerDir =
                Vector3d.Zero;

            foreach (Vector3d n in sideNormals)
                cornerDir += n;

            if (cornerDir.Unitize())
                dirs.Add(cornerDir);

            for (int i = 0;
                 i < Math.Min(2, sideNormals.Count);
                 i++)
            {
                dirs.Add(sideNormals[i]);
            }

            return dirs;
        }

        internal static List<Vector3d> CreateCrackViews(Vector3d mainDirection, Vector3d rotationAxis, double angleDeg)
        {
            List<Vector3d> dirs =
                new List<Vector3d>();

            if (!mainDirection.Unitize())
                return dirs;

            if (!rotationAxis.Unitize())
            {
                rotationAxis =
                    Vector3d.CrossProduct(
                        Vector3d.ZAxis,
                        mainDirection);

                if (!rotationAxis.Unitize())
                    rotationAxis = Vector3d.XAxis;
            }

            dirs.Add(mainDirection);

            Transform rotPlus =
                Transform.Rotation(
                    RhinoMath.ToRadians(angleDeg),
                    rotationAxis,
                    Point3d.Origin);

            Transform rotMinus =
                Transform.Rotation(
                    RhinoMath.ToRadians(-angleDeg),
                    rotationAxis,
                    Point3d.Origin);

            Vector3d plus =
                mainDirection;

            plus.Transform(rotPlus);

            if (plus.Unitize())
                dirs.Add(plus);

            Vector3d minus =
                mainDirection;

            minus.Transform(rotMinus);

            if (minus.Unitize())
                dirs.Add(minus);

            return dirs;
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
                xAxis,
                yAxis);
        }

        internal class RawDefectBranch
        {
            public int Index;
            public GH_Path Path;

            public int PieceId;
            public int PoseId;
            public int SideId;
            public int DefectId;

            public List<Point3d> Points;
            public BoundingBox BoundingBox;
            public Point3d Center;
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
                branch.PieceId = path.Indices[0];
                branch.PoseId = path.Indices[1];
                branch.SideId = path.Indices[2];
                branch.DefectId = path.Indices[3];

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

                regionMap[root].RawBranchIndices.Add(branches[i].Index);

                regionMap[root].Points.AddRange(branches[i].Points);

                regionMap[root].SideIds.Add(branches[i].SideId);

                if (!regionMap[root].PointsBySide.ContainsKey(branches[i].SideId))
                {
                    regionMap[root].PointsBySide[branches[i].SideId] =
                        new List<Point3d>();
                }

                regionMap[root].PointsBySide[branches[i].SideId].AddRange(
                    branches[i].Points);
            }

            return regionMap.Values.ToList();
        }
        internal static Vector3d AverageMeshNormalFromPoints(
    Mesh mesh,
    List<Point3d> pts,
    Vector3d fallbackNormal)
        {
            if (mesh == null || !mesh.IsValid || pts == null || pts.Count == 0)
                return fallbackNormal;

            mesh.FaceNormals.ComputeFaceNormals();

            Vector3d sum =
                Vector3d.Zero;

            int count = 0;

            foreach (Point3d p in pts)
            {
                MeshPoint mp =
                    mesh.ClosestMeshPoint(
                        p,
                        double.MaxValue);

                if (mp == null)
                    continue;

                int faceId =
                    mp.FaceIndex;

                if (faceId < 0 || faceId >= mesh.FaceNormals.Count)
                    continue;

                Vector3d n =
                    mesh.FaceNormals[faceId];

                if (!n.Unitize())
                    continue;

                // Keep normals consistently oriented before averaging.
                if (count > 0 && sum * n < 0.0)
                    n = -n;

                sum += n;
                count++;
            }

            if (count == 0 || !sum.Unitize())
                return fallbackNormal;

            return sum;
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
