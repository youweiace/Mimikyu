using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Text;
using System.Text.RegularExpressions;
using System.Threading.Tasks;
using System.IO;
using Newtonsoft.Json;
using Rhino.Geometry.Intersect;

namespace Mimikyu.Helper
{
    internal class PixelHelper
    {

        // =====================================================
        // JSON DATA CLASSES
        // =====================================================

        public class CameraIntrinsics
        {
            public int image_width { get; set; }
            public int image_height { get; set; }

            public CameraMatrix camera_matrix { get; set; }
        }

        public class CameraMatrix
        {
            public double fx { get; set; }
            public double fy { get; set; }
            public double cx { get; set; }
            public double cy { get; set; }
        }

        public class CameraToRobotJson
        {
            public string transform_name { get; set; }
            public string definition { get; set; }
            public string units { get; set; }

            public Translation translation_mm { get; set; }

            public double[][] rotation_matrix { get; set; }

            public double[][] homogeneous_matrix { get; set; }

            public string method { get; set; }
            public int used_image_count { get; set; }
            public List<string> used_images { get; set; }
            public string notes { get; set; }
        }

        public class Translation
        {
            public double x { get; set; }
            public double y { get; set; }
            public double z { get; set; }
        }

        public class KukaPose
        {
            public double X;
            public double Y;
            public double Z;
            public double A;
            public double B;
            public double C;
        }

        public class DefectContainer
        {
            public List<Defect> defects { get; set; }
        }

        public class Defect
        {
            public int id { get; set; }

            public double area_pixels { get; set; }

            public List<List<double>> contour { get; set; }
        }
        public class PixelObjectHit
        {
            public Point3d Pixel { get; set; }
            public Point3d Point { get; set; }
            public int FaceIndex { get; set; }
            public Vector3d FaceNormal { get; set; }
            public double Distance { get; set; }
            public string SideKey { get; set; }
        }

        // =====================================================
        // MAIN METHOD FOR GRASSHOPPER
        // =====================================================

        public static List<Point3d> ProjectPixelsToPlane(
            string intrinsicsJsonPath,
            string cameraToRobotJsonPath,
            string poseTxtPath,
            int poseIndex,
            List<Point3d> imagePixels,
            Plane targetPlane
        )
        {
            CameraIntrinsics K =
                LoadIntrinsics(intrinsicsJsonPath);

            double[,] T_tcp_camera =
                LoadCameraToRobot(cameraToRobotJsonPath);

            List<KukaPose> poses =
                LoadPoses(poseTxtPath);

            if (poseIndex < 0 || poseIndex >= poses.Count)
            {
                throw new Exception(
                    $"poseIndex {poseIndex} is invalid. Pose file contains {poses.Count} poses."
                );
            }

            KukaPose pose =
                poses[poseIndex];

            double[,] T_base_tcp =
                KukaPoseToTransform(pose);

            double[,] T_base_camera =
                Multiply4x4(
                    T_base_tcp,
                    T_tcp_camera
                );

            Point3d cameraOrigin =
                new Point3d(
                    T_base_camera[0, 3],
                    T_base_camera[1, 3],
                    T_base_camera[2, 3]
                );

            double[,] R_base_camera =
                ExtractRotation(T_base_camera);

            List<Point3d> projectedPoints =
                new List<Point3d>();

            foreach (Point3d pixel in imagePixels)
            {
                double u = pixel.X;
                double v = pixel.Y;

                Vector3d rayCamera =
                    PixelToRay(
                        u,
                        v,
                        K
                    );

                Vector3d rayBase =
                    Multiply3x3Vector(
                        R_base_camera,
                        rayCamera
                    );

                rayBase.Unitize();

                Point3d? hit =
                    IntersectRhinoPlane(
                        cameraOrigin,
                        rayBase,
                        targetPlane
                    );


                if (hit.HasValue)
                {
                    Point3d p = hit.Value;

                    bool valid =
                        p.IsValid &&
                        !double.IsNaN(p.X) &&
                        !double.IsNaN(p.Y) &&
                        !double.IsNaN(p.Z) &&
                        !double.IsInfinity(p.X) &&
                        !double.IsInfinity(p.Y) &&
                        !double.IsInfinity(p.Z);

                    if (valid)
                    {
                        projectedPoints.Add(p);
                    }
                    else
                    {
                        System.Diagnostics.Debug.WriteLine(
                            $"Invalid projected point: X={p.X}, Y={p.Y}, Z={p.Z}"
                        );
                    }
                }
                else
                {
                    System.Diagnostics.Debug.WriteLine(
                        $"No plane intersection for pixel: u={pixel.X}, v={pixel.Y}"
                    );
                }

            }

            return projectedPoints;
        }

        public static List<PixelObjectHit> ProjectPixelsToObjectMesh(
            string intrinsicsJsonPath,
            string cameraToRobotJsonPath,
            string poseTxtPath,
            int poseIndex,
            List<Point3d> imagePixels,
            Mesh objectMesh,
            Plane objectPlane,
            double maxDistance = 5000.0
        )
        {
            CameraIntrinsics K =
                LoadIntrinsics(intrinsicsJsonPath);

            double[,] T_tcp_camera =
                LoadCameraToRobot(cameraToRobotJsonPath);

            List<KukaPose> poses =
                LoadPoses(poseTxtPath);

            if (poseIndex < 0 || poseIndex >= poses.Count)
            {
                throw new Exception(
                    $"poseIndex {poseIndex} is invalid. Pose file contains {poses.Count} poses."
                );
            }

            if (objectMesh == null || !objectMesh.IsValid)
            {
                throw new Exception("Object mesh is null or invalid.");
            }

            objectMesh.FaceNormals.ComputeFaceNormals();
            objectMesh.Normals.ComputeNormals();

            KukaPose pose =
                poses[poseIndex];

            double[,] T_base_tcp =
                KukaPoseToTransform(pose);

            double[,] T_base_camera =
                Multiply4x4(
                    T_base_tcp,
                    T_tcp_camera
                );

            Point3d cameraOrigin =
                new Point3d(
                    T_base_camera[0, 3],
                    T_base_camera[1, 3],
                    T_base_camera[2, 3]
                );

            double[,] R_base_camera =
                ExtractRotation(T_base_camera);

            List<PixelObjectHit> hits =
                new List<PixelObjectHit>();

            foreach (Point3d pixel in imagePixels)
            {
                double u = pixel.X;
                double v = pixel.Y;

                Vector3d rayCamera =
                    PixelToRay(
                        u,
                        v,
                        K
                    );

                Vector3d rayBase =
                    Multiply3x3Vector(
                        R_base_camera,
                        rayCamera
                    );

                if (!rayBase.Unitize())
                {
                    continue;
                }

                Ray3d ray =
                    new Ray3d(
                        cameraOrigin,
                        rayBase
                    );

                int[] faceIds;
                double t =
                    Intersection.MeshRay(
                        objectMesh,
                        ray,
                        out faceIds
                    );

                if (t < 0)
                {
                    continue;
                }

                if (t > maxDistance)
                {
                    continue;
                }

                Point3d hitPoint =
                    ray.PointAt(t);

                if (!hitPoint.IsValid)
                {
                    continue;
                }

                int faceIndex =
                    -1;

                if (faceIds != null && faceIds.Length > 0)
                {
                    faceIndex = faceIds[0];
                }

                Vector3d normal =
                    Vector3d.Unset;

                if (faceIndex >= 0 && faceIndex < objectMesh.FaceNormals.Count)
                {
                    normal =
                        objectMesh.FaceNormals[faceIndex];

                    normal.Unitize();
                }

                string sideKey =
                    ClassifyNormalToObjectSide(
                        normal,
                        objectPlane
                    );

                hits.Add(
                    new PixelObjectHit
                    {
                        Pixel = pixel,
                        Point = hitPoint,
                        FaceIndex = faceIndex,
                        FaceNormal = normal,
                        Distance = t,
                        SideKey = sideKey
                    }
                );
            }

            return hits;
        }

        private static string ClassifyNormalToObjectSide(
            Vector3d normal,
            Plane objectPlane
        )
        {
            if (!normal.IsValid || normal.IsTiny())
            {
                return "Unknown";
            }

            normal.Unitize();

            Vector3d x = objectPlane.XAxis;
            Vector3d y = objectPlane.YAxis;
            Vector3d z = objectPlane.ZAxis;

            x.Unitize();
            y.Unitize();
            z.Unitize();

            double dx = Vector3d.Multiply(normal, x);
            double dy = Vector3d.Multiply(normal, y);
            double dz = Vector3d.Multiply(normal, z);

            double ax = Math.Abs(dx);
            double ay = Math.Abs(dy);
            double az = Math.Abs(dz);

            if (ax >= ay && ax >= az)
            {
                return dx >= 0 ? "posX" : "negX";
            }

            if (ay >= ax && ay >= az)
            {
                return dy >= 0 ? "posY" : "negY";
            }

            return dz >= 0 ? "posZ" : "negZ";
        }

        // =====================================================
        // LOAD JSON FILES WITH NEWTONSOFT.JSON
        // =====================================================

        private static CameraIntrinsics LoadIntrinsics(
            string path
        )
        {
            string json =
                File.ReadAllText(path);

            CameraIntrinsics K =
                JsonConvert.DeserializeObject<CameraIntrinsics>(
                    json
                );

            if (K == null)
            {
                throw new Exception(
                    "Could not read camera intrinsics JSON."
                );
            }

            return K;
        }

        private static double[,] LoadCameraToRobot(
            string path
        )
        {
            string json =
                File.ReadAllText(path);

            CameraToRobotJson data =
                JsonConvert.DeserializeObject<CameraToRobotJson>(
                    json
                );

            if (data == null)
            {
                throw new Exception(
                    "Could not read camera-to-robot JSON."
                );
            }

            if (data.homogeneous_matrix == null)
            {
                throw new Exception(
                    "JSON does not contain homogeneous_matrix."
                );
            }

            return JaggedToMatrix4x4(
                data.homogeneous_matrix
            );
        }

        private static double[,] JaggedToMatrix4x4(
            double[][] a
        )
        {
            if (a.Length != 4)
            {
                throw new Exception(
                    "homogeneous_matrix must have 4 rows."
                );
            }

            double[,] m =
                new double[4, 4];

            for (int r = 0; r < 4; r++)
            {
                if (a[r].Length != 4)
                {
                    throw new Exception(
                        "Each homogeneous_matrix row must have 4 values."
                    );
                }

                for (int c = 0; c < 4; c++)
                {
                    m[r, c] = a[r][c];
                }
            }

            return m;
        }

        public static List<List<Point3d>> LoadDefectContours(
            string jsonPath
        )
        {
            string json =
                File.ReadAllText(jsonPath);

            DefectContainer data =
                JsonConvert.DeserializeObject<DefectContainer>(
                    json
                );

            List<List<Point3d>> contours =
                new List<List<Point3d>>();

            foreach (Defect defect in data.defects)
            {
                List<Point3d> contour =
                    new List<Point3d>();

                foreach (List<double> pt in defect.contour)
                {
                    contour.Add(
                        new Point3d(
                            pt[0],
                            pt[1],
                            0
                        )
                    );
                }

                contours.Add(contour);
            }

            return contours;
        }


        // =====================================================
        // LOAD KUKA POSES
        // =====================================================

        private static List<KukaPose> LoadPoses(
            string path
        )
        {
            List<KukaPose> poses =
                new List<KukaPose>();

            string[] lines =
                File.ReadAllLines(path);

            foreach (string raw in lines)
            {
                string line =
                    raw.Trim();

                if (string.IsNullOrWhiteSpace(line))
                {
                    continue;
                }

                poses.Add(
                    ParseKukaPose(line)
                );
            }

            return poses;
        }

        private static KukaPose ParseKukaPose(
            string line
        )
        {
            KukaPose p =
                new KukaPose();

            p.X = FindPoseValue(line, "X");
            p.Y = FindPoseValue(line, "Y");
            p.Z = FindPoseValue(line, "Z");
            p.A = FindPoseValue(line, "A");
            p.B = FindPoseValue(line, "B");
            p.C = FindPoseValue(line, "C");

            return p;
        }

        private static double FindPoseValue(
            string line,
            string key
        )
        {
            Match m =
                Regex.Match(
                    line,
                    key + @"\s+(-?\d+\.?\d*)",
                    RegexOptions.IgnoreCase
                );

            if (!m.Success)
            {
                throw new Exception(
                    $"Could not find {key} in line:\n{line}"
                );
            }

            return double.Parse(
                m.Groups[1].Value,
                CultureInfo.InvariantCulture
            );
        }

        // =====================================================
        // KUKA POSE TO TRANSFORM
        // Same as Python:
        // R = Rz(A) * Ry(B) * Rx(C)
        // =====================================================

        private static double[,] KukaPoseToTransform(
            KukaPose p
        )
        {
            double[,] R =
                Multiply3x3(
                    Multiply3x3(
                        Rz(p.A),
                        Ry(p.B)
                    ),
                    Rx(p.C)
                );

            double[,] T =
                Identity4x4();

            for (int r = 0; r < 3; r++)
            {
                for (int c = 0; c < 3; c++)
                {
                    T[r, c] = R[r, c];
                }
            }

            T[0, 3] = p.X;
            T[1, 3] = p.Y;
            T[2, 3] = p.Z;

            return T;
        }

        private static double[,] Rx(
            double deg
        )
        {
            double a =
                DegToRad(deg);

            double c =
                Math.Cos(a);

            double s =
                Math.Sin(a);

            return new double[,]
            {
            { 1, 0, 0 },
            { 0, c, -s },
            { 0, s, c }
            };
        }

        private static double[,] Ry(
            double deg
        )
        {
            double a =
                DegToRad(deg);

            double c =
                Math.Cos(a);

            double s =
                Math.Sin(a);

            return new double[,]
            {
            { c, 0, s },
            { 0, 1, 0 },
            { -s, 0, c }
            };
        }

        private static double[,] Rz(
            double deg
        )
        {
            double a =
                DegToRad(deg);

            double c =
                Math.Cos(a);

            double s =
                Math.Sin(a);

            return new double[,]
            {
            { c, -s, 0 },
            { s, c, 0 },
            { 0, 0, 1 }
            };
        }

        private static double DegToRad(
            double deg
        )
        {
            return deg * Math.PI / 180.0;
        }

        // =====================================================
        // PIXEL TO CAMERA RAY
        // =====================================================

        private static Vector3d PixelToRay(
            double u,
            double v,
            CameraIntrinsics K
        )
        {

            double x =
                (u - K.camera_matrix.cx)
                / K.camera_matrix.fx;

            double y =
                (v - K.camera_matrix.cy)
                / K.camera_matrix.fy;

            Vector3d ray =
                new Vector3d(
                    x,
                    y,
                    1.0
                );

            ray.Unitize();

            return ray;
        }

        // =====================================================
        // RAY TO RHINO PLANE
        // =====================================================

        private static Point3d? IntersectRhinoPlane(
            Point3d rayOrigin,
            Vector3d rayDirection,
            Plane plane
        )
        {
            Vector3d n =
                plane.Normal;

            n.Unitize();

            double denom =
                Vector3d.Multiply(
                    rayDirection,
                    n
                );

            if (Math.Abs(denom) < 1e-9)
            {
                return null;
            }

            Vector3d originToPlane =
                plane.Origin - rayOrigin;

            double t =
                Vector3d.Multiply(
                    originToPlane,
                    n
                ) / denom;

            if (t < 0)
            {
                return null;
            }

            return rayOrigin + t * rayDirection;
        }

        // =====================================================
        // MATRIX HELPERS
        // =====================================================

        private static double[,] Identity4x4()
        {
            double[,] T =
                new double[4, 4];

            for (int i = 0; i < 4; i++)
            {
                T[i, i] = 1.0;
            }

            return T;
        }

        private static double[,] Multiply3x3(
            double[,] A,
            double[,] B
        )
        {
            double[,] C =
                new double[3, 3];

            for (int r = 0; r < 3; r++)
            {
                for (int c = 0; c < 3; c++)
                {
                    double sum =
                        0.0;

                    for (int k = 0; k < 3; k++)
                    {
                        sum += A[r, k] * B[k, c];
                    }

                    C[r, c] = sum;
                }
            }

            return C;
        }

        private static double[,] Multiply4x4(
            double[,] A,
            double[,] B
        )
        {
            double[,] C =
                new double[4, 4];

            for (int r = 0; r < 4; r++)
            {
                for (int c = 0; c < 4; c++)
                {
                    double sum =
                        0.0;

                    for (int k = 0; k < 4; k++)
                    {
                        sum += A[r, k] * B[k, c];
                    }

                    C[r, c] = sum;
                }
            }

            return C;
        }

        private static double[,] ExtractRotation(
            double[,] T
        )
        {
            double[,] R =
                new double[3, 3];

            for (int r = 0; r < 3; r++)
            {
                for (int c = 0; c < 3; c++)
                {
                    R[r, c] = T[r, c];
                }
            }

            return R;
        }

        private static Vector3d Multiply3x3Vector(
            double[,] R,
            Vector3d v
        )
        {
            return new Vector3d(
                R[0, 0] * v.X + R[0, 1] * v.Y + R[0, 2] * v.Z,
                R[1, 0] * v.X + R[1, 1] * v.Y + R[1, 2] * v.Z,
                R[2, 0] * v.X + R[2, 1] * v.Y + R[2, 2] * v.Z
            );
        }


    }
}
