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


namespace Mimikyu.Helper
{
    internal class PixelHelper
    {

        // =====================================================
        // JSON DATA CLASSES
        // =====================================================

        public class CameraIntrinsics
        {
            public double fx { get; set; }
            public double fy { get; set; }
            public double cx { get; set; }
            public double cy { get; set; }

            public int image_width { get; set; }
            public int image_height { get; set; }

            public List<double> distortion_coefficients { get; set; }
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
                    projectedPoints.Add(hit.Value);
                }
            }

            return projectedPoints;
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
                (u - K.cx) / K.fx;

            double y =
                (v - K.cy) / K.fy;

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
