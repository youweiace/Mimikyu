using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.IO;


namespace Mimikyu.Helper
{
    internal class RobotPose
    {
        public double X { get; set; }
        public double Y { get; set; }
        public double Z { get; set; }
        public double A { get; set; }
        public double B { get; set; }
        public double C { get; set; }
        public double E1 { get; set; }
        public double E2 { get; set; }
        public double E3 { get; set; }
        public double E4 { get; set; }
        public DateTime Timestamp { get; set; }
    }


    internal static class RobotPoseWriter
    {
        public static string ToTextLine(RobotPose pose, bool includeTimestamp = false)
        {
            if (includeTimestamp)
            {
                return string.Format(
                    CultureInfo.InvariantCulture,
                    "{{X {0:F3}, Y {1:F3}, Z {2:F3}, A {3:F3}, B {4:F3}, C {5:F3}, E1 {6:F3}, E2 {7:F3}, E3 {8:F3}, E4 {9:F3}, Timestamp {10:yyyy-MM-dd HH:mm:ss.fff}}}",
                    pose.X, pose.Y, pose.Z,
                    pose.A, pose.B, pose.C,
                    pose.E1, pose.E2, pose.E3, pose.E4,
                    pose.Timestamp
                );
            }

            return string.Format(
                CultureInfo.InvariantCulture,
                "LIN: X {0:F2}, Y {1:F2}, Z {2:F2}, A {3:F2}, B {4:F2}, C {5:F2}, E1 {6:F2}, E2 {7:F2}, E3 {8:F2}, E4 {9:F2}",
                pose.X,
                pose.Y,
                pose.Z,
                pose.A,
                pose.B,
                pose.C,
                pose.E1,
                pose.E2,
                pose.E3,
                pose.E4
            );
        }

        public static void SaveOne(RobotPose pose, string filePath, bool includeTimestamp = false)
        {
            File.WriteAllText(filePath, ToTextLine(pose, includeTimestamp));
        }

        public static void SaveMany(IEnumerable<RobotPose> poses, string filePath, bool includeTimestamp = false)
        {
            File.WriteAllLines(filePath, poses.Select(p => ToTextLine(p, includeTimestamp)));
        }
    }
}
