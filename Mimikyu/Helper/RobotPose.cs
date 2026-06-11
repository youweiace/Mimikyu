using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using System.Text.RegularExpressions;
using System.Threading.Tasks;


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

    internal static class RobotPoseReader
    {
        // Matches numbers like -123.456
        private static readonly Regex NumberRegex = new Regex(
            @"[-+]?\d*\.?\d+",
            RegexOptions.Compiled
        );

        public static RobotPose FromTextLine(string line)
        {
            if (string.IsNullOrWhiteSpace(line))
                throw new ArgumentException("Line is empty");

            var matches = NumberRegex.Matches(line);

            // We expect:
            // 10 numbers (X,Y,Z,A,B,C,E1,E2,E3,E4)
            // or 10 + timestamp (ignored here unless you want to parse it separately)
            if (matches.Count < 10)
                throw new FormatException("Invalid pose format");

            double[] values = matches
                .Cast<Match>()
                .Take(10)
                .Select(m => double.Parse(m.Value, CultureInfo.InvariantCulture))
                .ToArray();

            var pose = new RobotPose
            {
                X = values[0],
                Y = values[1],
                Z = values[2],
                A = values[3],
                B = values[4],
                C = values[5],
                E1 = values[6],
                E2 = values[7],
                E3 = values[8],
                E4 = values[9]
            };

            // Optional: parse timestamp if present
            if (line.Contains("Timestamp"))
            {
                var tsMatch = Regex.Match(line, @"Timestamp\s+([^\}]+)");
                if (tsMatch.Success &&
                    DateTime.TryParse(tsMatch.Groups[1].Value,
                    CultureInfo.InvariantCulture,
                    DateTimeStyles.None,
                    out DateTime dt))
                {
                    pose.Timestamp = dt;
                }
            }

            return pose;
        }

        public static RobotPose ReadOne(string filePath)
        {
            var line = File.ReadAllText(filePath);
            return FromTextLine(line);
        }

        public static List<RobotPose> ReadMany(string filePath)
        {
            return File.ReadAllLines(filePath)
                       .Where(l => !string.IsNullOrWhiteSpace(l))
                       .Select(FromTextLine)
                       .ToList();
        }
    }


    public static class TransformHelper
    {
        public static Transform MultiplyTransform(Transform A, Transform B)
        {
            Transform C = Transform.Identity;

            C.M00 = A.M00 * B.M00 + A.M01 * B.M10 + A.M02 * B.M20 + A.M03 * B.M30;
            C.M01 = A.M00 * B.M01 + A.M01 * B.M11 + A.M02 * B.M21 + A.M03 * B.M31;
            C.M02 = A.M00 * B.M02 + A.M01 * B.M12 + A.M02 * B.M22 + A.M03 * B.M32;
            C.M03 = A.M00 * B.M03 + A.M01 * B.M13 + A.M02 * B.M23 + A.M03 * B.M33;

            C.M10 = A.M10 * B.M00 + A.M11 * B.M10 + A.M12 * B.M20 + A.M13 * B.M30;
            C.M11 = A.M10 * B.M01 + A.M11 * B.M11 + A.M12 * B.M21 + A.M13 * B.M31;
            C.M12 = A.M10 * B.M02 + A.M11 * B.M12 + A.M12 * B.M22 + A.M13 * B.M32;
            C.M13 = A.M10 * B.M03 + A.M11 * B.M13 + A.M12 * B.M23 + A.M13 * B.M33;

            C.M20 = A.M20 * B.M00 + A.M21 * B.M10 + A.M22 * B.M20 + A.M23 * B.M30;
            C.M21 = A.M20 * B.M01 + A.M21 * B.M11 + A.M22 * B.M21 + A.M23 * B.M31;
            C.M22 = A.M20 * B.M02 + A.M21 * B.M12 + A.M22 * B.M22 + A.M23 * B.M32;
            C.M23 = A.M20 * B.M03 + A.M21 * B.M13 + A.M22 * B.M23 + A.M23 * B.M33;

            C.M30 = A.M30 * B.M00 + A.M31 * B.M10 + A.M32 * B.M20 + A.M33 * B.M30;
            C.M31 = A.M30 * B.M01 + A.M31 * B.M11 + A.M32 * B.M21 + A.M33 * B.M31;
            C.M32 = A.M30 * B.M02 + A.M31 * B.M12 + A.M32 * B.M22 + A.M33 * B.M32;
            C.M33 = A.M30 * B.M03 + A.M31 * B.M13 + A.M32 * B.M23 + A.M33 * B.M33;

            return C;
        }
    }
}
