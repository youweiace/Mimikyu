using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Drawing;

namespace Mimikyu.Helper
{
    internal static class PointCloudHelper
    {
        public class VoxelAccum
        {
            public double X = 0.0;
            public double Y = 0.0;
            public double Z = 0.0;
            public double R = 0.0;
            public double G = 0.0;
            public double B = 0.0;
            public int Count = 0;
            public bool HasColor = false;
        }
        public static PointCloud DuplicatePointCloud(PointCloud pc)
        {
            PointCloud copy = new PointCloud();
            if (pc == null) return copy;

            for (int i = 0; i < pc.Count; i++)
            {
                PointCloudItem item = pc[i];
                if (item.Color.IsEmpty)
                    copy.Add(item.Location);
                else
                    copy.Add(item.Location, item.Color);
            }

            return copy;
        }

        public static PointCloud MergeTwoClouds(PointCloud a, PointCloud b)
        {
            PointCloud merged = new PointCloud();

            if (a != null)
            {
                for (int i = 0; i < a.Count; i++)
                {
                    PointCloudItem item = a[i];
                    if (item.Color.IsEmpty) merged.Add(item.Location);
                    else merged.Add(item.Location, item.Color);
                }
            }

            if (b != null)
            {
                for (int i = 0; i < b.Count; i++)
                {
                    PointCloudItem item = b[i];
                    if (item.Color.IsEmpty) merged.Add(item.Location);
                    else merged.Add(item.Location, item.Color);
                }
            }

            return merged;
        }

        public static PointCloud MergeClouds(List<PointCloud> clouds, double voxel)
        {
            PointCloud merged = new PointCloud();

            if (clouds == null) return merged;

            for (int i = 0; i < clouds.Count; i++)
            {
                PointCloud c = clouds[i];
                if (c == null) continue;

                for (int j = 0; j < c.Count; j++)
                {
                    PointCloudItem item = c[j];
                    if (item.Color.IsEmpty)
                        merged.Add(item.Location);
                    else
                        merged.Add(item.Location, item.Color);
                }
            }

            if (voxel > 0.0)
                merged = VoxelDownsample(merged, voxel);

            return merged;
        }

        public static PointCloud VoxelDownsample(PointCloud pc, double voxel)
        {
            if (pc == null) return new PointCloud();
            if (pc.Count == 0) return new PointCloud();
            if (voxel <= 0.0) return DuplicatePointCloud(pc);

            Dictionary<string, VoxelAccum> cells = new Dictionary<string, VoxelAccum>();

            for (int i = 0; i < pc.Count; i++)
            {
                PointCloudItem item = pc[i];
                Point3d p = item.Location;

                int ix = (int)Math.Floor(p.X / voxel);
                int iy = (int)Math.Floor(p.Y / voxel);
                int iz = (int)Math.Floor(p.Z / voxel);
                string key = ix.ToString() + "_" + iy.ToString() + "_" + iz.ToString();

                VoxelAccum acc;
                if (!cells.TryGetValue(key, out acc))
                {
                    acc = new VoxelAccum();
                    cells[key] = acc;
                }

                acc.X += p.X;
                acc.Y += p.Y;
                acc.Z += p.Z;
                acc.Count++;

                if (!item.Color.IsEmpty)
                {
                    acc.R += item.Color.R;
                    acc.G += item.Color.G;
                    acc.B += item.Color.B;
                    acc.HasColor = true;
                }
            }

            PointCloud down = new PointCloud();

            foreach (KeyValuePair<string, VoxelAccum> kv in cells)
            {
                VoxelAccum acc = kv.Value;
                Point3d p = new Point3d(acc.X / acc.Count, acc.Y / acc.Count, acc.Z / acc.Count);

                if (acc.HasColor)
                {
                    int r = ClampToByte(acc.R / acc.Count);
                    int g = ClampToByte(acc.G / acc.Count);
                    int b = ClampToByte(acc.B / acc.Count);
                    down.Add(p, Color.FromArgb(r, g, b));
                }
                else
                {
                    down.Add(p);
                }
            }

            return down;
        }

        public static int ClampToByte(double x)
        {
            if (x < 0.0) return 0;
            if (x > 255.0) return 255;
            return (int)Math.Round(x);
        }

        public static List<Point3d> CloudToPoints(PointCloud pc)
        {
            List<Point3d> pts = new List<Point3d>();
            for (int i = 0; i < pc.Count; i++)
                pts.Add(pc[i].Location);
            return pts;
        }

        public static int ClosestPointIndexBruteForce(Point3d p, List<Point3d> pts, double maxDistance)
        {
            double best = maxDistance * maxDistance;
            int bestIndex = -1;

            for (int i = 0; i < pts.Count; i++)
            {
                double d2 = p.DistanceToSquared(pts[i]);
                if (d2 <= best)
                {
                    best = d2;
                    bestIndex = i;
                }
            }

            return bestIndex;
        }
    }
}
