using Grasshopper.Kernel;
using Mimikyu.Helper;
using Rhino.Geometry;
using Rhino.Render;
using System;
using System.Collections.Generic;
using System.Drawing;

namespace Mimikyu.Utlilities
{
    public class GH_MergePointCloud : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the GH_MergePointCloud class.
        /// </summary>
        public GH_MergePointCloud()
          : base("MergePointCloud", "Nickname",
              "Description",
              "Category", "Subcategory")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddBooleanParameter("Run", "R", "Trigger to run the merging and evaluation.", GH_ParamAccess.item);
            pManager.AddGenericParameter("WorldScans", "W", "The list of world-based scans (before ICP).", GH_ParamAccess.list);
            pManager.AddGenericParameter("ICPScans", "I", "The list of ICP-refined scans (after ICP).", GH_ParamAccess.list);
            pManager.AddGenericParameter("ICPValues", "V", "ICP vlaues used for iterations", GH_ParamAccess.item);
            pManager.AddNumberParameter("MergeVoxel", "M", "The voxel size for merging (0 to disable).", GH_ParamAccess.item, 0.5);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("MergedRobot", "MR", "The merged point cloud from robot-based scans.", GH_ParamAccess.item);
            pManager.AddGenericParameter("MergedICP", "MI", "The merged point cloud from ICP-refined scans.", GH_ParamAccess.item);
            pManager.AddTextParameter("Report", "Rep", "The evaluation report.", GH_ParamAccess.item);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {

            bool run = false;
            List<PointCloud> worldScans = new List<PointCloud>();
            List<PointCloud> icpScans = new List<PointCloud>();
            IcpValues icpValues = null;
            double mergeVoxel = 0.0;

            if (!DA.GetData(0, ref run)) return;
            if (!DA.GetDataList(1, worldScans)) return;
            if (!DA.GetDataList(2, icpScans)) return;
            if (!DA.GetData(3, ref icpValues)) return;
            if(!DA.GetData(4, ref mergeVoxel)) return;

            PointCloud mergedWorld = new PointCloud();
            PointCloud mergedRefined = new PointCloud();
            List<string> report = default;

            double voxelSize = icpValues.VoxelSize;
            double icpMaxDistance = icpValues.IcpMaxDistance;

            if (voxelSize <= 0.0) voxelSize = 0.5;
            if (icpMaxDistance <= 0.0) icpMaxDistance = 3.0;
            if (mergeVoxel < 0.0) mergeVoxel = 0.0;


            // --------------------------------------------------------------------------
            // Merge robot-based scans
            // --------------------------------------------------------------------------
            if (worldScans != null && worldScans.Count > 0)
            {
                mergedWorld = MergeClouds(worldScans, mergeVoxel);
            }
            else
            {
                Console.WriteLine("worldScans empty.");
            }

            // --------------------------------------------------------------------------
            // Merge ICP-refined scans
            // --------------------------------------------------------------------------
            if (icpScans != null && icpScans.Count > 0)
            {
                mergedRefined = MergeClouds(icpScans, mergeVoxel);
            }
            else
            {
                Console.WriteLine("icpScans empty.");
            }

            // --------------------------------------------------------------------------
            // Evaluate pairwise RMSE before / after
            // --------------------------------------------------------------------------
            if (worldScans != null && icpScans != null && worldScans.Count > 1 && icpScans.Count > 1)
            {
                int n = Math.Min(worldScans.Count, icpScans.Count);

                report.Add("Pairwise consecutive RMSE:");
                report.Add("  Pair      Before       After");

                double sumBefore = 0.0;
                double sumAfter = 0.0;
                int validCount = 0;

                for (int i = 0; i < n - 1; i++)
                {
                    PointCloud a0 = VoxelDownsample(worldScans[i], voxelSize);
                    PointCloud b0 = VoxelDownsample(worldScans[i + 1], voxelSize);

                    PointCloud a1 = VoxelDownsample(icpScans[i], voxelSize);
                    PointCloud b1 = VoxelDownsample(icpScans[i + 1], voxelSize);

                    double rmseBefore = PairRMSE(a0, b0, icpMaxDistance);
                    double rmseAfter = PairRMSE(a1, b1, icpMaxDistance);

                    report.Add(string.Format(
                      "  {0}->{1}    {2,8:0.0000}    {3,8:0.0000}",
                      i, i + 1, rmseBefore, rmseAfter));

                    if (!double.IsNaN(rmseBefore) && !double.IsNaN(rmseAfter))
                    {
                        sumBefore += rmseBefore;
                        sumAfter += rmseAfter;
                        validCount++;
                    }
                }

                if (validCount > 0)
                {
                    double meanBefore = sumBefore / validCount;
                    double meanAfter = sumAfter / validCount;
                    double improvement = (meanBefore > 1e-12)
                      ? ((meanBefore - meanAfter) / meanBefore * 100.0)
                      : 0.0;

                    report.Add(string.Format("Mean before: {0:0.0000}", meanBefore));
                    report.Add(string.Format("Mean after : {0:0.0000}", meanAfter));
                    report.Add(string.Format("Improvement: {0:+0.0;-0.0;0.0}%", improvement));
                }
            }

            DA.SetData(0, mergedWorld);
            DA.SetData(1, mergedRefined);
            DA.SetDataList(2, report);

        }


        // ============================================================================
        // DATA STRUCTURES
        // ============================================================================
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


        // ============================================================================
        // POINT CLOUD HELPERS
        // ============================================================================
        public PointCloud DuplicatePointCloud(PointCloud pc)
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

        public PointCloud MergeClouds(List<PointCloud> clouds, double voxel)
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

        public PointCloud VoxelDownsample(PointCloud pc, double voxel)
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

        public int ClampToByte(double x)
        {
            if (x < 0.0) return 0;
            if (x > 255.0) return 255;
            return (int)Math.Round(x);
        }

        public List<Point3d> CloudToPoints(PointCloud pc)
        {
            List<Point3d> pts = new List<Point3d>();
            for (int i = 0; i < pc.Count; i++)
                pts.Add(pc[i].Location);
            return pts;
        }

        public int ClosestPointIndexBruteForce(Point3d p, List<Point3d> pts, double maxDistance)
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

        public double PairRMSE(PointCloud a, PointCloud b, double maxDistance)
        {
            if (a == null || b == null) return double.NaN;
            if (a.Count == 0 || b.Count == 0) return double.NaN;

            List<Point3d> ptsA = CloudToPoints(a);
            List<Point3d> ptsB = CloudToPoints(b);

            int count = 0;
            double sumSq = 0.0;

            for (int i = 0; i < ptsA.Count; i++)
            {
                int j = ClosestPointIndexBruteForce(ptsA[i], ptsB, maxDistance);
                if (j >= 0)
                {
                    sumSq += ptsA[i].DistanceToSquared(ptsB[j]);
                    count++;
                }
            }

            if (count == 0) return double.NaN;
            return Math.Sqrt(sumSq / count);
        }


        /// <summary>
        /// Provides an Icon for the component.
        /// </summary>
        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                //You can add image files to your project resources and access them like this:
                // return Resources.IconForThisComponent;
                return null;
            }
        }

        /// <summary>
        /// Gets the unique ID for this component. Do not change this ID after release.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("AC00B557-F129-4CC1-A37E-C5A07B3F20C2"); }
        }
    }
}