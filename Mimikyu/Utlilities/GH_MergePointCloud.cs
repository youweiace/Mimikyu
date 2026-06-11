using Grasshopper.Kernel;
using Mimikyu.Helper;
using Rhino.Geometry;
using Rhino.Render;
using System;
using System.Collections.Generic;
using System.Drawing;
using Mimikyu.Helper;

namespace Mimikyu.Utlilities
{
    public class GH_MergePointCloud : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the GH_MergePointCloud class.
        /// </summary>
        public GH_MergePointCloud()
          : base("MergePointCloud", "MPC",
              "Merge Point cloud and compare world and iterative alignmnent",
              "Mimikyu", "Utilities")
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
            pManager[2].Optional = true;
            pManager[3].Optional = true;
            pManager[4].Optional = true;
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
            DA.GetDataList(2, icpScans);
            DA.GetData(3, ref icpValues);
            DA.GetData(4, ref mergeVoxel);

            PointCloud mergedWorld = new PointCloud();
            PointCloud mergedRefined = new PointCloud();
            List<string> report = default;

            if (run) 
            {
                // --------------------------------------------------------------------------
                // Merge robot-based scans
                // --------------------------------------------------------------------------
                if (worldScans != null && worldScans.Count > 0)
                {
                    mergedWorld = PointCloudHelper.MergeCloudsVoxel(worldScans, mergeVoxel);
                }
                else
                {
                    Console.WriteLine("worldScans empty.");
                }
                
                // --------------------------------------------------------------------------
                // Merge ICP-refined scans
                // --------------------------------------------------------------------------
                if (icpScans != null && icpValues != null && mergeVoxel != 0)
                {
                    double voxelSize = icpValues.VoxelSize;
                    double icpMaxDistance = icpValues.IcpMaxDistance;

                    if (voxelSize <= 0.0) voxelSize = 0.5;
                    if (icpMaxDistance <= 0.0) icpMaxDistance = 3.0;
                    if (mergeVoxel < 0.0) mergeVoxel = 0.0;

                    if (icpScans != null && icpScans.Count > 0)
                    {
                        mergedRefined = PointCloudHelper.MergeClouds(icpScans, mergeVoxel);
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
                            PointCloud a0 = PointCloudHelper.VoxelDownsample(worldScans[i], voxelSize);
                            PointCloud b0 = PointCloudHelper.VoxelDownsample(worldScans[i + 1], voxelSize);

                            PointCloud a1 = PointCloudHelper.VoxelDownsample(icpScans[i], voxelSize);
                            PointCloud b1 = PointCloudHelper.VoxelDownsample(icpScans[i + 1], voxelSize);

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
                }
            }

            DA.SetData(0, mergedWorld);
            DA.SetData(1, mergedRefined);
            DA.SetDataList(2, report);

        }


        public double PairRMSE(PointCloud a, PointCloud b, double maxDistance)
        {
            if (a == null || b == null) return double.NaN;
            if (a.Count == 0 || b.Count == 0) return double.NaN;

            List<Point3d> ptsA = PointCloudHelper.CloudToPoints(a);
            List<Point3d> ptsB = PointCloudHelper.CloudToPoints(b);

            int count = 0;
            double sumSq = 0.0;

            for (int i = 0; i < ptsA.Count; i++)
            {
                int j = PointCloudHelper.ClosestPointIndexBruteForce(ptsA[i], ptsB, maxDistance);
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