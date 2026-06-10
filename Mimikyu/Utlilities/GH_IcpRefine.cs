using Grasshopper.Kernel;
using Rhino.Geometry;
using Rhino.Render;
using System;
using System.Collections.Generic;
using System.Drawing;
using Mimikyu.Helper;

namespace Mimikyu.Utlilities
{
    public class GH_IcpRefine : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the GH_IcpRefine class.
        /// </summary>
        public GH_IcpRefine()
          : base("ICPRefinement", "Nickname",
              "Description",
              "Category", "Subcategory")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("WorldScans", "S", "The world scans to refine.", GH_ParamAccess.list);
            pManager.AddBooleanParameter("Trigger", "T", "Trigger to run the ICP refinement.", GH_ParamAccess.item);
            pManager.AddNumberParameter("VoxelSize", "V", "Voxel size for downsampling (default 0.5).", GH_ParamAccess.item, 0.5);
            pManager.AddNumberParameter("ICPMaxDistance", "D", "Maximum distance for ICP correspondences (default 3.0).", GH_ParamAccess.item, 3.0);
            pManager.AddIntegerParameter("ICPMaxIterations", "I", "Maximum iterations for ICP (default 100).", GH_ParamAccess.item, 100);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("RefinedScans", "R", "The refined scans.", GH_ParamAccess.list);
            pManager.AddGenericParameter("ICPValues", "V", "The ICP values (fitness, RMSE, etc.) for each scan.", GH_ParamAccess.item);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            List<PointCloud> worldScans = new List<PointCloud>();
            bool trigger = false;
            double voxelSize = 0.5;
            double icpMaxDistance = 3.0;
            int icpMaxIterations = 100;

            if (DA.GetDataList(0, worldScans)) return;

            List<PointCloud> outScans = new List<PointCloud>();
            List<Transform> outTransforms = new List<Transform>();

            if (worldScans == null || worldScans.Count == 0)
            {
                Console.WriteLine("worldScans is empty.");
                return;
            }

            if (voxelSize <= 0.0) voxelSize = 0.5;
            if (icpMaxDistance <= 0.0) icpMaxDistance = 3.0;
            if (icpMaxIterations <= 0) icpMaxIterations = 100;

            int n = worldScans.Count;

            // First scan fixed
            PointCloud first = DuplicatePointCloud(worldScans[0]);
            outScans.Add(first);
            outTransforms.Add(Transform.Identity);

            PointCloud accumulated = DuplicatePointCloud(first);
            PointCloud accumulatedDown = VoxelDownsample(accumulated, voxelSize);



            for (int i = 1; i < n; i++)
            {
                PointCloud sourceFull = DuplicatePointCloud(worldScans[i]);
                PointCloud sourceDown = VoxelDownsample(sourceFull, voxelSize);

                ICPResult result = RunICP(
                  sourceDown,
                  accumulatedDown,
                  icpMaxDistance,
                  icpMaxIterations,
                  1e-5
                );

                PointCloud refined = DuplicatePointCloud(sourceFull);
                refined.Transform(result.Transform);

                outScans.Add(refined);
                outTransforms.Add(result.Transform);

                accumulated = MergeTwoClouds(accumulated, refined);
                accumulatedDown = VoxelDownsample(accumulated, voxelSize);
            }

            IcpValues icpValue = new IcpValues
            {
                VoxelSize = voxelSize,
                IcpMaxDistance = icpMaxDistance,
                IcpMaxIterations = icpMaxIterations
            };


            DA.SetDataList(0, outScans);
            DA.SetData(1, icpValue);
        }

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
        // ICP
        // ============================================================================
        public ICPResult RunICP(PointCloud source, PointCloud target, double maxDistance, int maxIterations, double tol)
        {
            ICPResult result = new ICPResult();

            if (source == null || target == null) return result;
            if (source.Count < 3 || target.Count < 3) return result;

            List<Point3d> srcPts = CloudToPoints(source);
            List<Point3d> tgtPts = CloudToPoints(target);

            Transform total = Transform.Identity;
            double prevRmse = double.MaxValue;

            for (int iter = 0; iter < maxIterations; iter++)
            {
                List<Point3d> srcCorr = new List<Point3d>();
                List<Point3d> tgtCorr = new List<Point3d>();
                double sumSq = 0.0;

                for (int i = 0; i < srcPts.Count; i++)
                {
                    Point3d p = srcPts[i];
                    int j = ClosestPointIndexBruteForce(p, tgtPts, maxDistance);
                    if (j >= 0)
                    {
                        Point3d q = tgtPts[j];
                        srcCorr.Add(p);
                        tgtCorr.Add(q);
                        sumSq += p.DistanceToSquared(q);
                    }
                }

                int corrCount = srcCorr.Count;
                if (corrCount < 3) break;

                double rmse = Math.Sqrt(sumSq / corrCount);

                Transform delta = BestFitRigidTransform(srcCorr, tgtCorr);

                for (int i = 0; i < srcPts.Count; i++)
                {
                    Point3d p = srcPts[i];
                    p.Transform(delta);
                    srcPts[i] = p;
                }

                total = TransformHelper.MultiplyTransform(delta, total);

                if (Math.Abs(prevRmse - rmse) < tol)
                {
                    result.Transform = total;
                    result.Fitness = (double)corrCount / (double)source.Count;
                    result.RMSE = rmse;
                    result.CorrespondenceCount = corrCount;
                    return result;
                }

                prevRmse = rmse;
            }

            // Final evaluation
            int finalCount = 0;
            double finalSumSq = 0.0;

            for (int i = 0; i < srcPts.Count; i++)
            {
                int j = ClosestPointIndexBruteForce(srcPts[i], tgtPts, maxDistance);
                if (j >= 0)
                {
                    finalCount++;
                    finalSumSq += srcPts[i].DistanceToSquared(tgtPts[j]);
                }
            }

            result.Transform = total;
            result.Fitness = (source.Count > 0) ? ((double)finalCount / source.Count) : 0.0;
            result.RMSE = (finalCount > 0) ? Math.Sqrt(finalSumSq / finalCount) : double.NaN;
            result.CorrespondenceCount = finalCount;

            return result;
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

        public PointCloud MergeTwoClouds(PointCloud a, PointCloud b)
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

        // ============================================================================
        // BEST-FIT RIGID TRANSFORM (Horn quaternion method)
        // ============================================================================
        public Transform BestFitRigidTransform(List<Point3d> A, List<Point3d> B)
        {
            Transform xf = Transform.Identity;

            int n = Math.Min(A.Count, B.Count);
            if (n < 3) return xf;

            Point3d ca = Centroid(A);
            Point3d cb = Centroid(B);

            double Sxx = 0, Sxy = 0, Sxz = 0;
            double Syx = 0, Syy = 0, Syz = 0;
            double Szx = 0, Szy = 0, Szz = 0;

            for (int i = 0; i < n; i++)
            {
                double ax = A[i].X - ca.X;
                double ay = A[i].Y - ca.Y;
                double az = A[i].Z - ca.Z;

                double bx = B[i].X - cb.X;
                double by = B[i].Y - cb.Y;
                double bz = B[i].Z - cb.Z;

                Sxx += ax * bx; Sxy += ax * by; Sxz += ax * bz;
                Syx += ay * bx; Syy += ay * by; Syz += ay * bz;
                Szx += az * bx; Szy += az * by; Szz += az * bz;
            }

            double[,] N = new double[4, 4];
            N[0, 0] = Sxx + Syy + Szz;
            N[0, 1] = Syz - Szy;
            N[0, 2] = Szx - Sxz;
            N[0, 3] = Sxy - Syx;

            N[1, 0] = Syz - Szy;
            N[1, 1] = Sxx - Syy - Szz;
            N[1, 2] = Sxy + Syx;
            N[1, 3] = Szx + Sxz;

            N[2, 0] = Szx - Sxz;
            N[2, 1] = Sxy + Syx;
            N[2, 2] = -Sxx + Syy - Szz;
            N[2, 3] = Syz + Szy;

            N[3, 0] = Sxy - Syx;
            N[3, 1] = Szx + Sxz;
            N[3, 2] = Syz + Szy;
            N[3, 3] = -Sxx - Syy + Szz;

            double[] q = DominantEigenvector4x4(N);

            double qn = Math.Sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
            if (qn < 1e-12) return Transform.Identity;
            q[0] /= qn; q[1] /= qn; q[2] /= qn; q[3] /= qn;

            double w = q[0];
            double x = q[1];
            double y = q[2];
            double z = q[3];

            double r00 = 1.0 - 2.0 * (y * y + z * z);
            double r01 = 2.0 * (x * y - z * w);
            double r02 = 2.0 * (x * z + y * w);

            double r10 = 2.0 * (x * y + z * w);
            double r11 = 1.0 - 2.0 * (x * x + z * z);
            double r12 = 2.0 * (y * z - x * w);

            double r20 = 2.0 * (x * z - y * w);
            double r21 = 2.0 * (y * z + x * w);
            double r22 = 1.0 - 2.0 * (x * x + y * y);

            double tx = cb.X - (r00 * ca.X + r01 * ca.Y + r02 * ca.Z);
            double ty = cb.Y - (r10 * ca.X + r11 * ca.Y + r12 * ca.Z);
            double tz = cb.Z - (r20 * ca.X + r21 * ca.Y + r22 * ca.Z);

            xf = Transform.Identity;
            xf.M00 = r00; xf.M01 = r01; xf.M02 = r02; xf.M03 = tx;
            xf.M10 = r10; xf.M11 = r11; xf.M12 = r12; xf.M13 = ty;
            xf.M20 = r20; xf.M21 = r21; xf.M22 = r22; xf.M23 = tz;
            xf.M30 = 0.0; xf.M31 = 0.0; xf.M32 = 0.0; xf.M33 = 1.0;

            return xf;
        }

        public Point3d Centroid(List<Point3d> pts)
        {
            if (pts == null || pts.Count == 0) return Point3d.Origin;

            double x = 0.0, y = 0.0, z = 0.0;
            for (int i = 0; i < pts.Count; i++)
            {
                x += pts[i].X;
                y += pts[i].Y;
                z += pts[i].Z;
            }

            return new Point3d(x / pts.Count, y / pts.Count, z / pts.Count);
        }

        public double[] DominantEigenvector4x4(double[,] A)
        {
            double[] v = new double[4] { 1, 0, 0, 0 };

            for (int iter = 0; iter < 60; iter++)
            {
                double[] w = new double[4];

                for (int i = 0; i < 4; i++)
                {
                    w[i] = 0.0;
                    for (int j = 0; j < 4; j++)
                        w[i] += A[i, j] * v[j];
                }

                double norm = Math.Sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2] + w[3] * w[3]);
                if (norm < 1e-12) break;

                for (int i = 0; i < 4; i++)
                    v[i] = w[i] / norm;
            }

            return v;
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
            get { return new Guid("89BBC349-01C5-4478-A1BC-90607A9D5DA6"); }
        }
    }
}