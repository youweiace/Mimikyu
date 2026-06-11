using Grasshopper.Kernel;
using Mimikyu.Helper;
using Newtonsoft.Json.Linq;
using Rhino;
using Rhino.Geometry;
using Rhino.Render;
using System;
using System.Collections.Generic;
using System.IO;
using Mimikyu.Helper;

namespace Mimikyu.Utlilities
{
    public class GH_TransformScan : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the TransformScan class.
        /// </summary>
        public GH_TransformScan()
          : base("TransformScan", "TS",
              "Tranfsorm scanned point clouds ",
              "Mimikyu", "Utilities")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddBooleanParameter("Run", "R", "Run to transform the scan.", GH_ParamAccess.item);
            pManager.AddGenericParameter("Scan", "S", "The scan to transform.", GH_ParamAccess.list);
            pManager.AddGenericParameter("Pose", "P", "The robot pose to use from where scan was taken.", GH_ParamAccess.list);
            pManager.AddTextParameter("TransformPath", "TP", "File path to use as transformation", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("TransformedScan", "T", "The transformed scan.", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            bool run = false;
            List<PointCloud> scans = new List<PointCloud>();
            List<RobotPose> robotPose = new List<RobotPose>();
            string filePath = null;
            Transform CalibX = Transform.Identity;

            if (!DA.GetData(0, ref run)) return;
            if (!DA.GetDataList(1, scans)) return;
            if (!DA.GetDataList(2, robotPose)) return;
            if (!DA.GetData(3, ref filePath)) return;


            if (string.IsNullOrWhiteSpace(filePath))
            {
                Console.WriteLine("JsonPath is empty.");
                return;
            }

            if (!File.Exists(filePath))
            {
                Console.WriteLine("File does not exist:\n" + filePath);
                return;
            }

            try
            {
                string json = File.ReadAllText(filePath);
                JObject data = JObject.Parse(json);

                // ------------------------------------------------------------------------
                // Read rotation matrix
                // ------------------------------------------------------------------------
                JArray R = (JArray)data["rotation_matrix"];

                double r00 = (double)R[0][0];
                double r01 = (double)R[0][1];
                double r02 = (double)R[0][2];

                double r10 = (double)R[1][0];
                double r11 = (double)R[1][1];
                double r12 = (double)R[1][2];

                double r20 = (double)R[2][0];
                double r21 = (double)R[2][1];
                double r22 = (double)R[2][2];

                // ------------------------------------------------------------------------
                // Read translation
                // ------------------------------------------------------------------------
                JArray t = (JArray)data["translation_mm"];

                double tx = (double)t[0];
                double ty = (double)t[1];
                double tz = (double)t[2];

                // ------------------------------------------------------------------------
                // Build Rhino Transform
                // ------------------------------------------------------------------------
                Transform T = Transform.Identity;

                T.M00 = r00;
                T.M01 = r01;
                T.M02 = r02;
                T.M03 = tx;

                T.M10 = r10;
                T.M11 = r11;
                T.M12 = r12;
                T.M13 = ty;

                T.M20 = r20;
                T.M21 = r21;
                T.M22 = r22;
                T.M23 = tz;

                T.M30 = 0.0;
                T.M31 = 0.0;
                T.M32 = 0.0;
                T.M33 = 1.0;

                CalibX = T;

                Console.WriteLine(
                  "Calibration transform loaded successfully.\n\n" +
                  "Transform meaning:\n" +
                  "scanner -> flange\n\n" +
                  string.Format(
                    "Translation: [{0:0.###}, {1:0.###}, {2:0.###}] mm",
                    tx, ty, tz));
            }
            catch (Exception ex)
            {
                Console.WriteLine("Error reading JSON:\n" + ex.Message);
            }

            List<PointCloud> outScans = new List<PointCloud>();

            List<string> log = new List<string>();

            if (robotPose == null || scans == null)
            {
                Console.WriteLine("flangePoses or rawScans is null.");
                return;
            }

            int n = Math.Min(robotPose.Count, scans.Count);
            if (n == 0)
            {
                Console.WriteLine("No input pose/scan pairs.");
                return;
            }


            for (int i = 0; i < n; i++)
            {
                Transform T_flange_world = PoseToTransform(robotPose[i]);
                Transform T_world = TransformHelper.MultiplyTransform(T_flange_world, CalibX);

                PointCloud pc = PointCloudHelper.DuplicatePointCloud(scans[i]);
                pc.Transform(T_world);

                outScans.Add(pc);

                log.Add(string.Format("  Scan {0}: {1} pts", i, pc.Count));
            }

            DA.SetDataList(0, outScans);

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
            get { return new Guid("5C31F755-B003-409B-8B77-FF04E7403FF5"); }
        }

        private Transform PoseToTransform(RobotPose rp)
        {
            double a = RhinoMath.ToRadians(rp.A);
            double b = RhinoMath.ToRadians(rp.B);
            double c = RhinoMath.ToRadians(rp.C);

            Transform Rx = Transform.Rotation(c, Vector3d.XAxis, Point3d.Origin);
            Transform Ry = Transform.Rotation(b, Vector3d.YAxis, Point3d.Origin);
            Transform Rz = Transform.Rotation(a, Vector3d.ZAxis, Point3d.Origin);

            Transform R = TransformHelper.MultiplyTransform(Rz, TransformHelper.MultiplyTransform(Ry, Rx));

            R.M03 = rp.X;
            R.M13 = rp.Y;
            R.M23 = rp.Z;

            return R;
        }

    }
}