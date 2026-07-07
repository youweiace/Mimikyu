using Grasshopper.Kernel;
using Rhino.Geometry;
using Rhino.UI;
using System;
using System.Collections.Generic;
using Mimikyu.Helper;

namespace Mimikyu.Polyga
{
    public class GH_ScanRegion : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the GH_ScanRegion class.
        /// </summary>
        public GH_ScanRegion()
          : base("ScanRegion", "R",
              "Detailed Scanning Region",
              "Mimikyu", "Polyga")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddTextParameter("IntrinsicsPath", "I", "Path to the camera intrinsics file.", GH_ParamAccess.item);
            pManager.AddTextParameter("CameraToRobotPath", "C", "Path to the camera-to-robot transformation file.", GH_ParamAccess.item);
            pManager.AddTextParameter("PosePath", "P", "Path to the pose file.", GH_ParamAccess.item);
            pManager.AddIntegerParameter("PoseIndex", "PI", "Index of the pose to use.", GH_ParamAccess.item);
            pManager.AddPointParameter("Pixels", "Px", "List of pixel coordinates to project.", GH_ParamAccess.list);
            pManager.AddPlaneParameter("TargetPlane", "TP", "The target plane to project the pixels onto.", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddPointParameter("ProjectedPoints", "PP", "List of projected points on the target plane.", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {

            string intrinsicsPath = default;
            string cameraToRobotPath = default;
            string posePath = default;
            int poseIndex = 0;
            List<Point3d> pixels = new List<Point3d>();
            Plane targetPlane = Plane.Unset;

            if (!DA.GetData(0, ref intrinsicsPath)) return;
            if (!DA.GetData(1, ref cameraToRobotPath)) return;
            if (!DA.GetData(2, ref posePath)) return;
            if (!DA.GetData(3, ref poseIndex)) return;
            if (!DA.GetDataList(4, pixels)) return;
            if (!DA.GetData(5, ref targetPlane)) return;

            List<Point3d> projectedPoints = 
                PixelHelper.ProjectPixelsToPlane(
                                                    intrinsicsPath,
                                                    cameraToRobotPath,
                                                    posePath,
                                                    poseIndex,
                                                    pixels,
                                                    targetPlane
                                                );
            DA.SetDataList(0, projectedPoints);
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
            get { return new Guid("8CD84D49-673F-4F33-A2C0-00F7AD818852"); }
        }
    }
}