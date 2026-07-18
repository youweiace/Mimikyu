using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Mimikyu.Helper;

using Rhino.Geometry;
using Rhino.UI;
using System;
using System.Collections.Generic;
using System.Linq;
using static Mimikyu.Helper.PixelHelper;

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
            pManager.AddTextParameter("IntrinsicsPath", "I", "Json path to the camera intrinsics file.", GH_ParamAccess.item);
            pManager.AddTextParameter("CameraToRobotPath", "C", "Json path to the camera-to-robot transformation file.", GH_ParamAccess.item);
            pManager.AddTextParameter("PosePath", "P", "Json path to the robot pose file.", GH_ParamAccess.item);
            pManager.AddIntegerParameter("PoseIndex", "PI", "Index of the pose to use.", GH_ParamAccess.item);
            pManager.AddTextParameter("PixelPath", "Px", "Json path of pixel coordinates to project.", GH_ParamAccess.item);
            pManager.AddPlaneParameter("TargetPlane", "TP", "The target plane to project the pixels onto.", GH_ParamAccess.item);
            pManager.AddBoxParameter("Box", "B", "Object Bounding Box", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddPointParameter("ProjectedPoints", "PP", "List of projected points on the target plane.", GH_ParamAccess.tree);
            pManager.AddPointParameter("PointsHits", "PP", "List of projected points on the target plane.", GH_ParamAccess.tree);
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
            string pixelPath = default;
            Plane targetPlane = Plane.Unset;
            Box obb = default;

            if (!DA.GetData(0, ref intrinsicsPath)) return;
            if (!DA.GetData(1, ref cameraToRobotPath)) return;
            if (!DA.GetData(2, ref posePath)) return;
            if (!DA.GetData(3, ref poseIndex)) return;
            if (!DA.GetData(4, ref pixelPath)) return;
            if (!DA.GetData(5, ref targetPlane)) return;
            if (!DA.GetData(6, ref obb)) return;

            Plane objectPlane = obb.Plane;
            Mesh mesh = Mesh.CreateFromBox(obb, 1, 1, 1);

            List<List<Point3d>> contours =
                PixelHelper.LoadDefectContours(pixelPath);

            DataTree<Point3d> points = new DataTree<Point3d>();
            DataTree<Point3d> pixelHits = new DataTree<Point3d>();

            for (int i = 0; i < contours.Count; i++)
            { 
                List<Point3d> projectedPoints = 
                    PixelHelper.ProjectPixelsToPlane(
                                                        intrinsicsPath,
                                                        cameraToRobotPath,
                                                        posePath,
                                                        poseIndex,
                                                        contours[i],
                                                        targetPlane
                                                    );
                List<PixelObjectHit> objectHit =  PixelHelper.ProjectPixelsToObjectMesh(intrinsicsPath,
                    cameraToRobotPath, posePath, poseIndex, contours[i], mesh, objectPlane
                                                     );

                GH_Path path = new GH_Path(i);
                points.AddRange(projectedPoints, path);
                pixelHits.AddRange(objectHit.Select(p => p.Point), path);
            }

            DA.SetDataTree(0, points);
            DA.SetDataTree(1, pixelHits);
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