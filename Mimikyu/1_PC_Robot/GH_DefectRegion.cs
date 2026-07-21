using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Mimikyu.Helper;

using Rhino.Geometry;
using Rhino.UI;
using System;
using System.Collections.Generic;
using System.IO;
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
          : base("DefectRegion", "DefReg",
              "Defect Pixels Projected to Object",
              "Mimikyu", "PC_Robot")
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
            pManager.AddTextParameter("PixelPath", "Px", "Json path of pixel coordinates to project.", GH_ParamAccess.item);
            pManager.AddPlaneParameter("TargetPlane", "TP", "The target plane to project the pixels onto.", GH_ParamAccess.item);
            pManager.AddBoxParameter("Box", "B", "Object Bounding Box", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            //pManager.AddPointParameter("ProjectedPoints", "PP", "List of projected points on the target plane.", GH_ParamAccess.tree);
            pManager.AddPointParameter("DefectPoints", "DP", "Defect points projected on the BIM object", GH_ParamAccess.tree);
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
            string pixelPath = default;
            Plane targetPlane = Plane.Unset;
            Box obb = default;

            if (!DA.GetData(0, ref intrinsicsPath)) return;
            if (!DA.GetData(1, ref cameraToRobotPath)) return;
            if (!DA.GetData(2, ref posePath)) return;
            if (!DA.GetData(3, ref pixelPath)) return;
            if (!DA.GetData(4, ref targetPlane)) return;
            if (!DA.GetData(5, ref obb)) return;

            Plane objectPlane = obb.Plane;
            Mesh mesh = Mesh.CreateFromBox(obb, 1, 1, 1);


            Dictionary<string, List<List<Point3d>>> allDefects = LoadDefectContours(pixelPath);

            DataTree<Point3d> points = new DataTree<Point3d>();
            DataTree<Point3d> pixelHits = new DataTree<Point3d>();
            int poseIndex = 0;
            foreach (string imageName in allDefects.Keys)
            {
                List<List<Point3d>> contours = allDefects[imageName];

                for (int i = 0; i < contours.Count; i++)
                { 
                    //List<Point3d> projectedPoints = 
                    //    PixelHelper.ProjectPixelsToPlane(
                    //                                        intrinsicsPath,
                    //                                        cameraToRobotPath,
                    //                                        posePath,
                    //                                        poseIndex,
                    //                                        contours[i],
                    //                                        targetPlane
                    //                                    );
                    List<PixelObjectHit> objectHits = 
                                    ProjectPixelsToObjectMesh(
                                                              intrinsicsPath,
                                                              cameraToRobotPath,
                                                              posePath,
                                                              poseIndex,
                                                              contours[i],
                                                              mesh,
                                                              objectPlane
                                                             );
                    //points.AddRange(projectedPoints, path);
                    List<Point3d> pts = objectHits.Select(p => p.Point).ToList();
                    List<string> sideString = objectHits.Select(s => s.SideKey).ToList();

                    if (pts.Count != 0)
                    {
                        for (int p = 0; p < pts.Count; p++)
                        {
                            switch (sideString[p])
                            {
                                case "posX":
                                    GH_Path pathPosX = new GH_Path(poseIndex).AppendElement((int)Sides.posX).AppendElement(i);
                                    pixelHits.Add(pts[p], pathPosX);
                                    break;
                                case "posY":
                                    GH_Path pathPosY = new GH_Path(poseIndex).AppendElement((int)Sides.posY).AppendElement(i);
                                    pixelHits.Add(pts[p], pathPosY);
                                    break;
                                case "negX":
                                    GH_Path pathNegX = new GH_Path(poseIndex).AppendElement((int)Sides.negX).AppendElement(i);
                                    pixelHits.Add(pts[p], pathNegX);
                                    break;
                                case "negY":
                                    GH_Path pathNegY = new GH_Path(poseIndex).AppendElement((int)Sides.negY).AppendElement(i);
                                    pixelHits.Add(pts[p], pathNegY);
                                    break;
                                case "posZ":
                                    GH_Path pathPosZ = new GH_Path(poseIndex).AppendElement((int)Sides.posZ).AppendElement(i);
                                    pixelHits.Add(pts[p], pathPosZ);
                                    break;
                                case "negZ":
                                    GH_Path pathNegZ = new GH_Path(poseIndex).AppendElement((int)Sides.negZ).AppendElement(i);
                                    pixelHits.Add(pts[p], pathNegZ);
                                    break;
                            }
                        
                        }
                    }
                }
                poseIndex++;
            }

            //DA.SetDataTree(0, points);
            DA.SetDataTree(0, pixelHits);
        }
        public enum Sides
        {
            posX = 0,
            posY = 1,
            negX = 2,
            negY = 3,
            posZ = 4,
            negZ = 5
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