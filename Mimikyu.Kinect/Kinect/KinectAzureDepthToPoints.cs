using Grasshopper.Kernel;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using Microsoft.Azure.Kinect.Sensor;
using Grasshopper.Kernel.Types;
using Color = System.Drawing.Color;
using Grasshopper.Kernel.Parameters;
using System.Linq;
using System.Security.Cryptography;
using Mimikyu.Kinect.Helper;
using Mimikyu.Kinect.Properties;

namespace KinectAzureDK
{
    public class KinectAzureDepthToPoints : GH_Component
    {
        Transformation transformation = null;
        Image pointCloud = new Image(ImageFormat.Custom, 1,1, 1* 3 * sizeof(Int16));
        Image colorsInDepth = new Image(ImageFormat.ColorBGRA32, 1, 1, 1* 4 * sizeof(byte));
        Calibration oldCalibration;
        
        /// <summary>
        /// Each implementation of GH_Component must provide a public 
        /// constructor without any arguments.
        /// Category represents the Tab in which the component will appear, 
        /// Subcategory the panel. If you use non-existing tab or panel names, 
        /// new tabs/panels will automatically be created.
        /// </summary>
        public KinectAzureDepthToPoints()
          : base("Depth capture to Points", "DC2P",
            "Transforms the depth capture to 3D points",
            "Kinect", "Azure Kinect")
        {
        }

        protected override System.Drawing.Bitmap Icon => Resources.AzureKinectIcon;

        public override Guid ComponentGuid => new Guid("c67268f7-8e2f-4212-9e88-cb2e9456ae0e");
        

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Depth Capture", "DC", "Depth capture object", GH_ParamAccess.item);
            pManager.AddGenericParameter("Color Capture", "CC", "Color capture object", GH_ParamAccess.item);
            pManager[1].Optional = true;
            pManager.AddGenericParameter("Calibration", "c", "Calibration object", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddPointParameter("Points", "P", "Points in 3D space", GH_ParamAccess.list);
            pManager.AddColourParameter("Colors", "C", "Colors for each point", GH_ParamAccess.list);
        }



    

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object can be used to retrieve data from input parameters and 
        /// to store data in output parameters.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {

            Calibration calibration = new Calibration();
            Image depthCapture = null;
            Image colorCapture = null;

            if (!DA.GetData(0, ref depthCapture)) return;
            if (!DA.GetData(1, ref colorCapture)) return;
            if (!DA.GetData(2, ref calibration)) return;

            if(depthCapture.Format != ImageFormat.Depth16)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Passive IR captures can't be converted to 3D points");
                return;
            }

            if (oldCalibration != calibration)
            {
                oldCalibration = calibration;
                transformation = new Transformation(calibration);
            }

            if (pointCloud.WidthPixels != depthCapture.WidthPixels || pointCloud.HeightPixels != depthCapture.HeightPixels
                || colorsInDepth.WidthPixels != depthCapture.WidthPixels || colorsInDepth.HeightPixels != depthCapture.HeightPixels)
            {
                pointCloud = new Image(ImageFormat.Custom, depthCapture.WidthPixels, depthCapture.HeightPixels, depthCapture.WidthPixels * 3 * sizeof(Int16));
                colorsInDepth = new Image(ImageFormat.ColorBGRA32, depthCapture.WidthPixels, depthCapture.HeightPixels, depthCapture.WidthPixels * 4 * sizeof(byte));
            }


            List<GH_Point> points = new List<GH_Point>();
            List<GH_Colour> colors = new List<GH_Colour>();
            if (colorCapture != null)
            {
                outputWithColor(depthCapture, colorCapture, points, colors);
            }
            else
            {
                outputNoColor(depthCapture, points);
            }

            DA.SetDataList(0, points);
            DA.SetDataList(1, colors);
        }

        void outputWithColor(Image depthCapture,Image colorCapture, List<GH_Point> outPoints, List<GH_Colour> outColors)
        {
            transformation.ColorImageToDepthCamera(depthCapture, colorCapture, colorsInDepth);
            transformation.DepthImageToPointCloud(depthCapture, pointCloud);


            Memory<Short3> pointCloudMemory = pointCloud.GetPixels<Short3>();
            Span<Short3> pcs = pointCloudMemory.Span;

            Memory<Byte4> colorMemory = colorsInDepth.GetPixels<Byte4>();
            Span<Byte4> cs = colorMemory.Span;

            int pixels = pointCloud.WidthPixels * pointCloud.HeightPixels;
            for (int i = 0; i < pixels; i++)
            {
                if ((pcs[i].X == 0 && pcs[i].Y == 0 && pcs[i].Z == 0) || (cs[i].R == 0 && cs[i].G == 0 && cs[i].B == 0))
                    continue;

                outPoints.Add(new GH_Point(new Point3d(pcs[i].X, -pcs[i].Y, -pcs[i].Z)));
                outColors.Add(new GH_Colour(Color.FromArgb(cs[i].R, cs[i].G, cs[i].B)));
            }
        }

        void outputNoColor(Image depthCapture, List<GH_Point> outPoints)
        {
            transformation.DepthImageToPointCloud(depthCapture, pointCloud);

            Memory<Short3> pointCloudMemory = pointCloud.GetPixels<Short3>();
            Span<Short3> pcs = pointCloudMemory.Span;

            int pixels = pointCloud.WidthPixels * pointCloud.HeightPixels;
            for (int i = 0; i < pixels; i++)
            {
                if ((pcs[i].X == 0 && pcs[i].Y == 0 && pcs[i].Z == 0))
                    continue;

                outPoints.Add(new GH_Point(new Point3d(pcs[i].X, -pcs[i].Y, -pcs[i].Z)));
            }
        }

 

    }
}
