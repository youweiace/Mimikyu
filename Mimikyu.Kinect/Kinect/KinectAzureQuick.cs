using Grasshopper.Kernel;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using Microsoft.Azure.Kinect.Sensor;
using Grasshopper.Kernel.Types;
using Color = System.Drawing.Color;
using Grasshopper.Kernel.Parameters;
using System.Linq;
using Mimikyu.Kinect.Helper;
using Mimikyu.Kinect.Properties;

namespace KinectAzureDK
{
    public class KinectAzureQuick : GH_Component
    {

        bool isSetup = false;
        Device device = null;
        Transformation transformation = null;
        Image pointCloud;
        Image colorsInDepth;
        int oldFPS = 0;
        int oldDepthMode = 3;
        int oldColorMode = 0;
        Calibration calibration;
        /// <summary>
        /// Each implementation of GH_Component must provide a public 
        /// constructor without any arguments.
        /// Category represents the Tab in which the component will appear, 
        /// Subcategory the panel. If you use non-existing tab or panel names, 
        /// new tabs/panels will automatically be created.
        /// </summary>
        public KinectAzureQuick()
          : base("Azure Kinect Quick", "QKinect",
            "Receive data from a Azure Kinect. Abstracts most of the inner workings to get quickly get started.",
            "Kinect", "Azure Kinect")
        {
        }

        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                return Resources.AzureKinectIcon;
            }
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddBooleanParameter("Active", "A", "Set to true to connect to camera", GH_ParamAccess.item, false);

            pManager.AddIntegerParameter("FPS", "F", "FPS of recording", GH_ParamAccess.item, 1);
            Param_Integer fps = pManager[1] as Param_Integer;
            fps.AddNamedValue("5", 0);
            fps.AddNamedValue("15", 1);
            fps.AddNamedValue("30", 2);

            pManager.AddIntegerParameter("Depth Mode", "DM", "Mode to record depth", GH_ParamAccess.item, 0);
            Param_Integer depthMode = pManager[2] as Param_Integer;
            depthMode.AddNamedValue("WFOV 2x2 Binned", 0);
            depthMode.AddNamedValue("WFOV", 1);
            depthMode.AddNamedValue("NFOV 2x2 Binned", 2);
            depthMode.AddNamedValue("NFOV", 3);
            depthMode.AddNamedValue("Passive IR", 4);

            pManager.AddBooleanParameter("Enable Color", "C", "Enable Color Output", GH_ParamAccess.item, true);

            pManager.AddIntegerParameter("Color Mode", "CM", "Mode to record color", GH_ParamAccess.item, 0);
            Param_Integer colorMode = pManager[4] as Param_Integer;
            colorMode.AddNamedValue("720p", 0);
            colorMode.AddNamedValue("1080p", 1);
            colorMode.AddNamedValue("1440p", 2);
            colorMode.AddNamedValue("2160p", 3);
            colorMode.AddNamedValue("3072p", 4);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddPointParameter("Points", "P", "Points in 3D space", GH_ParamAccess.list);
            pManager.AddColourParameter("Colors", "C", "Colors for each point", GH_ParamAccess.list);
            pManager.AddGenericParameter("Depth Calibration", "DC", "Calibration of the depth camera", GH_ParamAccess.list);
            pManager.AddGenericParameter("Color Calibration", "CC", "Calibration of the color camera", GH_ParamAccess.list);
        }


        void TeardownKinect()
        {
            if (isSetup)
            {
                device.StopCameras();
                device.Dispose();
                isSetup = false;
            }
        }

        bool SetupKinect(int fps, int depthMode, int colorMode)
        {
            int installedCount = Device.GetInstalledCount();
            if (installedCount <= 0)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "No Kinect connected");
                return false;
            }

            try
            {
                device = Device.Open(0);

            }
            catch
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Could not open connection");
                return false;
            }

            DeviceConfiguration deviceConfiguration = new DeviceConfiguration();
            deviceConfiguration.SynchronizedImagesOnly = true;
            deviceConfiguration.ColorFormat = ImageFormat.ColorBGRA32;

            switch (depthMode)
            {
                case 0:
                    deviceConfiguration.DepthMode = DepthMode.WFOV_2x2Binned;
                    break;
                case 1:
                    deviceConfiguration.DepthMode = DepthMode.WFOV_Unbinned;
                    break;
                case 2:
                    deviceConfiguration.DepthMode = DepthMode.NFOV_2x2Binned;
                    break;
                case 3:
                    deviceConfiguration.DepthMode = DepthMode.NFOV_Unbinned;
                    break;
            }

            switch (colorMode)
            {
                case 0:
                    deviceConfiguration.ColorResolution = ColorResolution.R720p;
                    break;
                case 1:
                    deviceConfiguration.ColorResolution = ColorResolution.R1080p;
                    break;
                case 2:
                    deviceConfiguration.ColorResolution = ColorResolution.R1440p;
                    break;
                case 3:
                    deviceConfiguration.ColorResolution = ColorResolution.R2160p;
                    break;
                case 4:
                    deviceConfiguration.ColorResolution = ColorResolution.R3072p;
                    break;
            }

            switch (fps)
            {
                case 0:
                    deviceConfiguration.CameraFPS = FPS.FPS5;
                    break;
                case 1:
                    deviceConfiguration.CameraFPS = FPS.FPS15;
                    break;
                case 2:
                    deviceConfiguration.CameraFPS = FPS.FPS30;
                    break;
            }


            
            device.StartCameras(deviceConfiguration);
            calibration = device.GetCalibration();
            transformation = new Transformation(calibration);

            Capture capture = device.GetCapture();
            if (capture == null)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "Could not get capture");
                return false;
            }

            pointCloud = new Image(ImageFormat.Custom, capture.Depth.WidthPixels, capture.Depth.HeightPixels, capture.Depth.WidthPixels * 3 * sizeof(Int16));
            colorsInDepth = new Image(ImageFormat.ColorBGRA32, capture.Depth.WidthPixels, capture.Depth.HeightPixels, capture.Depth.WidthPixels * 4 * sizeof(byte));


            return true;
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object can be used to retrieve data from input parameters and 
        /// to store data in output parameters.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            bool isActive = false;
            int fps = 0;
            int depthMode = 3;
            bool colorOutput = true;
            int colorMode = 0;
           

            if (!DA.GetData(0, ref isActive)) return;
            if (!DA.GetData(1, ref fps)) return;
            if (!DA.GetData(2, ref depthMode)) return;
            if (!DA.GetData(3, ref colorOutput)) return;
            if (!DA.GetData(4, ref colorMode)) return;

            if (fps != oldFPS || depthMode != oldDepthMode || colorMode != oldColorMode)
            {
                TeardownKinect();

                if (fps > 2)
                {
                    AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "FPS options 0-2");
                    return;
                }
                if (depthMode > 3)
                {
                    AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Depth mode options 0-4");
                    return;
                }

                if (colorMode > 4)
                {
                    AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Color mode options 0-4");
                    return;
                }

                if (depthMode == 3 && fps == 2)
                {
                    AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "NFOV_Unbinned only works with up to 15 FPS");
                    TeardownKinect();
                    return;
                }

                oldFPS = fps;
                oldDepthMode = depthMode;
                oldColorMode = colorMode;
            }

            if (!isActive)
            {
                TeardownKinect();
                return;
            }

            if (!isSetup)
            {
                if (!SetupKinect(fps, depthMode, colorMode))
                    return;
                
                isSetup = true;
            }


            Capture capture = device.GetCapture();
            if (capture == null)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "Could not get capture");
                return;
            }

            List<GH_Point> points = new List<GH_Point>();
            List<GH_Colour> colors = new List<GH_Colour>();
            if (colorOutput)
            {
                outputWithColor(capture, points, colors);
            } else
            {
                outputNoColor(capture, points);
            }

            DA.SetDataList(0, points);
            DA.SetDataList(1, colors);
            DA.SetDataList(2, calibration.DepthCameraCalibration.Intrinsics.Parameters.Take(calibration.DepthCameraCalibration.Intrinsics.ParameterCount));
            DA.SetDataList(3, calibration.ColorCameraCalibration.Intrinsics.Parameters.Take(calibration.ColorCameraCalibration.Intrinsics.ParameterCount));

        }

        void outputWithColor(Capture capture, List<GH_Point> outPoints, List<GH_Colour> outColors)
        {
            transformation.ColorImageToDepthCamera(capture, colorsInDepth);
            transformation.DepthImageToPointCloud(capture.Depth, pointCloud);


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

        void outputNoColor(Capture capture, List<GH_Point> outPoints)
        {
            transformation.DepthImageToPointCloud(capture.Depth, pointCloud);

            Memory<Short3> pointCloudMemory = pointCloud.GetPixels<Short3>();
            Span<Short3> pcs = pointCloudMemory.Span;

             int pixels = pointCloud.WidthPixels * pointCloud.HeightPixels;
            for (int i = 0; i < pixels; i++)
            {
                if ((pcs[i].X == 0 && pcs[i].Y == 0 && pcs[i].Z == 0) )
                    continue;

                outPoints.Add(new GH_Point(new Point3d(pcs[i].X, -pcs[i].Y, -pcs[i].Z)));
            }
        }

        public override Guid ComponentGuid
        {
            get { return new Guid("6796fb20-84e2-40ea-8e63-dca1727842bb"); }
        }

        public override void RemovedFromDocument(GH_Document document)
        {
            base.RemovedFromDocument(document);
            if (isSetup)
            {
                device.StopCameras();
                device.Dispose();
                isSetup = false;
            }
        }

        public override void DocumentContextChanged(GH_Document document, GH_DocumentContext context)
        {
            base.DocumentContextChanged(document, context);

            if (context == GH_DocumentContext.Close || context == GH_DocumentContext.Unloaded)
            {
                if (isSetup)
                {
                    device.StopCameras();
                    device.Dispose();
                    isSetup = false;
                }
            }
        }


    }
}
