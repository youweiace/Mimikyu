using Grasshopper.Kernel;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using Microsoft.Azure.Kinect.Sensor;
using Grasshopper.Kernel.Types;
using Color = System.Drawing.Color;
using Grasshopper.Kernel.Parameters;
using System.Linq;
using Mimikyu.Kinect.Properties;

namespace KinectAzureDK
{
    public class KinectAzure : GH_Component
    {

        bool isSetup = false;
        Device device = null;
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
        public KinectAzure()
          : base("Azure Kinect", "Kinect",
            "Receive data from a Azure Kinect",
            "Kinect", "Azure Kinect")
        {
        }

        protected override System.Drawing.Bitmap Icon => Resources.AzureKinectIcon;

        public override Guid ComponentGuid => new Guid("0cf8d01f-5ed8-45f5-a574-178ba2f48a92");
       

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
            pManager.AddGenericParameter("Device", "D", "The azure device object", GH_ParamAccess.item);
            pManager.AddGenericParameter("Calibration", "c", "Calibration of the device", GH_ParamAccess.item);
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
                case 4:
                    deviceConfiguration.DepthMode = DepthMode.PassiveIR;
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
                if (depthMode > 4)
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



            DA.SetData(0, device);
            DA.SetData(1, calibration);

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
