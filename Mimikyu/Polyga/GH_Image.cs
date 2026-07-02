using Grasshopper.Kernel;
using Rhino.Geometry;
using SBSDKNet3;
using System;
using System.Collections.Generic;
using static Rhino.Render.ChangeQueue.Light;

namespace Mimikyu.Polyga
{
    public class GH_Image : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the GH_Calibration class.
        /// </summary>
        public GH_Image()
          : base("Image", "I",
              "Polyga Left Right Camera Image",
              "Mimikyu", "Polyga")
        {
        }

        private SBScanner scanner = null;
        int filename = 0;
        int eventId = 0;
        int lastId = 0;

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddBooleanParameter("Connect", "C", "Connect the Camera", GH_ParamAccess.item);
            pManager.AddIntegerParameter("EventId", "T", "Trigger the Camera", GH_ParamAccess.item);
            pManager.AddTextParameter("Directory", "D", "Save Directory", GH_ParamAccess.item);
            pManager.AddBooleanParameter("Reset", "R", "Reset Count", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            bool enabled = false;
            string directory = "";
            bool reset = false;

            if (!DA.GetData(0, ref enabled)) return;
            if (!DA.GetData(1, ref eventId)) return;
            if (!DA.GetData(2, ref directory)) return;
            if (!DA.GetData(3, ref reset)) return;


            if (!enabled)
            {
                if (scanner != null)
                {
                    try
                    {
                        scanner.disconnect();
                    }
                    catch (Exception ex)
                    {
                        AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Error disconnecting: " + ex.Message);
                    }
                }
                return;
            }
            if (scanner == null)
            {
                List<SBDeviceInfo> devList = SBFactory.getDevices();
                if (devList.Count == 0)
                {
                    AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "No camera found.");
                    return;
                }
                SBStatus status = SBStatus.DEVICE_NOT_CONNECTED;
                foreach (var dev in devList)
                {
                    scanner = SBFactory.createDevice(dev);
                    status = scanner.connect();
                    if (status == SBStatus.OK)
                    {
                        break;
                    }
                }
                if (status != SBStatus.OK)
                    AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Status not ok: " + status);
            }

            if (!scanner.isConnected())
            {
                SBStatus status = scanner.connect();
                if (status != SBStatus.OK)
                {
                    AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Status not ok: " + status);
                    return;
                }
            }

            if (reset)
            {
                filename = 0;
                eventId = 0;
                lastId = 0;
            }
            
            if (eventId > lastId)
            {
                SBScan scanResult = default;
                SBCaptureParams captureParams = new SBCaptureParams();

                captureParams.textureEnable = true;
                captureParams.textureExposure = 3.0;

                scanner.captureScanImages(out scanResult, captureParams);

                //left camera
                List<SBImage> leftImageList = scanResult.getCameraImages(0);
                SBImage leftTexture = scanResult.getTextureImage();
                //List<SBImage> rightImageList = scanResult.getCameraImages(1);

                leftImageList[0].save($"{directory}\\{filename}.jpg");
                leftTexture.save($"{directory}\\{filename}_texture.jpg");
                //rightImageList[0].save($"{directory}\\{filename}_right.jpg");

                filename += 1;
                lastId = eventId;
            }
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
            get { return new Guid("8E942AB6-A444-4EA2-B88D-62373EAFD81B"); }
        }
    }
}