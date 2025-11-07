using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Rhino.Geometry;
using SBSDKNet3;
using System;
using System.Collections.Generic;
using System.Drawing;

namespace Mimikyu.Polyga
{
    public class GH_PolygaControl : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the GH_PolygaControl class.
        /// </summary>
        public GH_PolygaControl()
          : base("PolygaControl", "C",
              "Control Polyga V1 camera",
              "Mimikyu", "Polyga")
        {
        }

        private SBScanner scanner = null;
        List<GH_Point> pointCloud = new List<GH_Point>();
        List<GH_Colour> colors = new List<GH_Colour>();
        double maxExposure = 1000.0;
        double minExposure = 0.0;

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddBooleanParameter("Enabled", "E", "Enable the camera", GH_ParamAccess.item, false);
            pManager.AddBooleanParameter("Trigger", "T", "Takes new capture if true", GH_ParamAccess.item, false);
            pManager.AddBooleanParameter("Color", "C", "Captures color if true", GH_ParamAccess.item, false);
            pManager.AddNumberParameter("Color Exposure", "CE", "Exposure of color capture", GH_ParamAccess.item, 2);
            pManager.AddNumberParameter("Scanner Exposure", "SE", "Exposure of scanner", GH_ParamAccess.item, 2);
            pManager.AddPointParameter("Color White Balance 0-4", "CWB", "White balance of scanner", GH_ParamAccess.item, new Point3d(1.51, 1, 2.3));
            pManager[5].Optional = true;
            pManager.AddBooleanParameter("Colour Flash Enabled", "CFE", "Set to true to enable flash of colour capture", GH_ParamAccess.item, true);
            pManager[6].Optional = true;
            pManager.AddNumberParameter("Projector Brightness", "PB", "Brightness of the Projector", GH_ParamAccess.item, 2);
            pManager.AddNumberParameter("Camera Gain", "PG", "Gain of the Camera", GH_ParamAccess.item, 2);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddPointParameter("Points", "P", "The points", GH_ParamAccess.list);
            pManager.AddColourParameter("Colors", "C", "The colors", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            bool enabled = false;
            bool trigger = false;
            bool captureColor = false;
            double colorExposure = 2000;
            double scannerExposure = 10;
            Point3d whiteBalance = new Point3d(1.51, 1, 2.3);
            Boolean flashEnabled = true;
            double brightness = 1.0;
            double gain = 1.0;

            if (!DA.GetData(0, ref enabled)) return;
            if (!DA.GetData(1, ref trigger)) return;
            if (!DA.GetData(2, ref captureColor)) return;
            if (!DA.GetData(3, ref colorExposure)) return;
            if (!DA.GetData(3, ref colorExposure)) return;
            if (!DA.GetData(4, ref scannerExposure)) return;
            DA.GetData(5, ref whiteBalance);
            DA.GetData(6, ref flashEnabled);
            if (!DA.GetData(7, ref brightness)) return;
            if (!DA.GetData(8, ref gain)) return;

            if (!enabled)
            {
                if (scanner != null)
                    scanner.disconnect();
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

            if (scannerExposure < minExposure || scannerExposure > maxExposure)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Camera exposure needs to be between: " + minExposure + "-" + maxExposure);
                return;
            }

            if (trigger)
            {
                SBMesh mesh = new SBMesh();


                SBProcessParams processParams = new SBProcessParams();
                SBCaptureParams captureParams = new SBCaptureParams();

                captureParams.textureEnable = captureColor;
                captureParams.textureExposure = colorExposure;

                scanner.setWhiteBalance((float)whiteBalance.X, (float)whiteBalance.Y, (float)whiteBalance.Z);

                scanner.setCameraExposure(scannerExposure);
                scanner.setCameraGain(gain);
                scanner.setProjectorBrightness(brightness);

                if (flashEnabled)
                {
                    scanner.setProjectorPattern(SBProjectorPatternEnums.WHITE);
                }
                else
                {
                    scanner.setProjectorPattern(SBProjectorPatternEnums.BLACK);
                    scanner.setExternalFlashForTexturePreview(true);

                }
                scanner.scan(out mesh, processParams, captureParams);




                if (mesh == null)
                {
                    AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Failed to  get mesh");
                }
                else
                {
                    var vertices = mesh.getVertices();
                    var colorsReturned = mesh.getVertexColors();

                    pointCloud = new List<GH_Point>();
                    colors = new List<GH_Colour>();

                    for (var i = 0; i < vertices.Count; i++)
                    {
                        pointCloud.Add(new GH_Point(new Point3d(vertices[i].x, vertices[i].y, vertices[i].z)));
                        colors.Add(new GH_Colour(Color.FromArgb(colorsReturned[i].r, colorsReturned[i].g, colorsReturned[i].b)));
                    }
                }
            }

            DA.SetDataList(0, pointCloud);
            DA.SetDataList(1, colors);
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
            get { return new Guid("03282557-ACEF-40AB-A4D3-ECE0F59085F2"); }
        }
    }
}