using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Rhino.Geometry;
using Rhino.Render.ChangeQueue;
using SBSDKNet3;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;

namespace Mimikyu.Polyga
{
    public class GH_StartScanMotion : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the GH_PolygaControl class.
        /// </summary>
        public GH_StartScanMotion()
          : base("ScanMotion", "Mo",
              "Polyga V1 camera scan continuous motion",
              "Mimikyu", "Polyga")
        {
        }

        private SBScanner scanner = null;
        private ScanHandler handler = null;
        private bool isStreaming = false;
        private bool pendingUpdate = false;
        private int lastEventId = -1;
        private bool eventTriggered = false;
        List<GH_Point> pointCloud = new List<GH_Point>();
        List<GH_Colour> colors = new List<GH_Colour>();
        double maxExposure = 1000.0;
        double minExposure = 0.0;
        private readonly object lockObject = new object();

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddBooleanParameter("Enabled", "E", "Enable the camera", GH_ParamAccess.item, false);
            pManager.AddIntegerParameter("EventId", "EV", "Trigger when the event id changes", GH_ParamAccess.item, 0);
            pManager.AddBooleanParameter("Color", "C", "Captures color if true", GH_ParamAccess.item, false);
            pManager.AddNumberParameter("Color Exposure", "CE", "Exposure of color capture", GH_ParamAccess.item, 2);
            pManager.AddNumberParameter("Scanner Exposure", "SE", "Exposure of scanner", GH_ParamAccess.item, 2);
            pManager.AddPointParameter("Color White Balance 0-4", "CWB", "White balance of scanner", GH_ParamAccess.item, new Point3d(1.51, 1, 2.3));
            pManager[5].Optional = true;
            pManager.AddBooleanParameter("Colour Flash Enabled", "CFE", "Set to true to enable flash of colour capture", GH_ParamAccess.item, true);
            pManager[6].Optional = true;
            pManager.AddNumberParameter("Projector Brightness", "PB", "Brightness of the Projector", GH_ParamAccess.item, 2);
            pManager.AddNumberParameter("Camera Gain", "PG", "Gain of the Camera", GH_ParamAccess.item, 2);
            pManager.AddIntegerParameter("Exposure Steps", "ES", "Number of exposure steps", GH_ParamAccess.item);
            pManager.AddNumberParameter("Step Size", "SS", "Size of each exposure step", GH_ParamAccess.item);
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
            int eventId = 0;
            bool captureColor = false;
            double colorExposure = 2000;
            double scannerExposure = 10;
            Point3d whiteBalance = new Point3d(1.51, 1, 2.3);
            Boolean flashEnabled = true;
            double brightness = 1.0;
            double gain = 1.0;

            if (!DA.GetData(0, ref enabled)) return;
            if (!DA.GetData(1, ref eventId)) return;
            if (!DA.GetData(2, ref captureColor)) return;
            if (!DA.GetData(3, ref colorExposure)) return;
            if (!DA.GetData(4, ref scannerExposure)) return;
            DA.GetData(5, ref whiteBalance);
            DA.GetData(6, ref flashEnabled);
            if (!DA.GetData(7, ref brightness)) return;
            if (!DA.GetData(8, ref gain)) return;

            if (!enabled)
            {
                if (isStreaming)
                {
                    try { scanner.stopScanStream(); } catch { }
                    isStreaming = false;
                }
                eventTriggered = false;
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

            if (eventId != lastEventId)
            {
                lastEventId = eventId;
                eventTriggered = true;
            }


            try
            {
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
                SBScan scan = new SBScan();
                SBMesh mesh = new SBMesh();
                if (!isStreaming)
                {

                    handler = new ScanHandler(this);
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
                        scanner.setProjectorPattern(SBProjectorPatternEnums.FOCUS);
                    }
                    else
                    {
                        scanner.setProjectorPattern(SBProjectorPatternEnums.BLACK);
                        scanner.setExternalFlashForTexturePreview(true);

                    }

                    scanner.startScanStream(processParams, true, handler);
                    isStreaming = true;
                }
            }
            catch (DllNotFoundException ex)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "DLL not found: " + ex.Message + ". Ensure all native DLLs are in the same folder as the .gha file.");
            }
            catch (BadImageFormatException ex)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Architecture mismatch: " + ex.Message + ". Ensure you're building for x64.");
            }
            catch (Exception ex)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Error: " + ex.Message + "\nStack: " + ex.StackTrace);
            }
            List<GH_Point> outputPoints;
            List<GH_Colour> outputColors;
            lock (lockObject)
            {
                outputPoints = new List<GH_Point>(pointCloud);
                outputColors = new List<GH_Colour>(colors);
                pendingUpdate = false;
            }
            DA.SetDataList(0, outputPoints);
            DA.SetDataList(1, outputColors);
        }
        private class ScanHandler : SBScanHandler
        {
            private GH_StartScanMotion component;

            public ScanHandler(GH_StartScanMotion component)
            {
                this.component = component;
            }
            public void onScanCaptured(SBScanner scanner, SBScan outScan, SBMesh mesh)
            {
                if (mesh == null || component == null) return;
                if (!component.isStreaming) return;
                if (!component.IsEventTriggered()) return;

                try
                {
                    var vertices = mesh.getVertices();
                    var colorsReturned = mesh.getVertexColors();
                    if (vertices == null || colorsReturned == null) return;
                    if (vertices.Count == 0) return;

                    var newPoints = new List<GH_Point>(vertices.Count);
                    var newColors = new List<GH_Colour>(vertices.Count);

                    int count = Math.Min(vertices.Count, colorsReturned.Count);
                    for (var i = 0; i < count; i++)
                    {
                        newPoints.Add(new GH_Point(new Point3d(vertices[i].x, vertices[i].y, vertices[i].z)));
                        newColors.Add(new GH_Colour(Color.FromArgb(colorsReturned[i].r, colorsReturned[i].g, colorsReturned[i].b)));
                    }

                    lock (component.lockObject)
                    {
                        component.pointCloud = newPoints;
                        component.colors = newColors;

                        // Skip if an update is already scheduled
                        if (component.pendingUpdate) return;
                        component.pendingUpdate = true;
                    }

                    // ScheduleSolution with a delay lets the UI thread breathe
                    var doc = component.OnPingDocument();
                    if (doc != null)
                    {
                        doc.ScheduleSolution(100, d =>
                        {
                            component.ExpireSolution(false);
                        });
                    }

                    component.OnScanCompleted();
                }
                catch (Exception)
                {
                    // Silently ignore callback errors from stale/invalid mesh data
                }
            }
        }

        private void OnScanCompleted()
        {
            lock (lockObject)
            {
                eventTriggered = false;
            }
        }

        private bool IsEventTriggered()
        {
            lock (lockObject)
            {
                return eventTriggered;
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
            get { return new Guid("3c4d96bc-663f-49d5-999d-f056cbdd3f6c"); }
        }
    }
}