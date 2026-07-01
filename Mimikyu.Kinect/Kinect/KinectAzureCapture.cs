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
    public class KinectAzureCapture : GH_Component
    {

        Capture capture;

        /// <summary>
        /// Each implementation of GH_Component must provide a public 
        /// constructor without any arguments.
        /// Category represents the Tab in which the component will appear, 
        /// Subcategory the panel. If you use non-existing tab or panel names, 
        /// new tabs/panels will automatically be created.
        /// </summary>
        public KinectAzureCapture()
          : base("Capture", "Cap",
            "Receive data from a Azure Kinect",
            "Kinect", "Azure Kinect")
        {
        }

        public override Guid ComponentGuid => new Guid("3d6d94f7-c429-4ff8-8a53-67c14eb81531");


        protected override System.Drawing.Bitmap Icon => Resources.AzureKinectIcon;


        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Device", "D", "The kinect device object", GH_ParamAccess.item);
            pManager.AddBooleanParameter("Trigger", "T", "Takes new capture if true", GH_ParamAccess.item, false);

        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("Color Capture", "CC", "The color capture", GH_ParamAccess.item);
            pManager.AddGenericParameter("Depth Capture", "DC", "The depth capture", GH_ParamAccess.item);
        }




        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object can be used to retrieve data from input parameters and 
        /// to store data in output parameters.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Device device = null;
            bool trigger = false;
           

            if (!DA.GetData(0, ref device)) return;
            if (!DA.GetData(1, ref trigger)) return;

            if (trigger)
            {
                try { 
                    capture = device.GetCapture();
                }
                catch ( ObjectDisposedException)
                {
                    AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "Device is not valid anymore.");
                }
            }
            if (capture == null)
                return;

            if (capture.Color != null)
                DA.SetData(0, capture.Color.Reference());


            if (capture.Depth != null)
                DA.SetData(1, capture.Depth.Reference());
            else if( capture.IR != null)
                DA.SetData(1, capture.IR.Reference());
           
        }
    }
}
