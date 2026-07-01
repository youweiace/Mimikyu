using Grasshopper.Kernel;
using Rhino.Geometry;
using System;
using Microsoft.Azure.Kinect.Sensor;
using Grasshopper.Kernel.Types;
using Color = System.Drawing.Color;
using System.Drawing;
using System.Runtime.InteropServices;
using System.Drawing.Imaging;
using Mimikyu.Kinect.Helper;
using Mimikyu.Kinect.Properties;

namespace KinectAzureDK
{
    public class KinectAzureDepthColorize : GH_Component
    {
        /// <summary>
        /// Each implementation of GH_Component must provide a public 
        /// constructor without any arguments.
        /// Category represents the Tab in which the component will appear, 
        /// Subcategory the panel. If you use non-existing tab or panel names, 
        /// new tabs/panels will automatically be created.
        /// </summary>
        public KinectAzureDepthColorize()
          : base("Depth2Image", "D2I",
            "Retrieves the color image from a capture.",
            "Kinect", "Azure Kinect")
        {
        }

        protected override System.Drawing.Bitmap Icon => Resources.AzureKinectIcon;

        public override Guid ComponentGuid => new Guid("745aadae-e5e1-4817-bcd3-e07acedeae63");

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Depth Capture", "DC", "Image object", GH_ParamAccess.item);
            pManager.AddNumberParameter ("Min", "Mi", "The lower bound for visualization", GH_ParamAccess.item,0);
            pManager.AddNumberParameter("Max", "Ma", "The upper bound for visualization", GH_ParamAccess.item, 2000);

        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("Color Image", "CI", "Image as bitmap", GH_ParamAccess.item);

        }


        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object can be used to retrieve data from input parameters and 
        /// to store data in output parameters.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Microsoft.Azure.Kinect.Sensor.Image image = null;
            double min = 0;
            double max = 2000;
           
            if (!DA.GetData(0, ref image)) return;
            if (!DA.GetData(1, ref min)) return;
            if (!DA.GetData(2, ref max)) return;

            if(min >= max)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Max must be larger than Min");
                return;
            }


            if (image.Format == Microsoft.Azure.Kinect.Sensor.ImageFormat.Depth16 || image.Format == Microsoft.Azure.Kinect.Sensor.ImageFormat.IR16)
            {
                Bitmap bitmap = new Bitmap(image.WidthPixels, image.HeightPixels, PixelFormat.Format32bppArgb);
                AzureKienctHelpers.DepthImageToBitmap(image, bitmap, min, max);

                DA.SetData(0, bitmap);
            }


        }

    }
}
