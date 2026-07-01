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
    public class KinectAzureColorImage : GH_Component
    {
        /// <summary>
        /// Each implementation of GH_Component must provide a public 
        /// constructor without any arguments.
        /// Category represents the Tab in which the component will appear, 
        /// Subcategory the panel. If you use non-existing tab or panel names, 
        /// new tabs/panels will automatically be created.
        /// </summary>
        public KinectAzureColorImage()
          : base("Color Image", "CImage",
            "Retrieves the color image from a capture.",
            "Kinect", "Azure Kinect")
        {
        }

        protected override System.Drawing.Bitmap Icon => Resources.AzureKinectIcon;

        public override Guid ComponentGuid => new Guid("01bdd9a9-ad4d-4b84-9394-5456c776df89");

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Color capture", "CC", "Image object", GH_ParamAccess.item);

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
           
            if (!DA.GetData(0, ref image)) return;

            if (image.Format == Microsoft.Azure.Kinect.Sensor.ImageFormat.ColorBGRA32)
            {
                Bitmap bitmap = new Bitmap(image.WidthPixels, image.HeightPixels, PixelFormat.Format32bppArgb);
                AzureKienctHelpers.ColorImageToBitmap(image, bitmap);
                DA.SetData(0, bitmap);
            }
        }

    }
}
