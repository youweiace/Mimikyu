using Grasshopper.Kernel;
using Rhino.Geometry;
using System;
using Microsoft.Azure.Kinect.Sensor;
using Grasshopper.Kernel.Types;
using Color = System.Drawing.Color;
using System.Drawing;
using System.Runtime.InteropServices;
using System.Drawing.Imaging;
using System.Linq;
using Mimikyu.Kinect.Properties;

namespace KinectAzureDK
{
    public class KinectAzureExplodeCalibration : GH_Component
    {
        /// <summary>
        /// Each implementation of GH_Component must provide a public 
        /// constructor without any arguments.
        /// Category represents the Tab in which the component will appear, 
        /// Subcategory the panel. If you use non-existing tab or panel names, 
        /// new tabs/panels will automatically be created.
        /// </summary>
        public KinectAzureExplodeCalibration()
          : base("Explode Calibration", "ExCal",
            "Reveals the intrinisc of the current calibration.",
            "Kinect", "Azure Kinect")
        {
        }

        protected override System.Drawing.Bitmap Icon => Resources.AzureKinectIcon;

        public override Guid ComponentGuid => new Guid("293d71f5-c8e1-4e29-af72-4235bba5520e");

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Calibration", "c", "Calibration object", GH_ParamAccess.item);


        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddNumberParameter("Color Intrinsics", "CCI", "Color Camera Intrinsics", GH_ParamAccess.list);
            pManager.AddNumberParameter("Depth Intrinsics", "DCI", "Depth Camera Intrinsics", GH_ParamAccess.list);

        }


        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object can be used to retrieve data from input parameters and 
        /// to store data in output parameters.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Calibration calibration = new Calibration();
            if (!DA.GetData(0, ref calibration)) return;

            DA.SetDataList(0, calibration.ColorCameraCalibration.Intrinsics.Parameters.Take(calibration.ColorCameraCalibration.Intrinsics.ParameterCount));
            DA.SetDataList(1, calibration.DepthCameraCalibration.Intrinsics.Parameters.Take(calibration.DepthCameraCalibration.Intrinsics.ParameterCount));


        }

    }
}
