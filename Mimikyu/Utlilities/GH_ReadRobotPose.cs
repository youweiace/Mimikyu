using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Mimikyu.Helper;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Globalization;

namespace Mimikyu.Utlilities
{
    public class GH_ReadRobotPose : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the SaveRobotPose class.
        /// </summary>
        public GH_ReadRobotPose()
          : base("ReadRobotPose", "RP",
              "Read robot pose from local folder",
              "Mimikyu", "Utilities")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {

            pManager.AddBooleanParameter("Read", "S", "Trigger to save the robot pose.", GH_ParamAccess.item);
            pManager.AddTextParameter("FilePath", "F", "The file path to read the robot pose.", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("RobotPose", "P", "The robot pose.", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            List<RobotPose> robotPose = new List<RobotPose>();
            bool read = false;
            string filePath = null;

            if (!DA.GetData(0, ref read)) return;
            if (!DA.GetData(1, ref filePath)) return;


            if (read)
            {
                robotPose = RobotPoseReader.ReadMany(filePath);
            }

            DA.SetDataList(0, robotPose);
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
            get { return new Guid("2cc5e61d-cd5b-4c9d-8eb6-0606185d6ef9"); }
        }

    }
}