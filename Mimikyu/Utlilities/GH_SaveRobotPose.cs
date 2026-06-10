using Grasshopper.Kernel;
using Mimikyu.Helper;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Globalization;

namespace Mimikyu.Utlilities
{
    public class GH_SaveRobotPose : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the SaveRobotPose class.
        /// </summary>
        public GH_SaveRobotPose()
          : base("SaveRobotPose", "Nickname",
              "Description",
              "Category", "Subcategory")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("RobotPose", "P", "The robot pose to save.", GH_ParamAccess.list);
            pManager.AddBooleanParameter("Save", "S", "Trigger to save the robot pose.", GH_ParamAccess.item);
            pManager.AddTextParameter("FilePath", "F", "The file path to save the robot pose.", GH_ParamAccess.item, "robot_poses.txt");
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
            List<RobotPose> robotPose = null;
            bool save = false;
            string filePath = null;

            if (!DA.GetDataList(0, robotPose)) return;
            if (!DA.GetData(1, ref save)) return;
            if(!DA.GetData(2, ref filePath)) return;

            if (save)
            {
                RobotPoseWriter.SaveMany(robotPose, filePath, true);
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
            get { return new Guid("334FAECE-AB22-4335-BBED-7DF326751F12"); }
        }

    }
}