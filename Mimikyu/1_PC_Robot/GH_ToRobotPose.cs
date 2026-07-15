using Grasshopper.Kernel;
using Mimikyu.Helper;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text.RegularExpressions;

namespace Mimikyu.Utlilities
{
    public class GH_ToRobotPose : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the GH_ToRobotPose class.
        /// </summary>
        public GH_ToRobotPose()
          : base("ToRobotPose", "TR",
              "Convert to Rhino Position to Robot Pose",
              "Mimikyu", "PC_Robot")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddTextParameter("PoseString", "PS", "Robot Pose String", GH_ParamAccess.list);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("RobotPose", "RP", "Robot Pose", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            List<string> inPose = new List<string>();
            DA.GetDataList(0, inPose);

            List<RobotPose> outPose = new List<RobotPose>();
            foreach (string pose in inPose)
            {

                var values = new Dictionary<string, double>();

                foreach (Match m in Regex.Matches(pose, @"([A-Z]\d*)\s+([-\d.]+)"))
                {
                    values[m.Groups[1].Value] = double.Parse(
                        m.Groups[2].Value,
                        CultureInfo.InvariantCulture);
                }

                RobotPose robotPose = new RobotPose
                {
                    X = values["X"],
                    Y = values["Y"],
                    Z = values["Z"],
                    A = values["A"],
                    B = values["B"],
                    C = values["C"],
                    E1 = values["E1"],
                    E2 = values["E2"],
                    E3 = values["E3"],
                    E4 = values["E4"]
                };
                outPose.Add(robotPose);
            }

            DA.SetDataList(0, outPose);
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
            get { return new Guid("3DFE7198-0A59-441D-840E-18418220D883"); }
        }
    }
}