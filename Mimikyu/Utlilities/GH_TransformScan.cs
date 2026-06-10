using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;

namespace Mimikyu.Utlilities
{
    public class GH_TransformScan : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the TransformScan class.
        /// </summary>
        public GH_TransformScan()
          : base("TransformScan", "TS",
              "Transorm ",
              "Category", "Subcategory")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Scan", "S", "The scan to transform.", GH_ParamAccess.list);
            pManager.AddGenericParameter("Pose", "P", "The robot pose to use from where scan was taken.", GH_ParamAccess.list);
            pManager.AddTextParameter("FilePath", "F", "File path to use as transformation", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("TransformedScan", "T", "The transformed scan.", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
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
            get { return new Guid("5C31F755-B003-409B-8B77-FF04E7403FF5"); }
        }
    }
}