using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;

namespace Mimikyu.Utlilities
{
    public class GH_DataRecorder : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the GH_DataRecorder class.
        /// </summary>
        public GH_DataRecorder()
          : base("DataRecorder", "R",
              "A data recorder that can be enabled and reset",
              "Mimikyu", "Utilities")
        {
        }

        List<Object> items = new List<Object>();

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Item", "I", "The item to record.", GH_ParamAccess.item);
            pManager.AddBooleanParameter("Trigger", "T", "If true stores the item.", GH_ParamAccess.item, false);
            pManager.AddBooleanParameter("Reset", "R", "Clears the storage.", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("Items", "Is", "List of items", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            object item = null;
            bool record = false;
            bool reset = false;

            if (!DA.GetData(1, ref record)) return;
            if (!DA.GetData(2, ref reset)) return;
            if (!DA.GetData(0, ref item)) record = false;

            if (reset)
            {
                items.Clear();
                //items = new List<object>(); 
            }
            else if (item != null && record)
            {
                items.Add(item);
            }

            DA.SetDataList(0, items);
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
            get { return new Guid("8E55A476-FB07-46B4-B78A-2A96216D8DF7"); }
        }
    }
}