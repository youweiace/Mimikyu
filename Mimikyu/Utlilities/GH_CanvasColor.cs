using System;
using System.Collections.Generic;

using Grasshopper;
using Grasshopper.Kernel;
using Rhino.Geometry;
using System.Drawing;
using Mimikyu.Properties;

namespace Mimikyu.Utlilities
{
    public class GH_CanvasColor : GH_Component
    {
        /// <summary>
        /// Each implementation of GH_Component must provide a public 
        /// constructor without any arguments.
        /// Category represents the Tab in which the component will appear, 
        /// Subcategory the panel. If you use non-existing tab or panel names, 
        /// new tabs/panels will automatically be created.
        /// </summary>
        public GH_CanvasColor()
          : base("O_O", "O_O",
            "Change Canvas Color",
            "Mimikyu", "Utilities")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddBooleanParameter("Change", "Ch", "Change Canvas Color", GH_ParamAccess.item, false);
            pManager.AddColourParameter("Canvas", "C", "Canvas Color", GH_ParamAccess.item);
            pManager.AddColourParameter("Grid", "G", "Grid Color", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object can be used to retrieve data from input parameters and 
        /// to store data in output parameters.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            bool switcher = false;
            Color canvas = Color.FromArgb(255, 212, 208, 200);
            Color grid = Color.FromArgb(30, 0, 0, 0);

            DA.GetData(0, ref switcher);
            DA.GetData(1, ref canvas);
            DA.GetData(2, ref grid);

            if (switcher == true)
            {
                Grasshopper.GUI.Canvas.GH_Skin.canvas_grid = grid;
                Grasshopper.GUI.Canvas.GH_Skin.canvas_back = canvas;
                Grasshopper.GUI.Canvas.GH_Skin.canvas_edge = Color.FromArgb(255, 0, 0, 0);
                Grasshopper.GUI.Canvas.GH_Skin.canvas_shade = Color.FromArgb(80, 0, 0, 0);
            }
            else
            {
                //DEFAULTS
                Grasshopper.GUI.Canvas.GH_Skin.canvas_grid = Color.FromArgb(30, 0, 0, 0);
                Grasshopper.GUI.Canvas.GH_Skin.canvas_back = Color.FromArgb(255, 212, 208, 200);
                Grasshopper.GUI.Canvas.GH_Skin.canvas_edge = Color.FromArgb(255, 0, 0, 0);
                Grasshopper.GUI.Canvas.GH_Skin.canvas_shade = Color.FromArgb(80, 0, 0, 0);
            }
        }

        /// <summary>
        /// Provides an Icon for every component that will be visible in the User Interface.
        /// Icons need to be 24x24 pixels.
        /// You can add image files to your project resources and access them like this:
        /// return Resources.IconForThisComponent;
        /// </summary>
        protected override Bitmap Icon => Resources.Mimikyu;
        

        /// <summary>
        /// Each component must have a unique Guid to identify it. 
        /// It is vital this Guid doesn't change otherwise old ghx files 
        /// that use the old ID will partially fail during loading.
        /// </summary>
        public override Guid ComponentGuid => new Guid("2c7d51f0-0adf-450d-8fb4-3fd930672a84");
    }
}