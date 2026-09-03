using Grasshopper.Kernel;
using Mimikyu.Helper;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;

namespace Mimikyu._1_PC_Robot
{
    public class GH_SortPlanesSerpentine : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the GH_SortPlanesSerpentine class.
        /// </summary>
        public GH_SortPlanesSerpentine()
          : base("SortPlanesSerpentine", "SPS",
              "Serpentine (boustrophedon) ordering",
              "Mimikyu", "PC_Robot")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddPlaneParameter("Planes", "P", "Unordered Planes", GH_ParamAccess.list);
            pManager.AddPointParameter("CameraBase", "CB", "Height where camera should flip to the other side", GH_ParamAccess.item);
            pManager.AddIntegerParameter("SortBy", "S", "Sort by 0: X, 1: Y", GH_ParamAccess.item, 1);
            pManager[1].Optional = true;
            pManager[2].Optional = true;
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddPlaneParameter("Planes", "P", "Serpentine Planes", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            List<Plane> planes = new List<Plane>();
            Point3d cameraBase = Point3d.Unset;
            int sortBy = 1;

            if (!DA.GetDataList(0, planes)) return;
            DA.GetData(1, ref cameraBase);
            DA.GetData(2, ref sortBy);


            double rowTolerance = 10.0;
            List<Plane> sortedBy = new List<Plane>();
            // Group into rows
            List<List<Plane>> rows = new List<List<Plane>>();
            List<Plane> currentRow = new List<Plane>();
            // Sort by Y first
            switch (sortBy)
            {
                case 1: 
                     sortedBy = planes
                        .OrderBy(p => p.OriginY)
                        .ToList();

                    double currentY = sortedBy[0].OriginY;

                    foreach (var p in sortedBy)
                    {
                        if (Math.Abs(p.OriginY - currentY) <= rowTolerance)
                        {
                            currentRow.Add(p);
                        }
                        else
                        {
                            rows.Add(currentRow);

                            currentRow = new List<Plane>();
                            currentRow.Add(p);

                            currentY = p.OriginY;
                        }
                    }

                    rows.Add(currentRow);

                    break;
                case 0:
                    sortedBy = planes
                        .OrderBy(p => p.OriginX)
                        .ToList();

                    double currentX = sortedBy[0].OriginX;

                    foreach (var p in sortedBy)
                    {
                        if (Math.Abs(p.OriginX - currentX) <= rowTolerance)
                        {
                            currentRow.Add(p);
                        }
                        else
                        {
                            rows.Add(currentRow);

                            currentRow = new List<Plane>();
                            currentRow.Add(p);

                            currentX = p.OriginX;
                        }
                    }

                    rows.Add(currentRow);
                    break;
            }

            // Build serpentine path
            List<Plane> serpentinePlanes = new List<Plane>();

            for (int row = 0; row < rows.Count; row++)
            {
                var rowPlanes = rows[row]
                    .OrderByDescending(p => p.OriginZ)
                    .ToList();

                if (row % 2 == 1)
                    rowPlanes.Reverse();

                serpentinePlanes.AddRange(rowPlanes);
            }

            if (cameraBase != Point3d.Unset)
            {
                for (int i = 0; i < planes.Count; i++)
                {
                    Plane p = serpentinePlanes[i];
                    if (p.OriginZ < cameraBase.Z && p.OriginY > cameraBase.Y)
                    {
                        p.Rotate(-Math.PI*0.95, p.ZAxis);//negative to rotate the other way
                    }
                    else if (p.OriginZ < cameraBase.Z && p.OriginY < cameraBase.Y)
                    {
                        p.Rotate(-Math.PI*0.95, p.ZAxis);
                    }
                    serpentinePlanes[i] = p;

                }
            }

            DA.SetDataList(0, serpentinePlanes);
        
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
            get { return new Guid("BBA3C63D-ED0D-4B1B-ADC6-25999AD5B6B1"); }
        }
    }
}