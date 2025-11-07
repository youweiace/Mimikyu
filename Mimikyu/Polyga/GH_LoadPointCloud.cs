using Grasshopper.Kernel;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;

namespace Mimikyu.Polyga
{
    public class GH_LoadPointCloud : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the GH_LoadPointCloud class.
        /// </summary>
        public GH_LoadPointCloud()
          : base("LoadPC", "L",
              "Load Point Cloud",
              "Mimikyu", "Polyga")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddTextParameter("Path", "P", "Path to read.", GH_ParamAccess.item);
            pManager.AddBooleanParameter("Active", "A", "Set to true to load.", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddPointParameter("Vertices", "V", "Vertices to save.", GH_ParamAccess.list);
            pManager.AddColourParameter("Colors", "C", "Colors of each vertex. Lengths must match.", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            string path = "";
            bool isActive = false;

            if (!DA.GetData(0, ref path)) return;
            if (!DA.GetData(1, ref isActive)) return;


            if (!File.Exists(path))
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Path does not exist.");
                return;
            }

            if (!isActive)
            {
                // if (lastError != null) {
                //      AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Error: " + lastError.Message);
                //  }
                //   else
                //  {
                //    DA.SetDataList(0, lastReadVertices);
                //     DA.SetDataList(1, lastReadColors);
                //   }
                return;
            }
            List<Color> colors;
            List<Point3d> vertices;
            try
            {
                ReadPlyFile(path, out vertices, out colors);
            }
            catch (Exception e)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Error: " + e.Message);
                //lastError = e;
                return;
            }

            //lastError = null;
            DA.SetDataList(0, vertices);
            DA.SetDataList(1, colors);
        }

        private void ReadPlyFile(string path, out List<Point3d> vertices, out List<Color> colors)
        {

            bool colorsInFile = false;
            bool colorsFloat = false;


            using (StreamReader sr = new StreamReader(path))
            {

                if (sr.ReadLine() != "ply")
                    throw new ArgumentException("File not a ply file: " + path);

                if (sr.ReadLine() != "format ascii 1.0")
                    throw new ArgumentException("File not a ASCII 1.0 format: " + path);

                var line = sr.ReadLine();


                int numVertices = Convert.ToInt32(line.Split(' ')[2]);
                vertices = new List<Point3d>(numVertices);
                colors = new List<Color>(numVertices);

                if (sr.ReadLine() != "property float32 x")
                    throw new ArgumentException("File misformatted: " + path);

                if (sr.ReadLine() != "property float32 y")
                    throw new ArgumentException("File misformatted: " + path);

                if (sr.ReadLine() != "property float32 z")
                    throw new ArgumentException("File misformatted: " + path);

                var redLine = sr.ReadLine();
                if (redLine == "property uchar red")
                {
                    colorsInFile = true;
                    colorsFloat = false;
                }
                else if (redLine == "property uchar float32")
                {
                    colorsInFile = true;
                    colorsFloat = true;
                }
                else
                {
                    colorsInFile = false;
                }

                sr.ReadLine();
                sr.ReadLine();

                if (sr.ReadLine() != "end_header")
                    throw new ArgumentException("File misformatted: " + path);

                for (var i = 0; i < numVertices; i++)
                {
                    line = sr.ReadLine();
                    if (line == null)
                        throw new ArgumentException("File too short: " + path);

                    var parts = line.Split(' ');
                    var x = Convert.ToDouble(parts[0]);
                    var y = Convert.ToDouble(parts[1]);
                    var z = Convert.ToDouble(parts[2]);
                    vertices.Add(new Point3d(x, y, z));

                    if (colorsInFile)
                    {
                        var r = Convert.ToDouble(parts[3]);
                        var g = Convert.ToDouble(parts[4]);
                        var b = Convert.ToDouble(parts[5]);

                        if (!colorsFloat)
                        {
                            r /= 255.0;
                            g /= 255.0;
                            b /= 255.0;
                        }

                        r = Math.Pow(r, 2.2) * 255;
                        g = Math.Pow(g, 2.2) * 255;
                        b = Math.Pow(b, 2.2) * 255;
                        colors.Add(Color.FromArgb((int)r, (int)g, (int)b));
                    }
                }
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
            get { return new Guid("DB9E2779-9C9A-4F33-9C7B-63C32BB5E92F"); }
        }
    }
}