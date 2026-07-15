using Grasshopper.Kernel;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;

namespace Mimikyu.Pointcloud
{
    public class GH_LoadPointCloud : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the GH_LoadPointCloud class.
        /// </summary>
        public GH_LoadPointCloud()
          : base("LoadCloud", "L",
              "Load Point Cloud",
              "Mimikyu", "PointCloud")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddTextParameter("Path", "P", "Path to read.", GH_ParamAccess.list);
            pManager.AddBooleanParameter("Active", "A", "Set to true to load.", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("PointCloud", "PC", "Read Point Cloud", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            List<string> paths = new List<string>();
            bool isActive = false;

            if (!DA.GetDataList(0, paths)) return;
            if (!DA.GetData(1, ref isActive)) return;

            if (!isActive)
            {
                return;
            }

            List<PointCloud> pointClouds = new List<PointCloud>();

            for (int i = 0; i < paths.Count; i++)
            { 
                if (!File.Exists(paths[i]))
                {
                    AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Path does not exist.");
                    return;
                }

                List<Color> colors;
                List<Point3d> vertices;
                PointCloud pointCloud = new PointCloud();
                try
                {
                    ReadPlyFile(paths[i], out vertices, out colors);
                    pointCloud.AddRange(vertices, colors);
                }
                catch (Exception e)
                {
                    AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Error: " + e.Message);
                    return;
                }
                pointClouds.Add(pointCloud);
            }

            DA.SetDataList(0, pointClouds);

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
            get { return new Guid("ac9af98d-c8f4-4a91-bb5c-56fe440d2dfe"); }
        }
    }
}