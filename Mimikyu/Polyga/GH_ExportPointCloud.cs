using Grasshopper.Kernel;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.Text;

namespace Mimikyu.Polyga
{
    public class GH_ExportPointCloud : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the GH_SavePointCloud class.
        /// </summary>
        public GH_ExportPointCloud()
          : base("ExportPC", "E",
              "Export Point Cloud",
              "Mimikyu", "Polyga")
        {
        }

        string lastWrittenPath = "";

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddTextParameter("Dir", "D", "Directory to save to.", GH_ParamAccess.item);
            pManager.AddTextParameter("Filename", "F", "Filename to be written.", GH_ParamAccess.item);
            pManager.AddPointParameter("Vertices", "V", "Vertices to save.", GH_ParamAccess.list);
            pManager.AddColourParameter("Colors", "C", "Colors of each vertex. Lengths must match.", GH_ParamAccess.list);
            pManager[3].Optional = true;
            pManager.AddBooleanParameter("Active", "A", "Set to true to save.", GH_ParamAccess.item);
            pManager.AddBooleanParameter("ColorsAsInt", "CI", "Set to true to save colors as int.", GH_ParamAccess.item, true);
            pManager[5].Optional = true;
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddTextParameter("Path", "P", "The path the data was written to.", GH_ParamAccess.item);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            string dir = "";
            string filename = "";
            List<Point3d> vertices = new List<Point3d>();
            List<Color> colors = new List<Color>();
            bool isActive = false;
            bool colorsAsInt = true;

            if (!DA.GetData(0, ref dir)) return;
            if (!DA.GetData(1, ref filename)) return;
            if (!DA.GetDataList(2, vertices)) return;
            bool gotColors = DA.GetDataList(3, colors);
            if (!DA.GetData(4, ref isActive)) return;
            DA.GetData(5, ref colorsAsInt);

            if (!isActive)
            {
                DA.SetData(0, lastWrittenPath);
                return;
            }

            if (!Directory.Exists(dir))
            {
                Directory.CreateDirectory(dir);
                AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "Directory did not exist, created directory.");
                return;
            }

            string path = Path.Combine(dir, filename);
            bool outputColors = vertices.Count == colors.Count;

            if (!outputColors && gotColors)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "Length of vertices and colors mismatch. Will not output colors.");
            }

            try
            {
                WritePlyFile(path, vertices, colors, colorsAsInt);
                lastWrittenPath = path;
            }
            catch (Exception e)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Error: " + e.Message);
            }

            DA.SetData(0, lastWrittenPath);
        }

        private void WritePlyFile(string path, List<Point3d> vertices, List<Color> colors, bool colorsAsInt)
        {

            bool outputColors = vertices.Count == colors.Count;

            const int BufferSize = 512000;  // 512 Kilobytes
            using (StreamWriter sw = new StreamWriter(path, false, new ASCIIEncoding(), BufferSize))
            {

                sw.NewLine = "\n";
                sw.WriteLine("ply");
                sw.WriteLine("format ascii 1.0");
                sw.WriteLine(String.Format("element vertex {0}", vertices.Count));
                sw.WriteLine("property float32 x");
                sw.WriteLine("property float32 y");
                sw.WriteLine("property float32 z");

                if (outputColors)
                {
                    if (colorsAsInt)
                    {
                        sw.WriteLine("property uchar red");
                        sw.WriteLine("property uchar green");
                        sw.WriteLine("property uchar blue");

                    }
                    else
                    {
                        sw.WriteLine("property float32 red");
                        sw.WriteLine("property float32 green");
                        sw.WriteLine("property float32 blue");
                    }

                }
                sw.WriteLine("end_header");

                int vertexCount = vertices.Count;
                if (outputColors)
                {

                    for (int i = 0; i < vertexCount; i++)
                    {
                        sw.Write(vertices[i].X);
                        sw.Write(" ");
                        sw.Write(vertices[i].Y);
                        sw.Write(" ");
                        sw.Write(vertices[i].Z);
                        sw.Write(" ");

                        var r = Math.Pow(colors[i].R / 255.0f, (1.0 / 2.2)) * 255;
                        var g = Math.Pow(colors[i].G / 255.0f, (1.0 / 2.2)) * 255;
                        var b = Math.Pow(colors[i].B / 255.0f, (1.0 / 2.2)) * 255;


                        if (colorsAsInt)
                        {
                            sw.Write((int)r);
                            sw.Write(" ");
                            sw.Write((int)g);
                            sw.Write(" ");
                            sw.WriteLine((int)b);
                        }
                        else
                        {
                            sw.Write(r / 255.0f);
                            sw.Write(" ");
                            sw.Write(g / 255.0f);
                            sw.Write(" ");
                            sw.WriteLine(b / 255.0f);
                        }
                    }
                }
                else
                {
                    for (int i = 0; i < vertexCount; i++)
                    {
                        sw.Write(vertices[i].X);
                        sw.Write(" ");
                        sw.Write(vertices[i].Y);
                        sw.Write(" ");
                        sw.WriteLine(vertices[i].Z);
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
            get { return new Guid("2B03090C-D679-45DD-9DBA-308198CAAD75"); }
        }
    }
}