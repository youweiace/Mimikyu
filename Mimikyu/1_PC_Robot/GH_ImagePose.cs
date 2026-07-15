using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Geometry.Delaunay;
using Rhino.Geometry;
using Rhino.Geometry.Collections;
using Rhino.Render.ChangeQueue;
using System;
using System.Collections.Generic;
using System.Linq;
using static Rhino.Render.TextureGraphInfo;
using static System.Windows.Forms.DataFormats;
using Mesh = Rhino.Geometry.Mesh;

namespace Mimikyu._1_PC_Robot
{
    public class GH_ImagePose : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the GH_ImagePose class.
        /// </summary>
        public GH_ImagePose()
          : base("ImagePose", "ImPo",
              "View-planning / scan-plane generation algorithm",
              "Mimikyu", "PC_Robot")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddBrepParameter("Object", "Obj", "Object to define scan area", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddPlaneParameter("Poses", "P", "Pose as Planes", GH_ParamAccess.tree);
            pManager.AddBoxParameter("Box", "B", "Box", GH_ParamAccess.item);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Brep inGeo = default;
            double CaptureW = 428;
            double CaptureH = 330;
            double Distance = 500;
            double Overlap = 0.10;

            DataTree<Plane> planeTree = new DataTree<Plane>();

            if (!DA.GetData(0, ref inGeo)) return;

            Mesh mesh = BrepToSingleMesh(inGeo);

            Box obb = GetMinimumBoundingBox3D(mesh);
            Plane objectPlane = obb.Plane;

            double sx = obb.X.Length;
            double sy = obb.Y.Length;
            double sz = obb.Z.Length;

            Point3d center = obb.Center;

            Point3d frontCenter =
                center - objectPlane.YAxis * (sy * 0.5);

            Point3d backCenter =
                center + objectPlane.YAxis * (sy * 0.5);

            Point3d rightCenter =
                center + objectPlane.XAxis * (sx * 0.5);

            Point3d leftCenter =
                center - objectPlane.XAxis * (sx * 0.5);

            Point3d topCenter =
                center + objectPlane.ZAxis * (sz * 0.5);

            Point3d bottomCenter =
                center - objectPlane.ZAxis * (sz * 0.5);

            List<ScanFace> faces = new List<ScanFace>();

            //--------------------------------------------------
            // FRONT
            //--------------------------------------------------

            faces.Add(new ScanFace()
            {
                Center = frontCenter,
                Normal = -objectPlane.YAxis,
                UAxis = objectPlane.XAxis,
                VAxis = objectPlane.ZAxis,
                Width = sx,
                Height = sz
            });

            //--------------------------------------------------
            // BACK
            //--------------------------------------------------

            faces.Add(new ScanFace()
            {
                Center = backCenter,
                Normal = objectPlane.YAxis,
                UAxis = -objectPlane.XAxis,
                VAxis = objectPlane.ZAxis,
                Width = sx,
                Height = sz
            });

            //--------------------------------------------------
            // RIGHT
            //--------------------------------------------------

            faces.Add(new ScanFace()
            {
                Center = rightCenter,
                Normal = objectPlane.XAxis,
                UAxis = objectPlane.YAxis,
                VAxis = objectPlane.ZAxis,
                Width = sy,
                Height = sz
            });

            //--------------------------------------------------
            // LEFT
            //--------------------------------------------------

            faces.Add(new ScanFace()
            {
                Center = leftCenter,
                Normal = -objectPlane.XAxis,
                UAxis = -objectPlane.YAxis,
                VAxis = objectPlane.ZAxis,
                Width = sy,
                Height = sz
            });

            //--------------------------------------------------
            // TOP
            //--------------------------------------------------

            faces.Add(new ScanFace()
            {
                Center = topCenter,
                Normal = objectPlane.ZAxis,
                UAxis = -objectPlane.XAxis,
                VAxis = objectPlane.YAxis,
                Width = sx,
                Height = sy
            });

            //--------------------------------------------------
            // BOTTOM
            //--------------------------------------------------

            faces.Add(new ScanFace()
            {
                Center = bottomCenter,
                Normal = -objectPlane.ZAxis,
                UAxis = objectPlane.XAxis,
                VAxis = -objectPlane.YAxis,
                Width = sx,
                Height = sy
            });


            double stepU = CaptureH * (1.0 - Overlap);
            double stepV = CaptureW * (1.0 - Overlap);

            for (int f = 0; f < faces.Count; f++)
            {
                ScanFace face = faces[f];

                int countU = Math.Max(1,
                    (int)Math.Ceiling(face.Width / stepU));

                int countV = Math.Max(1,
                    (int)Math.Ceiling(face.Height / stepV));

                for (int row = 0; row < countV; row++)
                {
                    bool reverse = (row % 2 == 1);

                    for (int colIter = 0; colIter < countU; colIter++)
                    {
                        int col = reverse
                            ? countU - 1 - colIter
                            : colIter;

                        double u;
                        double v;

                        if (countU == 1)
                            u = 0.5;
                        else
                            u = (double)col / (countU - 1);

                        if (countV == 1)
                            v = 0.5;
                        else
                            v = (double)row / (countV - 1);

                        Point3d facePoint = face.PointAt(u, v);

                        Point3d camOrigin =
                            facePoint + face.Normal * Distance;

                        Vector3d zAxis = face.Normal;
                        zAxis.Unitize();

                        Vector3d xAxis = face.UAxis;
                        xAxis.Unitize();

                        Vector3d yAxis =
                            Vector3d.CrossProduct(zAxis, xAxis);
                        yAxis.Unitize();

                        xAxis =
                            Vector3d.CrossProduct(yAxis, zAxis);
                        xAxis.Unitize();

                        Plane camPlane =
                            new Plane(camOrigin, xAxis, yAxis);

                        GH_Path path = new GH_Path(f);
                        planeTree.Add(camPlane, path);


                    }
                }
            }

            DA.SetDataTree(0, planeTree);
            DA.SetData(1, obb);
        }
        public class ScanFace
        {

            public Point3d Center;
            public Vector3d Normal;
            public Vector3d UAxis;
            public Vector3d VAxis;
            public double Width;
            public double Height;

            public Point3d PointAt(double u, double v)
            {
                double uu = (u - 0.5) * Width;
                double vv = (v - 0.5) * Height;

                return Center + UAxis * uu + VAxis * vv;
            }
        }
        private static Mesh BrepToSingleMesh(Brep brep)
        {
            Mesh[] meshes = Mesh.CreateFromBrep(brep, MeshingParameters.Default);

            Mesh joined = new Mesh();

            if (meshes == null || meshes.Length == 0)
                return joined;

            foreach (Mesh m in meshes)
            {
                if (m != null)
                    joined.Append(m);
            }

            joined.Vertices.CombineIdentical(true, true);
            joined.Vertices.CullUnused();
            joined.UnifyNormals();
            joined.Normals.ComputeNormals();
            joined.Compact();

            return joined;
        }

        public static Box GetMinimumBoundingBox3D(Rhino.Geometry.Mesh inputMesh)
        {

            // Note: The inputMesh is already a convex hull

            MeshFaceList faces = inputMesh.Faces;

            List<Plane> planes = new List<Plane>();

            // Get all the possible planes
            foreach (MeshFace face in faces)
            {
                List<Point3d> pts = new List<Point3d>();
                pts.Add(inputMesh.Vertices[face.A]);
                pts.Add(inputMesh.Vertices[face.B]);
                pts.Add(inputMesh.Vertices[face.C]);
                Plane tempPlane = new Plane();
                if (Plane.FitPlaneToPoints(pts, out tempPlane) == PlaneFitResult.Success)
                    planes.Add(tempPlane);
            }

            List<Box> orientedBoxes = new List<Box>();

            foreach (Plane pln in planes)
            {
                Box bb = new Box();
                inputMesh.GetBoundingBox(pln, out bb);
                orientedBoxes.Add(bb);
            }

            // Sort the bounding boxes by volume
            List<Box> SortedBoudningBoxes = orientedBoxes.OrderBy(o => o.Volume).ToList();

            // Return the smallest one
            return SortedBoudningBoxes[0];
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
            get { return new Guid("8D1F0AFF-37E9-4B8C-932B-A5F3C14CBD9B"); }
        }
    }
}