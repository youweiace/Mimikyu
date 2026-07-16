using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Geometry.Delaunay;
using Rhino;
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
            pManager.AddNumberParameter("Capture Width", "W", "Width of the capture area in mm", GH_ParamAccess.item, 428);
            pManager.AddNumberParameter("Capture Height", "H", "Height of the capture area in mm", GH_ParamAccess.item, 330);
            pManager.AddNumberParameter("Distance", "D", "Distance from the object to the camera in mm", GH_ParamAccess.item, 500);
            pManager.AddIntegerParameter("Mode", "M", "Scanning mode\n 0: Face Scans\n 1: Oblique 2 Sides\n 2: Oblique 4 Corners (under development)", GH_ParamAccess.item);
            pManager[1].Optional = true;
            pManager[2].Optional = true;
            pManager[3].Optional = true;
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
            double CaptureW = default;
            double CaptureH = default;
            double Distance = default;
            double Overlap = 0.10;
            int mode = 0;

            DataTree<Plane> planeTree = new DataTree<Plane>();

            if (!DA.GetData(0, ref inGeo)) return;
            if (!DA.GetData(1, ref CaptureW)) return;
            if (!DA.GetData(2, ref CaptureH)) return;
            if (!DA.GetData(3, ref Distance)) return;
            if (!DA.GetData(4, ref mode)) return;

            Mesh mesh = BrepToSingleMesh(inGeo);

            Box obb = GetMinimumBoundingBox3D(mesh);
            Plane objectPlane = obb.Plane;

            double sx = obb.X.Length;
            double sy = obb.Y.Length;
            double sz = obb.Z.Length;
            Point3d center = obb.Center;

            switch (mode)
            {
                case 0:

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
                    break;

                case 1:

                    // Optional angle input later
                    double ObliqueAngleDeg = 45.0;


                    // ------------------------------------------------------------
                    // OBB dimensions and axes
                    // ------------------------------------------------------------

                    double[] sizes =
                    {
                        sx,
                        sy,
                        sz
                    };

                    Vector3d[] axes =
                    {
                        objectPlane.XAxis,
                        objectPlane.YAxis,
                        objectPlane.ZAxis
                    };

                    for (int i = 0; i < 3; i++)
                        axes[i].Unitize();


                    // ------------------------------------------------------------
                    // Find which OBB axis is most vertical
                    // ------------------------------------------------------------

                    double[] zAlign =
                    {
                        Math.Abs(axes[0] * Vector3d.ZAxis),
                        Math.Abs(axes[1] * Vector3d.ZAxis),
                        Math.Abs(axes[2] * Vector3d.ZAxis)
                    };

                    int verticalId = 0;

                    if (zAlign[1] > zAlign[verticalId])
                        verticalId = 1;

                    if (zAlign[2] > zAlign[verticalId])
                        verticalId = 2;


                    // ------------------------------------------------------------
                    // Remaining 2 axes
                    // ------------------------------------------------------------

                    List<int> remaining = new List<int>();

                    for (int i = 0; i < 3; i++)
                    {
                        if (i != verticalId)
                            remaining.Add(i);
                    }

                    int idA = remaining[0];
                    int idB = remaining[1];


                    // ------------------------------------------------------------
                    // Longest horizontal axis = scan direction
                    // Other horizontal axis = side direction
                    // ------------------------------------------------------------

                    int longId;
                    int sideId;

                    if (sizes[idA] >= sizes[idB])
                    {
                        longId = idA;
                        sideId = idB;
                    }
                    else
                    {
                        longId = idB;
                        sideId = idA;
                    }


                    // ------------------------------------------------------------
                    // Final object-aligned frame
                    // ------------------------------------------------------------

                    Vector3d lengthAxis = axes[longId];
                    Vector3d sideAxis = axes[sideId];
                    Vector3d verticalAxis = axes[verticalId];

                    lengthAxis.Unitize();
                    sideAxis.Unitize();
                    verticalAxis.Unitize();


                    // Flip object Z upward if needed
                    if (verticalAxis * Vector3d.ZAxis < 0)
                    {
                        verticalAxis = -verticalAxis;
                    }


                    // Rebuild frame orthogonally
                    sideAxis = Vector3d.CrossProduct(verticalAxis, lengthAxis);
                    sideAxis.Unitize();

                    lengthAxis = Vector3d.CrossProduct(sideAxis, verticalAxis);
                    lengthAxis.Unitize();


                    // Keep directions consistent
                    if (lengthAxis * axes[longId] < 0)
                        lengthAxis = -lengthAxis;

                    if (sideAxis * axes[sideId] < 0)
                        sideAxis = -sideAxis;


                    // Dimensions
                    double length = sizes[longId];
                    double sideWidth = sizes[sideId];
                    double height = sizes[verticalId];


                    // ------------------------------------------------------------
                    // Oblique directions
                    // ------------------------------------------------------------

                    double angle = RhinoMath.ToRadians(45);

                    Vector3d leftDir =
                          verticalAxis * Math.Sin(angle)
                        - sideAxis * Math.Cos(angle);

                    Vector3d rightDir =
                          verticalAxis * Math.Sin(angle)
                        + sideAxis * Math.Cos(angle);

                    leftDir.Unitize();
                    rightDir.Unitize();

                    // ------------------------------------------------------------
                    // Number of positions along object length
                    // ------------------------------------------------------------

                    double step =
                        CaptureW * (1.0 - Overlap);

                    int count =
                        Math.Max(
                            1,
                            (int)Math.Ceiling(length / step));


                    // ------------------------------------------------------------
                    // Top scan line
                    // ------------------------------------------------------------

                    Point3d upCenter =
                        center +
                        verticalAxis * (height * 0.5);



                    // ------------------------------------------------------------
                    // Generate scan targets and camera planes
                    // ------------------------------------------------------------

                    GH_Path leftPath = new GH_Path(0);
                    GH_Path rightPath = new GH_Path(1);

                    for (int i = 0; i < count; i++)
                    {
                        double t;

                        if (count == 1)
                            t = 0.5;
                        else
                            t = (double)i / (count - 1);

                        Point3d target =
                            upCenter +
                            lengthAxis * ((t - 0.5) * length);

                        Plane leftPlane =
                            CreateCameraPlane(
                                target,
                                leftDir,
                                Distance,
                                lengthAxis);

                        Plane rightPlane =
                            CreateCameraPlane(
                                target,
                                rightDir,
                                Distance,
                                lengthAxis);

                        planeTree.Add(leftPlane, leftPath);
                        planeTree.Add(rightPlane, rightPath);
                    }



                    break;
                case 2:
                    break;
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

        public static Box GetMinimumBoundingBox3D(Mesh inputMesh)
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
            List<Box> SortedBoundingBoxes = orientedBoxes.OrderBy(o => o.Volume).ToList();

            // Return the smallest one
            return SortedBoundingBoxes[0];
        }

        private static Plane CreateCameraPlane(Point3d target, Vector3d direction, double distance, Vector3d preferredXAxis)
        {
            direction.Unitize();

            Point3d camPos =
                target + direction * distance;

            // TCP/camera Z points away from object
            Vector3d zAxis = direction;
            zAxis.Unitize();

            // Try to keep plane X axis along the scan length
            Vector3d xAxis =
                preferredXAxis
                - zAxis * Vector3d.Multiply(preferredXAxis, zAxis);

            if (!xAxis.Unitize())
            {
                xAxis =
                    Vector3d.CrossProduct(
                        Vector3d.ZAxis,
                        zAxis);

                if (!xAxis.Unitize())
                    xAxis = Vector3d.XAxis;
            }

            // Ensure Plane.ZAxis = zAxis
            Vector3d yAxis =
                Vector3d.CrossProduct(
                    zAxis,
                    xAxis);

            yAxis.Unitize();

            xAxis =
                Vector3d.CrossProduct(
                    yAxis,
                    zAxis);

            xAxis.Unitize();

            return new Plane(
                camPos,
                -xAxis,
                -yAxis);
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