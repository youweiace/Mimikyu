using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Rhino;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using static Mimikyu.Helper.PixelHelper;
using static Mimikyu.Helper.ScannerHelper;
using Mesh = Rhino.Geometry.Mesh;


namespace Mimikyu._1_PC_Robot
{
    public class GH_ObjectPose : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the GH_ImagePose class.
        /// </summary>
        public GH_ObjectPose()
          : base("ObjectPose", "ObjPo",
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
            pManager.AddTextParameter("IntrinsicsPath", "I", "Json path to the camera intrinsics file.", GH_ParamAccess.item);
            pManager.AddIntegerParameter("Mode", "M", "Scanning mode\n 0: Face Scans\n 1: Oblique 2 Sides\n 2: Oblique 4 Corners (under development)", GH_ParamAccess.item);
            pManager.AddNumberParameter("Margin", "M", "Scan extending edge margin percentage", GH_ParamAccess.item, 0.15);
            pManager[1].Optional = true;
            pManager[2].Optional = true;
            pManager[3].Optional = true;
            pManager[4].Optional = true;
            pManager[5].Optional = true;

        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddPlaneParameter("Poses", "P", "Pose as Planes", GH_ParamAccess.tree);
            pManager.AddBoxParameter("Box", "B", "Box", GH_ParamAccess.item);
            pManager.AddGenericParameter("Scan Object","SO","Shared scan object definition for downstream defect projection and defect scanning",GH_ParamAccess.item);
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
            string intrinsicsPath = null;
            double Overlap = 0.10;
            int mode = 0;
            double marginBuffer = 0.15;

            DataTree<Plane> planeTree = new DataTree<Plane>();

            if (!DA.GetData(0, ref inGeo)) return;
            if (!DA.GetData(1, ref CaptureW)) return;
            if (!DA.GetData(2, ref CaptureH)) return;
            if (!DA.GetData(3, ref Distance)) return;
            DA.GetData(4, ref intrinsicsPath);
            if (!DA.GetData(5, ref mode)) return;
            if (!DA.GetData(6, ref marginBuffer)) return;

            ScanObject scanObject =
                ScanObject.FromBrep(inGeo);

            Mesh mesh =
                scanObject.Mesh;

            Box obb =
                scanObject.BoundingBox;

            Plane objectPlane =
                scanObject.ObjectPlane;

            double sx =
                scanObject.SizeX;

            double sy =
                scanObject.SizeY;

            double sz =
                scanObject.SizeZ;

            Point3d center =
                scanObject.Center;

            switch (mode)
            {
                case 0:

                    List<ScanFace> faces =
                        scanObject.GetObjectPoseFacesInOriginalOrder();


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

                                double marginU =
                                    Math.Min(
                                        0.49,
                                        (CaptureH * (0.5 - marginBuffer)) / face.Width);

                                double marginV =
                                    Math.Min(
                                        0.49,
                                        (CaptureW * (0.5 - marginBuffer)) / face.Height);

                                if (countU == 1)
                                    u = 0.5;
                                else
                                    u =
                                        marginU +
                                        ((double)col / (countU - 1)) *
                                        (1.0 - 2.0 * marginU);

                                if (countV == 1)
                                    v = 0.5;
                                else
                                    v =
                                        marginV +
                                        ((double)row / (countV - 1)) *
                                        (1.0 - 2.0 * marginV);


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

                    double angle = RhinoMath.ToRadians(ObliqueAngleDeg);

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

                    if (intrinsicsPath != null)
                    {
                        CameraIntrinsics K = LoadIntrinsics(intrinsicsPath);
                        
                        double imageWidth = K.image_width;
                        double imageHeight = K.image_height;

                        double fx = K.camera_matrix.fx;
                        double fy = K.camera_matrix.fy;

                        double distanceW =
                            CaptureW * fx / imageWidth;

                        double distanceH =
                            CaptureH * fy / imageHeight;

                        Distance = Math.Max(distanceW, distanceH);

                        CaptureW = Distance * imageWidth / fx;
                        CaptureH = Distance * imageHeight / fy;
                    }

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

                    double margin =
                        Math.Min(
                            0.49,
                            (CaptureW * (0.5 - marginBuffer)) / length);

                    for (int i = 0; i < count; i++)
                    {
                        double t;

                        if (count == 1)
                        {
                            t = 0.5;
                        }
                        else
                        {
                            t =
                                margin +
                                ((double)i / (count - 1)) *
                                (1.0 - 2.0 * margin);
                        }

                        Point3d target =
                            center +
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
            DA.SetData(2, scanObject);
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