using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;
using Rhino;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using static Mimikyu.Helper.ScannerHelper;

namespace Mimikyu
{
    public class GH_DefectPose : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the GH_DefectPose class.
        /// </summary>
        public GH_DefectPose()
          : base("DefectPose", "DP",
              "Generate detailed scan poses from 3D defect point branches using local 3D OBB",
              "Mimikyu", "PC_Robot")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {

            pManager.AddMeshParameter("Object Mesh", "M", "Object mesh used to snap scan targets and get local surface normals", GH_ParamAccess.item);
            pManager.AddPointParameter("Defect Points", "Dpts", "Tree of 3D defect points. Initial branches are raw defect IDs or surface-grouped defect patches.", GH_ParamAccess.tree);
            pManager.AddNumberParameter("Capture Width", "W", "Camera/scanner capture width in mm", GH_ParamAccess.item, 428.0);
            pManager.AddNumberParameter("Capture Height", "H", "Camera/scanner capture height in mm", GH_ParamAccess.item, 330.0);
            pManager.AddNumberParameter("Distance", "D", "Camera distance from scan target in mm", GH_ParamAccess.item, 500.0);
            pManager.AddNumberParameter("Overlap", "O", "Overlap ratio between neighbouring captures, e.g. 0.10 = 10%", GH_ParamAccess.item, 0.10);
            pManager.AddNumberParameter("Merge Distance", "Md", "Distance in mm used to merge nearby defect branches into one scan region", GH_ParamAccess.item, 80.0);
            pManager.AddIntegerParameter("Max Hull Points", "Mh", "Maximum number of points used to compute the convex hull per merged region. Keeps hull computation reasonable.", GH_ParamAccess.item, 120);
            pManager.AddNumberParameter("3D Threshold", "T3D", "If the smallest OBB dimension is larger than this value, the region is marked as 3D/non-planar.", GH_ParamAccess.item, 20.0);
            pManager.AddBoxParameter(
    "Object Box",
    "B",
    "Object oriented bounding box used to define stable scan-object sides",
    GH_ParamAccess.item);

            pManager[9].Optional = false;

            pManager[2].Optional = true;
            pManager[3].Optional = true;
            pManager[4].Optional = true;
            pManager[5].Optional = true;
            pManager[6].Optional = true;
            pManager[7].Optional = true;
            pManager[8].Optional = true;

        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {

            pManager.AddPlaneParameter("Scan Poses", "P", "Generated camera scan poses. One branch per merged scan region.", GH_ParamAccess.tree);
            pManager.AddBoxParameter("Region OBBs", "B", "Minimum 3D OBB for each merged defect scan region.", GH_ParamAccess.tree);
            pManager.AddPointParameter("Region Points", "Rpts", "Merged defect points. One branch per final scan region.", GH_ParamAccess.tree);
            pManager.AddMeshParameter("Hull Meshes", "H", "Convex hull mesh used for each merged scan region.", GH_ParamAccess.tree);
            pManager.AddTextParameter("Info", "I", "Debug information per merged scan region.", GH_ParamAccess.tree);

        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {


            Mesh objectMesh = null;
            GH_Structure<GH_Point> defectTree = null;

            double captureW = 428.0;
            double captureH = 330.0;
            double distance = 500.0;
            double overlap = 0.10;
            double mergeDistance = 80.0;
            int maxHullPoints = 120;
            double threeDThreshold = 5.0;
            Box objectBox = Box.Unset;

            if (!DA.GetData(0, ref objectMesh)) return;
            if (!DA.GetDataTree(1, out defectTree)) return;

            DA.GetData(2, ref captureW);
            DA.GetData(3, ref captureH);
            DA.GetData(4, ref distance);
            DA.GetData(5, ref overlap);
            DA.GetData(6, ref mergeDistance);
            DA.GetData(7, ref maxHullPoints);
            DA.GetData(8, ref threeDThreshold);
            if (!DA.GetData(9, ref objectBox))return;

            DataTree<Plane> poseTree = new DataTree<Plane>();
            DataTree<Box> boxTree = new DataTree<Box>();
            DataTree<Point3d> regionPointTree = new DataTree<Point3d>();
            DataTree<Mesh> hullTree = new DataTree<Mesh>();
            DataTree<string> infoTree = new DataTree<string>();

            if (objectMesh == null || !objectMesh.IsValid)
            {
                AddRuntimeMessage(
                    GH_RuntimeMessageLevel.Error,
                    "Object mesh is null or invalid.");

                return;
            }

            if (defectTree == null || defectTree.PathCount == 0)
            {
                AddRuntimeMessage(
                    GH_RuntimeMessageLevel.Warning,
                    "No defect point branches found.");

                return;
            }

            objectMesh.FaceNormals.ComputeFaceNormals();
            objectMesh.Normals.ComputeNormals();
            ScanObject scanObject =
    ScanObject.FromMeshAndBox(
        objectMesh,
        objectBox);
            captureW = Math.Abs(captureW);
            captureH = Math.Abs(captureH);
            distance = Math.Abs(distance);
            mergeDistance = Math.Abs(mergeDistance);

            maxHullPoints = Math.Max(20, maxHullPoints);

            overlap = Math.Max(0.0, Math.Min(0.9, overlap));

            double stepU = captureW * (1.0 - overlap);
            double stepV = captureH * (1.0 - overlap);

            if (stepU <= RhinoMath.ZeroTolerance)
                stepU = captureW;

            if (stepV <= RhinoMath.ZeroTolerance)
                stepV = captureH;

            // ------------------------------------------------------------
            // 1. Read raw defect branches
            // ------------------------------------------------------------
            List<RawDefectBranch> rawBranches =
                ReadRawDefectBranches(defectTree);

            if (rawBranches.Count == 0)
            {
                AddRuntimeMessage(
                    GH_RuntimeMessageLevel.Warning,
                    "No valid defect points found.");

                return;
            }

            // ------------------------------------------------------------
            // 2. Merge nearby raw branches into final scan regions
            // ------------------------------------------------------------
            List<MergedRegion> regions =
                MergeNearbyBranches(
                    rawBranches,
                    mergeDistance);

            if (regions.Count == 0)
            {
                AddRuntimeMessage(
                    GH_RuntimeMessageLevel.Warning,
                    "No merged defect regions created.");

                return;
            }

            // ------------------------------------------------------------
            // 3. For each merged region, create hull, compute OBB, tile poses
            // ------------------------------------------------------------
            for (int r = 0; r < regions.Count; r++)
            {
                MergedRegion region = regions[r];
                GH_Path outPath = new GH_Path(r);

                List<Point3d> regionPts = region.Points;

                if (regionPts == null || regionPts.Count < 4)
                {
                    infoTree.Add(
                        "Skipped region " + r +
                        ": fewer than 4 points after merging.",
                        outPath);

                    continue;
                }

                foreach (Point3d p in regionPts)
                    regionPointTree.Add(p, outPath);

                List<Point3d> hullInputPts =
                    RemoveDuplicatePoints(
                        regionPts,
                        RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);


                // ------------------------------------------------------------
                // Decide if this merged region is planar or truly 3D
                // ------------------------------------------------------------
                bool isPlanar;
                double planarThickness;

                isPlanar =
                    IsPointSetPlanar(
                        hullInputPts,
                        threeDThreshold,
                        out planarThickness);

                Box regionBox = Box.Unset;
                Mesh hullMesh = null;

                if (isPlanar)
                {
                    // --------------------------------------------------------
                    // Planar defect:
                    // Use best-fit plane + planar local bounding box.
                    // This is better for cracks, knots, stains, surface damage.
                    // --------------------------------------------------------
                    bool okPlanarBox =
                        TryCreatePlanarDefectBox(
                            hullInputPts,
                            out regionBox);

                    if (!okPlanarBox || !regionBox.IsValid)
                    {
                        infoTree.Add(
                            "Skipped region " + r +
                            ": failed to create planar defect box.",
                            outPath);

                        continue;
                    }

                    infoTree.Add(
                        "Region " + r +
                        " uses PLANAR workflow. Thickness = " +
                        planarThickness.ToString("F2"),
                        outPath);
                }
                else
                {
                    // --------------------------------------------------------
                    // True 3D defect:
                    // Use MIConvexHull + your original minimum 3D OBB.
                    // This is better for broken edges, corner damage, chunks.
                    // --------------------------------------------------------
                    string hullDebug;

                    bool okHull =
                        CreateHullMeshMIConvexHull(
                            hullInputPts,
                            out hullMesh,
                            out hullDebug);

                    infoTree.Add(
                        "Region " + r + " hull debug: " + hullDebug,
                        outPath);

                    if (!okHull || hullMesh == null || !hullMesh.IsValid || hullMesh.Faces.Count == 0)
                    {
                        infoTree.Add(
                            "Skipped region " + r +
                            ": failed to create a valid 3D convex hull.",
                            outPath);

                        continue;
                    }

                    hullMesh.Normals.ComputeNormals();
                    hullMesh.FaceNormals.ComputeFaceNormals();
                    hullMesh.Compact();

                    hullTree.Add(hullMesh, outPath);

                    regionBox =
                        GetMinimumBoundingBox3D(hullMesh);

                    if (!regionBox.IsValid)
                    {
                        infoTree.Add(
                            "Skipped region " + r +
                            ": invalid minimum 3D OBB.",
                            outPath);

                        continue;
                    }

                    infoTree.Add(
                        "Region " + r +
                        " uses 3D HULL workflow. Thickness = " +
                        planarThickness.ToString("F2") +
                        ", hull faces = " + hullMesh.Faces.Count,
                        outPath);
                }

                boxTree.Add(regionBox, outPath);

                Vector3d[] axes;
                double[] sizes;

                GetBoxAxesAndSizes(
                    regionBox,
                    out axes,
                    out sizes);

                int uId;
                int vId;
                int nId;

                GetScanAxisIds(
                    sizes,
                    out uId,
                    out vId,
                    out nId);

                Vector3d uAxis = axes[uId];
                Vector3d vAxis = axes[vId];
                Vector3d boxNormal = axes[nId];


                uAxis.Unitize();
                vAxis.Unitize();
                boxNormal.Unitize();

                List<Vector3d> scanDirections =
                    GetScanDirections(
                        region,
                        scanObject);

                RegionType regionType = GetRegionType(region);

                infoTree.Add(
                            "Region " + r +
                            " scan type: " + regionType.ToString() +
                            " | SideIds: " + string.Join(",", region.SideIds) +
                            " | Scan directions: " + scanDirections.Count,
                            outPath);

                double sizeU = sizes[uId];
                double sizeV = sizes[vId];
                double sizeN = sizes[nId];


                int countU =
                    Math.Max(
                        1,
                        (int)Math.Ceiling(sizeU / stepU));

                int countV =
                    Math.Max(
                        1,
                        (int)Math.Ceiling(sizeV / stepV));

                string typeText =
                    sizeN > threeDThreshold
                    ? "3D / non-planar candidate"
                    : "near-surface candidate";

                infoTree.Add(
                    "Region " + r +
                    " | Raw branches merged: " + string.Join(",", region.RawBranchIndices) +
                    " | Points: " + regionPts.Count +
                    " | Hull points used: " + hullInputPts.Count +
                    " | Size U: " + sizeU.ToString("F2") +
                    " | Size V: " + sizeV.ToString("F2") +
                    " | Thickness: " + sizeN.ToString("F2") +
                    " | countU: " + countU +
                    " | countV: " + countV +
                    " | Type: " + typeText,
                    outPath);

                // --------------------------------------------------------
                // 4. Generate tiled scan targets on the OBB middle plane
                // --------------------------------------------------------
                for (int row = 0; row < countV; row++)
                {
                    bool reverse = row % 2 == 1;

                    for (int colIter = 0; colIter < countU; colIter++)
                    {
                        int col =
                            reverse
                            ? countU - 1 - colIter
                            : colIter;

                        double tu =
                            countU == 1
                            ? 0.5
                            : (double)col / (double)(countU - 1);

                        double tv =
                            countV == 1
                            ? 0.5
                            : (double)row / (double)(countV - 1);

                        double[] t = new double[3];

                        t[uId] = tu;
                        t[vId] = tv;
                        t[nId] = 0.5;

                        Point3d roughTarget =
                            regionBox.PointAt(
                                t[0],
                                t[1],
                                t[2]);

                        Point3d surfaceTarget;
                        Vector3d localNormal;

                        bool okSurface =
                            ProjectPointToMeshSurface(
                                objectMesh,
                                roughTarget,
                                boxNormal,
                                out surfaceTarget,
                                out localNormal);

                        if (!okSurface)
                        {
                            infoTree.Add(
                                "Skipped one target in region " + r +
                                ": could not snap target to object mesh.",
                                outPath);

                            continue;
                        }

                        if (!localNormal.Unitize())
                            localNormal = boxNormal;

                        //RegionType regionType =    GetRegionType(region);

                        List<Vector3d> finalViewDirections =
                            new List<Vector3d>();

                        if (regionType == RegionType.Surface)
                        {
                            foreach (Vector3d dirRaw in scanDirections)
                            {
                                Vector3d dir =
                                    dirRaw;

                                if (!dir.Unitize())
                                    continue;

                                List<Vector3d> crackViews =
                                    CreateCrackViews(
                                        dir,
                                        uAxis,
                                        30.0);

                                finalViewDirections.AddRange(
                                    crackViews);
                            }
                        }
                        else
                        {
                            foreach (Vector3d dirRaw in scanDirections)
                            {
                                Vector3d dir =
                                    dirRaw;

                                if (!dir.Unitize())
                                    continue;

                                finalViewDirections.Add(
                                    dir);
                            }
                        }

                        foreach (Vector3d viewDirRaw in finalViewDirections)
                        {
                            Vector3d viewDir =
                                viewDirRaw;

                            if (!viewDir.Unitize())
                                continue;

                            Plane pose =
                                CreateCameraPlane(
                                    surfaceTarget,
                                    viewDir,
                                    distance,
                                    uAxis);

                            poseTree.Add(
                                pose,
                                outPath);
                        }
                    }
                }
            }

            DA.SetDataTree(0, poseTree);
            DA.SetDataTree(1, boxTree);
            DA.SetDataTree(2, regionPointTree);
            DA.SetDataTree(3, hullTree);
            DA.SetDataTree(4, infoTree);


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
            get { return new Guid("3E83515E-313C-453B-878E-92F39A8BA785"); }
        }
    }
}