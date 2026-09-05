using Grasshopper.Kernel;
using Mimikyu.Helper;
using Rhino;
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
            // ------------------------------------------------------------
            // Rule 1:
            //
            // The input list is treated as one scan group/branch.
            //
            // All planes in the branch receive the same left/right
            // classification based on the average branch position.
            //
            // CameraBase is used as the fixed global reference.
            // If CameraBase is not connected, World Origin is used.
            //
            // Right branch -> Plane Y faces World -Y
            // Left branch  -> Plane Y faces World +Y
            //
            // Planes already facing correctly remain unchanged.
            // Rotation around local Z preserves Plane.ZAxis.
            // ------------------------------------------------------------
            if (serpentinePlanes.Count > 0)
            {
                // Representative point of this entire input branch.
                Point3d branchCenter =
                    new Point3d(
                        serpentinePlanes.Average(p => p.OriginX),
                        serpentinePlanes.Average(p => p.OriginY),
                        serpentinePlanes.Average(p => p.OriginZ));

                // A fixed reference is essential when processing only one branch.
                Point3d referencePoint =
                    cameraBase.IsValid
                    ? cameraBase
                    : Point3d.Origin;

                // Direction from the fixed reference toward this branch.
                Vector3d referenceToBranch =
                    branchCenter - referencePoint;

                referenceToBranch.Z = 0.0;

                if (!referenceToBranch.Unitize())
                {
                    referenceToBranch = Vector3d.XAxis;
                }

                /*
                 * Define right relative to the viewing direction.
                 *
                 * Keep this cross-product order because this is the order
                 * that worked correctly in your previous test.
                 */
                Vector3d rightDirection =
                    Vector3d.CrossProduct(
                        referenceToBranch,
                        Vector3d.ZAxis);

                if (!rightDirection.Unitize())
                {
                    rightDirection = -Vector3d.YAxis;
                }

                /*
                 * Determine the side ONCE for the entire branch.
                 *
                 * Here CameraBase/World Origin is the global dividing reference.
                 * Do not use p.Origin - branchCenter for each plane because
                 * that separates the +10 and -10 degree camera positions.
                 */
                Vector3d referenceToBranchCenter =
                    branchCenter - referencePoint;

                referenceToBranchCenter.Z = 0.0;

                double sideValue =
                    referenceToBranchCenter * rightDirection;

                /*
                 * Because rightDirection is perpendicular to referenceToBranch,
                 * the expression above can be close to zero by construction.
                 *
                 * For your current World-X-facing setup, use branch Y relative
                 * to the fixed reference to distinguish the physical sides.
                 */
                double branchSideValue =
                    branchCenter.Y - referencePoint.Y;

                double sideTolerance = 1.0;

                bool branchIsRightSide;

                if (Math.Abs(branchSideValue) <= sideTolerance)
                {
                    // Branch lies approximately on the reference Y.
                    // Leave all planes unchanged because the side is ambiguous.
                    AddRuntimeMessage(
                        GH_RuntimeMessageLevel.Remark,
                        "Rule 1 skipped: branch center is on the reference Y. " +
                        "Branch center Y = " +
                        branchCenter.Y.ToString("F2") +
                        ", reference Y = " +
                        referencePoint.Y.ToString("F2"));

                    branchIsRightSide = false;
                }
                else
                {
                    /*
                     * With the successful reversed side convention:
                     *
                     * Positive relative World Y is treated as left.
                     * Negative relative World Y is treated as right.
                     */
                    branchIsRightSide =
                        branchSideValue < 0.0;

                    Vector3d requiredYDirection =
                        branchIsRightSide
                        ? -Vector3d.YAxis
                        : Vector3d.YAxis;

                    int checkedCount = 0;
                    int correctCount = 0;
                    int flippedCount = 0;
                    int skippedCount = 0;

                    // Accept axes within 20 degrees of World X.
                    double worldXAlignmentTolerance =
                        Math.Cos(
                            RhinoMath.ToRadians(20.0));

                    for (int i = 0; i < serpentinePlanes.Count; i++)
                    {
                        Plane p =
                            serpentinePlanes[i];

                        if (!p.IsValid)
                            continue;

                        Vector3d planeX =
                            p.XAxis;

                        Vector3d planeY =
                            p.YAxis;

                        if (!planeX.Unitize() ||
                            !planeY.Unitize())
                        {
                            continue;
                        }

                        /*
                         * Apply only to planes whose local X axis is
                         * approximately parallel to World X.
                         */
                        double worldXAlignment =
                            Math.Abs(
                                planeX * Vector3d.XAxis);

                        if (worldXAlignment < worldXAlignmentTolerance)
                        {
                            skippedCount++;
                            continue;
                        }

                        checkedCount++;

                        /*
                         * Positive:
                         * Plane Y already points toward requiredYDirection.
                         *
                         * Negative:
                         * Plane Y points in the opposite direction.
                         */
                        double directionDot =
                            planeY * requiredYDirection;

                        if (directionDot >= 0.0)
                        {
                            // Already correct, do not rotate.
                            correctCount++;
                            continue;
                        }

                        /*
                         * Rotate around local Z:
                         *
                         * Z remains unchanged.
                         * X reverses.
                         * Y reverses.
                         */
                        bool rotated =
                            p.Rotate(
                                Math.PI,
                                p.ZAxis);

                        if (rotated)
                        {
                            serpentinePlanes[i] = p;
                            flippedCount++;
                        }
                    }

                    AddRuntimeMessage(
                        GH_RuntimeMessageLevel.Remark,
                        "Rule 1 | Branch center: " +
                        branchCenter.ToString() +
                        " | Reference: " +
                        referencePoint.ToString() +
                        " | Branch side: " +
                        (branchIsRightSide ? "RIGHT" : "LEFT") +
                        " | Required Y: " +
                        requiredYDirection.ToString() +
                        " | Checked: " +
                        checkedCount +
                        " | Already correct: " +
                        correctCount +
                        " | Flipped: " +
                        flippedCount +
                        " | Skipped: " +
                        skippedCount);
                }
            }            // ------------------------------------------------------------
            // Rule 2:
            //
            // For consecutive planes whose X axes scan along World X,
            // prevent an exact 180-degree Y-axis difference.
            //
            // The current plane remains unchanged unless the difference
            // is 180 degrees.
            // ------------------------------------------------------------
            for (int i = 1; i < serpentinePlanes.Count; i++)
            {
                Plane previous =
                    serpentinePlanes[i - 1];

                Plane current =
                    serpentinePlanes[i];

                if (!previous.IsValid ||
                    !current.IsValid)
                {
                    continue;
                }

                Vector3d previousX =
                    previous.XAxis;

                Vector3d currentX =
                    current.XAxis;

                if (!previousX.Unitize() ||
                    !currentX.Unitize())
                {
                    continue;
                }

                double previousWorldXAlignment =
                    Math.Abs(
                        previousX * Vector3d.XAxis);

                double currentWorldXAlignment =
                    Math.Abs(
                        currentX * Vector3d.XAxis);

                // Only process consecutive planes that scan along World X.
                if (previousWorldXAlignment < 0.90 ||
                    currentWorldXAlignment < 0.90)
                {
                    continue;
                }

                double previousAngle =
                    AngleAboutWorldX(previous);

                double currentAngle =
                    AngleAboutWorldX(current);

                double difference =
                    NormalizeDeg(
                        currentAngle - previousAngle);

                if (Math.Abs(difference) >= 180.0 - 1e-6)
                {
                    double targetDifference =
                        difference >= 0.0
                        ? 179.0
                        : -179.0;

                    double correction =
                        targetDifference - difference;

                    current.Rotate(
                        RhinoMath.ToRadians(correction),
                        Vector3d.ZAxis,
                        current.Origin);

                    serpentinePlanes[i] =
                        current;
                }
            }

            DA.SetDataList(0, serpentinePlanes);
        
        }
        private double NormalizeDeg(double angle)
        {
            while (angle > 180.0)
                angle -= 360.0;

            while (angle < -180.0)
                angle += 360.0;

            return angle;
        }
        private double AngleAboutWorldX(Plane p)
        {
            Vector3d y =
                new Vector3d(
                    0,
                    p.YAxis.Y,
                    p.YAxis.Z);

            if (!y.Unitize())
                return 0.0;

            return Rhino.RhinoMath.ToDegrees(
                Math.Atan2(
                    y.Z,
                    y.Y));
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