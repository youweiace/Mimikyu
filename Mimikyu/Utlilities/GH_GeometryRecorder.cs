using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Rhino.Geometry;
using System;
using System.Collections.Generic;

namespace Mimikyu.Utlilities
{
    public class GH_GeometryRecorder : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the GH_DataRecorder class.
        /// </summary>
        public GH_GeometryRecorder()
          : base("GeometryRecorder", "R",
              "A data recorder that saves geometry",
              "Mimikyu", "Utilities")
        {
        }


        private readonly List<object> items = new List<object>();

        // Tracks the last event that was already recorded
        private int lastRecordedEventId = -1;


        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Item", "I", "The item to record.", GH_ParamAccess.item);
            pManager.AddIntegerParameter("EventId", "E", "A unique ID that changes whenever a new event/data arrival occurs.", GH_ParamAccess.item, -1);
            pManager.AddBooleanParameter("Reset", "R", "Clears the storage.", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("Items", "Is", "List of items", GH_ParamAccess.list);
            pManager.AddIntegerParameter("LastEventId", "L", "Last recorded EventId", GH_ParamAccess.item);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {

            object item = null;
            int eventId = -1;
            bool reset = false;

            if (!DA.GetData(1, ref eventId)) return;
            if (!DA.GetData(2, ref reset)) return;
            DA.GetData(0, ref item);

            if (reset)
            {
                items.Clear();

                // Recommended:
                // keep lastRecordedEventId as-is, so the current event is NOT immediately re-recorded
                // after reset unless a truly new EventId arrives.

                // If you WANT reset to also allow the current event to be recorded again,
                // uncomment the next line:
                // lastRecordedEventId = -1;
            }
            else if (item != null && eventId != lastRecordedEventId)
            {
                object storedItem = DuplicateItem(item);
                items.Add(storedItem);
                lastRecordedEventId = eventId;
            }

            DA.SetDataList(0, items);
            DA.SetData(1, lastRecordedEventId);

            Message = $"Count: {items.Count}";
        }

        /// <summary>
        /// Creates a safe copy of the incoming item whenever possible.
        /// This helps prevent recorded items from changing later if the original object is modified.
        /// </summary>
        private object DuplicateItem(object item)
        {
            if (item == null)
                return null;

            // If Grasshopper gives us a wrapper, unwrap it first
            if (item is GH_ObjectWrapper wrapper)
            {
                if (wrapper.Value == null)
                    return null;

                return DuplicateItem(wrapper.Value);
            }

            // If it's a GH goo type, try to duplicate/unwrap safely
            if (item is IGH_Goo goo)
            {
                // Try to convert to a script variable / raw Rhino type first
                try
                {
                    object scriptVar = goo.ScriptVariable();
                    if (scriptVar != null && !ReferenceEquals(scriptVar, goo))
                        return DuplicateItem(scriptVar);
                }
                catch
                {
                    // ignore and continue
                }

                try
                {
                    IGH_Goo dupGoo = goo.Duplicate();
                    if (dupGoo != null)
                        return dupGoo;
                }
                catch
                {
                    // ignore and continue
                }
            }

            // Rhino reference geometry -> duplicate
            if (item is GeometryBase geometry)
                return geometry.Duplicate();

            // Common Rhino value types -> safe to return directly
            if (item is Point3d) return item;
            if (item is Vector3d) return item;
            if (item is Plane) return item;
            if (item is Line) return item;
            if (item is Interval) return item;
            if (item is Transform) return item;
            if (item is Rectangle3d) return item;
            if (item is BoundingBox) return item;
            if (item is System.Drawing.Color) return item;

            // Generic cloneable objects
            if (item is ICloneable cloneable)
                return cloneable.Clone();

            // Fallback:
            // if this is a mutable custom class and not cloneable,
            // this will store the same reference.
            return item;
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
            get { return new Guid("53362123-aff5-4bf2-ab7c-d5927f0d61c2"); }
        }
    }
}