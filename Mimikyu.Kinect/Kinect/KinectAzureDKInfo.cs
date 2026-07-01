using Grasshopper;
using Grasshopper.Kernel;
using System;
using System.Drawing;

namespace KinectAzureDK
{
    public class KinectAzureDKInfo : GH_AssemblyInfo
    {
        public override string Name => "KinectAzureDK";

        //Return a 24x24 pixel bitmap to represent this GHA library.
        public override Bitmap Icon => null;

        //Return a short string describing the purpose of this GHA library.
        public override string Description => "";

        public override Guid Id => new Guid("2adbf388-9266-4bc4-ba73-662431d29fe2");

        //Return a string identifying you or your company.
        public override string AuthorName => "";

        //Return a string representing your preferred contact details.
        public override string AuthorContact => "";
    }
}