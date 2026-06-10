using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Mimikyu.Helper
{
    internal class MimikyuObject
    {
        public List<Point3d> MimikPoint { get; set; }
        public System.Drawing.Color MimikColor { get; set; }
        PointCloud MimikCloud { get; set; }
    }
}
