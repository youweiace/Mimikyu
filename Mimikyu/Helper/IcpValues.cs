using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Mimikyu.Helper
{
    public class IcpValues
    {
        public double VoxelSize { get; set; }
        public double IcpMaxDistance { get; set; }
        public int IcpMaxIterations { get; set; }
    }
    public class ICPResult
    {
        public Transform Transform;
        public double Fitness;
        public double RMSE;
        public int CorrespondenceCount;

        public ICPResult()
        {
            Transform = Transform.Identity;
            Fitness = 0.0;
            RMSE = double.NaN;
            CorrespondenceCount = 0;
        }
    }
}
