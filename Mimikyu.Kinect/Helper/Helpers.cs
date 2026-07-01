using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Imaging;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Azure.Kinect.Sensor;
namespace Mimikyu.Kinect.Helper
{
    class AzureKienctHelpers
    {
        static public void ColorImageToBitmap(Microsoft.Azure.Kinect.Sensor.Image img, Bitmap outBitmap)
        {
            Memory<Byte4> colorMemory = img.GetPixels<Byte4>();
            Span<Byte4> cs = colorMemory.Span;

            BitmapData bitmapData = outBitmap.LockBits(
                new Rectangle(0, 0, img.WidthPixels, img.HeightPixels),
                ImageLockMode.WriteOnly,
                outBitmap.PixelFormat);

            IntPtr ptr = bitmapData.Scan0;
            Marshal.Copy(MemoryMarshal.AsBytes(cs).ToArray(), 0, ptr, img.WidthPixels * img.HeightPixels * 4);
            outBitmap.UnlockBits(bitmapData);
        }

        static public void DepthImageToBitmap(Microsoft.Azure.Kinect.Sensor.Image img, Bitmap outBitmap, double min, double max)
        {
            Memory<Byte2> memory = img.GetPixels<Byte2>();
            Span<Byte2> s = memory.Span;
            
            byte[] colorData = new byte[img.WidthPixels * img.HeightPixels * 4];
            for(int i = 0; i < s.Length; i++)
            {
                int depth = s[i].Int;

                double cv;
                if (depth < min)
                    cv = 0;
                else if (depth > max)
                    cv = 1;
                else
                    cv = (depth - min) / (max - min);

                byte normalizedColor = (byte)(cv * 255);

                colorData[i * 4 + 0] = normalizedColor;
                colorData[i * 4 + 1] = normalizedColor;
                colorData[i * 4 + 2] = normalizedColor;
                colorData[i * 4 + 3] = 255;
            }

            BitmapData bitmapData = outBitmap.LockBits(
                new Rectangle(0, 0, img.WidthPixels, img.HeightPixels),
                ImageLockMode.WriteOnly,
                outBitmap.PixelFormat);
            IntPtr ptr = bitmapData.Scan0;
            Marshal.Copy(colorData, 0, ptr, img.WidthPixels * img.HeightPixels * 4);
            outBitmap.UnlockBits(bitmapData);


        }


    }

    struct Byte4
    {
        public byte B;
        public byte G;
        public byte R;
        public byte A;
    }

    struct Byte2
    {
        public byte Hi;
        public byte Lo;

       // public int Int => BitConverter.ToInt16(new byte[] { Hi, Lo }, 0)
             public int Int => (Lo << 8) + Hi;
        //
    }
}
