using System;
using System.IO;
using System.Windows;

namespace ns_tem2rgb
{
    class tem2rgb
    {
        private byte[] data;

        public tem2rgb(string path)
        {
            try
            { this.data = File.ReadAllBytes(path); }
            catch
            { MessageBox.Show("色温-rgb对照表读取失败"); }
        }

        public byte[] getrgbbytem(int tem, int deg = 2)
        {
            byte[] res = new byte[] { 0, 0, 0 };

            if (tem < 1000)
            { tem = 1000; }
            if (tem > 40000)
            { tem = 40000; }

            int indx = Convert.ToInt32((tem - 1000) / 100);
            int ad = 0;
            if (deg == 2)
            { ad = 0; }
            if (deg == 10)
            { ad = 3; }

            for (int i = 0; i < 3; i++)
            {
                res[i] = this.data[indx * 6 + ad + i];
            }
            return res;
        }

        public byte[] getrgb(double tem, double[] temrange)
        {
            byte[] res = new byte[] { 0, 0, 0 };
            if (tem < temrange[0])
            { tem = temrange[0]; }
            if (tem > temrange[1])
            { tem = temrange[1]; }

            double scale = (temrange[1] - temrange[0]) / 780;
            int indx = Convert.ToInt32((tem - temrange[0]) / scale);
            for (int i = 0; i < 3; i++)
            {
                res[i] = this.data[indx * 3 + i];
            }
            return res;
        }
    }
}