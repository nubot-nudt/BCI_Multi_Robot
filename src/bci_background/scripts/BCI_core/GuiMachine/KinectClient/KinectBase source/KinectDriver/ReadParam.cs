using System.IO;
using System;
using System.Windows;

namespace ns_readparam
{
    public class ReadParam
    {
        public float[] Row1, Row2, Row3;
        private float[] tRow1, tRow2, tRow3;
        public float[] coe;
        private float[] coe1;

        public ReadParam()
        {
            this.Row1 = new float[3] { 1, 0, 0 };
            this.Row2 = new float[3] { 0, 1, 0 };
            this.Row3 = new float[3] { 0, 0, 1 };
            this.tRow1 = new float[3] { 1, 0, 0 };
            this.tRow2 = new float[3] { 0, 1, 0 };
            this.tRow3 = new float[3] { 0, 0, 1 };
            this.coe = new float[5] { 0, 0, 1, 0, 1};
            this.coe1 = new float[5] { 0, 0, 1, 0, 1};
        }

        public bool GetTransitMatrix(string path)
        {
            try
            {
                byte[] buf = File.ReadAllBytes(path);
                int indx = 0;
                for (int i = 0; i < 3; i++)
                {
                    this.tRow1[i] = BitConverter.ToSingle(buf, indx);
                    indx += 4;
                }
                for (int i = 0; i < 3; i++)
                {
                    this.tRow2[i] = BitConverter.ToSingle(buf, indx);
                    indx += 4;
                }
                for (int i = 0; i < 3; i++)
                {
                    this.tRow3[i] = BitConverter.ToSingle(buf, indx);
                    indx += 4;
                }

                this.Row1 = this.tRow1;
                this.Row2 = this.tRow2;
                this.Row3 = this.tRow3;

                MessageBox.Show("正在使用"+path+"定义的世界坐标系 单位mm");
                return true;
           }
            catch
            {
                MessageBox.Show("正在使用摄像机坐标系 单位mm");
                return false; 
            }
        }

        public bool GetGroundParam(string path)
        {
            try
            {
                byte[] buf = File.ReadAllBytes(path);
                int indx = 0;
                for (int i = 0; i < 5; i++)
                {
                    this.coe1[i] = BitConverter.ToSingle(buf, indx);
                    indx += 4;
                }

                this.coe = this.coe1;
                MessageBox.Show("正在使用" + path + "定义的地面滤波参数");
                return true;
            }
            catch
            {
                MessageBox.Show("不使用地面滤波");
                return false;
            }
        }
    }
}
