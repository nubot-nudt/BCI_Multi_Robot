namespace kinect_server
{
    ///author: mrtang
    ///date: 2017.5
    ///updated:2017.7
    ///version:1.0
    ///email:mrtang@nudt.edu.cn

    using System;
    using System.Windows;
    using System.IO;
    using System.Text;
    using System.Collections.Generic;
    using Microsoft.Kinect;
    using Microsoft.Kinect.Interop;
    using ShareMemLib;
    using System.Net;
    using System.Net.Sockets;
    using System.Runtime.InteropServices;
    using System.Threading;
    using ns_readparam;
    using ns_tem2rgb;

    using System.Diagnostics;
    
    //本程序依赖于外部参数 installtion_param.txt 和 groundfilter_param.txt
    //这些参数确定了摄像机安装关系和地面数据
    //本程序通过共享内存向其他进程传递彩色和深度数据，以及世界坐标系下的点云数据

    public partial class MainWindow : Window
    {
        private KinectSensor sensor;
        private byte[] colorPixels,rwcolorPixels;
        private byte[] depthPixels,pointcloudbytes;
        private DepthImagePixel[] rawdepthPixels;
        private CoordinateMapper camera_calib;
        private ColorImagePoint[] cali_points;
        private ShareMem SHrgb,SHdepth,SHpointcloud;                //for c# and Xa c++
        private ShareMem SHrgb_py, SHdepth_py, SHpointcloud_py;     //for c# and python
        private int w_rgb_index, w_depth_index;
        private ReadParam rp;
        private bool fg_flg;
        private tem2rgb tr;
        private float groundfilter_threshold = 100;
        private bool update_color;
        private ColorImageFrameReadyEventArgs color_e;

        public MainWindow()
        {
            InitializeComponent();
        }

        /// <summary>
        /// 初始化
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void mainw_Loaded(object sender, RoutedEventArgs e)
        {
            this.update_color = false;
            this.w_rgb_index = 0;
            this.w_depth_index = 0;
            ///初始化共享内存
            this.SHrgb = new ShareMem();
            this.SHdepth = new ShareMem();
            this.SHpointcloud = new ShareMem();
            if (0 != this.SHrgb.Init("_sharemem_for_colorpixels_", 921604))
            { this.exit("shared memory initial error."); }
            if (0 != this.SHdepth.Init("_sharemem_for_depthpixels_", 921604))
            { this.exit("shared memory initial error."); }
            if (0 != this.SHpointcloud.Init("_sharemem_for_point_cloud_", 3686404))
            { this.exit("shared memory initial error."); }

            this.SHrgb_py = new ShareMem();
            this.SHdepth_py = new ShareMem();
            this.SHpointcloud_py = new ShareMem();
            if (0 != this.SHrgb_py.Init("_sharemem_for_colorpixels_cspy", 921604))
            { this.exit("shared memory initial error."); }
            if (0 != this.SHdepth_py.Init("_sharemem_for_depthpixels_cspy", 921604))
            { this.exit("shared memory initial error."); }
            if (0 != this.SHpointcloud_py.Init("_sharemem_for_point_cloud_cspy", 3686404))
            { this.exit("shared memory initial error."); }

            string exename = System.Diagnostics.Process.GetCurrentProcess().MainModule.FileName;
            string directoryName = Path.GetDirectoryName(exename);
            this.Visibility = System.Windows.Visibility.Hidden;
            this.tr = new tem2rgb(directoryName + "//tem_rgb.txt");
            this.rp = new ReadParam();
            this.rp.GetTransitMatrix(directoryName + "//installtion_param.txt");
            this.fg_flg = this.rp.GetGroundParam(directoryName + "//groundfilter_param.txt");

            foreach (var potentialSensor in KinectSensor.KinectSensors)
            {
                if (potentialSensor.Status == KinectStatus.Connected)
                {
                    this.sensor = potentialSensor;
                    break;
                }
            }

            if (null != this.sensor)
            {
                this.sensor.ColorStream.Enable(ColorImageFormat.RgbResolution640x480Fps30);
                this.sensor.DepthStream.Enable(DepthImageFormat.Resolution640x480Fps30);
                this.colorPixels = new byte[921604];
                this.rwcolorPixels = new byte[this.sensor.ColorStream.FramePixelDataLength];
                this.depthPixels = new byte[921604];
                this.rawdepthPixels = new DepthImagePixel[this.sensor.DepthStream.FramePixelDataLength];
                this.pointcloudbytes = new byte[3686404];
                this.cali_points = new ColorImagePoint[640 * 480];
                this.camera_calib = new CoordinateMapper(this.sensor);

                this.sensor.ColorFrameReady += this.SensorColorFrameReady;
                this.sensor.DepthFrameReady += this.SensorDepthFrameReady;

                try
                {
                    this.sensor.Start();
                }
                catch (IOException)
                {
                    this.sensor = null;
                    this.exit("");
                }

                Thread t = new Thread(new ThreadStart(this.updatecolor));
                t.Start();
            }
            else
            {
                this.exit("no kinect found. program ended.");
            }

        }

        private void exit(string mess)
        {
            MessageBox.Show(mess);
            Environment.Exit(0);
        }

        /// <summary>
        /// 关闭
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void mainw_Closed(object sender, EventArgs e)
        {
            if (null != this.sensor)
            {
                this.sensor.Stop();
            }
        }

        /// <summary>
        /// 彩色图像更新
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        /// 
        private void updatecolor()
        {
            while(true)
            {
                if (this.update_color)
                {
                    this.update_color = false;
                    using (ColorImageFrame colorFrame = this.color_e.OpenColorImageFrame())
                    {

                        if (colorFrame != null)
                        {
                            int indx = 0;
                            this.w_rgb_index++;

                            byte[] tem = BitConverter.GetBytes(this.w_rgb_index);
                            for (int i = 0; i < 4; i++)
                            {
                                this.colorPixels[indx++] = tem[i];
                            }
                            //图像反转到正常
                            colorFrame.CopyPixelDataTo(this.rwcolorPixels);
                            for (int i = 0; i < 480; i++)
                            {
                                for (int j = 639; j > -1; j--)
                                {
                                    for (int k = 2; k > -1; k--)
                                    { this.colorPixels[indx++] = this.rwcolorPixels[i * 640 * 4 + j * 4 + k]; }
                                }
                            }
                            this.SHrgb.Write(this.colorPixels, 0, 921604);
                            this.SHrgb_py.Write(this.colorPixels, 0, 921604);
                        }
                    }
                }
            }
        }


        private void SensorColorFrameReady(object sender, ColorImageFrameReadyEventArgs e)
        {
            this.update_color = true;
            this.color_e = e;
        }


        /// <summary>
        /// 深度图像更新
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void SensorDepthFrameReady(object sender, DepthImageFrameReadyEventArgs e)
        {
            using (DepthImageFrame depthFrame = e.OpenDepthImageFrame())
            {
                if (depthFrame != null)
                {
                    short dep;
                    byte[] pc = new byte[4];
                    byte[] rgb = new byte[3];
                    int inx = 0;
                    int tem_x,tem_y;
                    float xx,yy,zz;
                    float X, Y, Z;

                    byte[] wdepthindx = BitConverter.GetBytes(this.w_depth_index++);
                    for (int i=0;i<4;i++)
                    {   
                        this.pointcloudbytes[i]=wdepthindx[i];
                        this.depthPixels[i] = wdepthindx[i];
                    }

                    //获取深度
                    depthFrame.CopyDepthImagePixelDataTo(this.rawdepthPixels);
                    this.camera_calib.MapDepthFrameToColorFrame(DepthImageFormat.Resolution640x480Fps30, this.rawdepthPixels, ColorImageFormat.RgbResolution640x480Fps30, this.cali_points);

                    //依据映射关系重新生成配准深度图像
                    for (int i = 0; i < this.cali_points.Length; ++i)
                    {
                        inx = this.cali_points[i].Y * 640 + 639 - this.cali_points[i].X; //将深度图像左右反转,这个点真实对应的位置
                        dep = this.rawdepthPixels[i].Depth; //该像素点对应的校准后的深度

                        tem_x = 320 - this.cali_points[i].X;
                        tem_y = this.cali_points[i].Y-240;
                        xx = Convert.ToSingle(dep*tem_x*0.0995/52.5);
                        yy = Convert.ToSingle(dep * tem_y * 0.0995 / 52.5);
                        zz = Convert.ToSingle(dep);

                        //地面滤波
                        if (this.fg_flg)
                        {
                            if (Math.Abs(xx * this.rp.coe[0] + yy * this.rp.coe[1] + zz * this.rp.coe[2] + this.rp.coe[3]) < this.groundfilter_threshold)
                            {
                                xx = 0;
                                yy = 0;
                                zz = 0;
                            }
                        }

                        X = this.rp.Row1[0] * xx + this.rp.Row1[1] * yy + this.rp.Row1[2] * zz;
                        Y = this.rp.Row2[0] * xx + this.rp.Row2[1] * yy + this.rp.Row2[2] * zz;
                        Z = this.rp.Row3[0] * xx + this.rp.Row3[1] * yy + this.rp.Row3[2] * zz;

                        if ((Z<700) & (X>-400) & (X<400))   //轮椅自身范围
                        {
                            X = 0;
                            Y = 0;
                            Z = 0;
                            zz = 0;
                        }

                        pc = BitConverter.GetBytes(X);
                        for (int ii = 0; ii < 4; ii++)
                        { this.pointcloudbytes[4+inx * 12 + ii] = pc[ii]; }
                        pc = BitConverter.GetBytes(Y);
                        for (int ii = 0; ii < 4; ii++)
                        { this.pointcloudbytes[8 + inx * 12 + ii] = pc[ii]; }
                        pc = BitConverter.GetBytes(Z);
                        for (int ii = 0; ii < 4; ii++)
                        { this.pointcloudbytes[12 + inx * 12 + ii] = pc[ii]; }

                        //生成深度图
                        if (zz == 0)
                        { rgb = new byte[] { 0, 0, 0 }; }
                        else
                        { rgb = this.tr.getrgb(zz, new double[] { 1000, 8000 }); }
                        this.depthPixels[4+3 * inx] = rgb[0];
                        this.depthPixels[5 + 3 * inx] = rgb[1];
                        this.depthPixels[6 + 3 * inx] = rgb[2];
                    }
                    this.SHdepth.Write(this.depthPixels,0,921604);
                    this.SHpointcloud.Write(this.pointcloudbytes, 0, 3686404);

                    this.SHdepth_py.Write(this.depthPixels, 0, 921604);
                    this.SHpointcloud_py.Write(this.pointcloudbytes, 0, 3686404);
                }
            }
        }
    }
}
