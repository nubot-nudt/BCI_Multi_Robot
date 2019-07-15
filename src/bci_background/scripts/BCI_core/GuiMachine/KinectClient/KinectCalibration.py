#!user/bin/python
# -*-coding:utf-8-*-

#FileName: camera_setting_calibration.py
#Version: 1.0
#Author: Jingsheng Tang
#Date: 2017/9/11
#Email: mrtang@nudt.edu.cn
#Github: trzp


# 该脚本用于摄像机安装标定。摄像机安装在某设备上时，为了能够将摄像机获取的坐标从
# 摄像机坐标系转化到目标坐标系，我们运用该程序获取过渡矩阵。以轮椅安装为例。摄像
# 头以某一个倾斜角度安装，我们期望目标坐标系与轮椅前进方向保持一致，并且竖直方向
# 与地球引力方向一致。我们将轮椅调整到正对一面墙，并且该墙上悬挂一个铅垂线。该墙
# 面和铅垂线定义了世界坐标系。我们测量墙面的法向量作为z方向，铅垂线作为y方向。进
# 而求得过渡矩阵。过渡矩阵求取参考http://blog.csdn.net/u011240016/article/details/52821139


import os,sys
farpath = os.path.split(sys.argv[0])[0]
os.sys.path.append(farpath+'/Param')
os.sys.path.append(farpath+'/User')

from KinectClient import KinectClientV2
import numpy as np
import pygame
from pygame.locals import *
import time
import win32api

class Calib_Base:
    def __init__(self):
        self.Z = None   #目标坐标系的zyz
        self.Y = None
        self.X = None
        self.P = None

    def convert(self,pc): #pc r*c*3 pointcloud ndarray
        s = pc.shape
        pp = pc.flatten().reshape((s[0]*s[1],3)).transpose()
        ind = np.where(pp[2,:]==0)[0]
        pp = np.delete(pp,ind,axis=1)
        return np.asmatrix(pp)  #return x*3 matrix

    def __PlaneFitting(self,pp):#3*x
        cpp = np.cov(pp)
        a,v = np.linalg.eig(cpp)
        ind = np.argsort(a)[0]
        vv = v[:,ind]
        mvv = np.asmatrix(vv).T
        m = -(np.mean(pp,axis=1).T*mvv)[0,0]
        return np.hstack((vv,m))

    def RPlaneFitting(self,pp): #平面系数1,2,3,4，阈值
        while True:
            coe = self.__PlaneFitting(pp)
            num = pp.shape[1]
            v = np.asmatrix(coe[:3])
            ds = v*pp
            md = np.mean(ds)
            nds = ds-md
            threshold = 3*np.sqrt(nds*nds.T/(num-1))
            ind = np.where(nds>threshold)[1]
            if ind.size==0: break
            pp = np.delete(pp,ind,axis=1)

        coe = coe*np.sign(coe[2])   #令z为正
        self.Z = coe[:3]
        return np.append(coe,threshold)

    def normalizev(self,v):
        return v/np.linalg.norm(v)

    def SetVertical(self,pos):  #该方法将铅垂线上两点投影到法向量定义的平面上，从而定义Y方向
        v = pos[1]-pos[0]
        h = sum(v*self.Z)
        vv = v - self.Z*h
        self.Y = self.normalizev(vv)
        return self.Y

    def getX(self):     #在定义了y,z方向之后，通过叉积计算求取x方向，至此，目标坐标系建立
        #求x  yxz -> x
        ax,ay,az = self.Y
        bx,by,bz = self.Z
        cx = ay*bz - az*by
        cy = -az*bx + ax*bz
        cz = ax*by - ay*bx
        v = self.normalizev(np.array((cx,cy,cz)))
        self.X = v

    def getP(self): #过度矩阵 p = alpha^-1*beta=beta (alpha单位矩阵)
        self.P = np.asmatrix(np.vstack((self.X,self.Y,self.Z)))
        return self.P

    def Calib(self,pointcloud,veticalP):
        pp = self.convert(pointcloud)
        self.RPlaneFitting(pp)      #通过拟合平面求Z向量
        self.SetVertical(veticalP)  #通过指定的两点求Y向量
        self.getX()                 #通过yz求x
        return self.getP()                 #求过度矩阵

    def saveparam_npy(self,path,content):
        np.save(path,content)

    def saveparam_txt(self,path,content):
        sst = content.flatten().astype(np.float32).tostring()
        file = open(path,'w')
        file.write(sst)
        file.close()

class FitGround: #用来求取世界坐标系过渡矩阵
    def __init__(self):
        win32api.MessageBox(0,u'正在求取地面拟合参数，应当在摄像机坐标系下进行')
        pygame.init()
        self.screen = pygame.display.set_mode((640,480),0,32)
        self.kinect = KinectClientV2()
        self.sc = Calib_Base()
        time.sleep(0.5)
    
    def do(self,path):
        p = []
        END = False
        win32api.MessageBox(0,u'选择两个角点定义地面')
        win32api.MessageBox(0,u'选择左上角点(右键)')
        while True:
            self.screen.blit(self.kinect.ColorImage,(0,0))
            evs = pygame.event.get()
            for ev in evs:
                if ev.type == QUIT:
                    END = True
                if ev.type == MOUSEBUTTONUP and ev.button == 3:
                    if len(p)==0:
                        p.append(ev.pos)
                        win32api.MessageBox(0,u'选择右下角点')
                    elif len(p)==1:
                        p.append(ev.pos)
                        END = True
                        break
            time.sleep(0.2)
            pygame.display.update()
            if END:break

        pc = self.kinect.PointCloud[int(p[0][1]):int(p[1][1]),int(p[0][0]):int(p[1][0]),:]
        pp = self.sc.convert(pc)
        coe = self.sc.RPlaneFitting(pp)
        self.sc.saveparam_txt(path,coe)
        win32api.MessageBox(0,u'地面拟合完成')
        pygame.quit()

class CameraSettingCalibration:
    def __init__(self):
        win32api.MessageBox(0,u'正在求取摄像机安装参数')
        pygame.init()
        self.screen = pygame.display.set_mode((640,480),0,32)
        self.kinect = KinectClientV2()
        self.sc = Calib_Base()
        self.sw2 = False
        time.sleep(0.5)

    def do(self,path):
        sw = False
        p = []
        p2 = []
        tem = []
        END = False
        if not self.sw2:
            win32api.MessageBox(0,u'选择两个角点定义基准平面')
            win32api.MessageBox(0,u'选择左上角点(右键)')
        while True:
            self.screen.blit(self.kinect.ColorImage,(0,0))
            # pygame.draw.line(self.screen,(255,0,0),(320,0),(320,480))
            evs = pygame.event.get()
            for ev in evs:
                if ev.type == QUIT:
                    END = True
                if not self.sw2: #确定平面
                    if ev.type == MOUSEBUTTONUP and ev.button == 3:
                        if len(p)==0:
                            p.append(ev.pos)
                            win32api.MessageBox(0,u'选择右下角点')
                        elif len(p)==1:
                            p.append(ev.pos)
                            win32api.MessageBox(0,u'选择完毕,请沿竖直向下方向确定两点')
                            sw = True
                            break

                        if sw: #确定铅垂线
                            if len(p2)==0:
                                pv = self.kinect.getXYZ(ev.pos)
                                if pv[2]==0:
                                   win32api.MessageBox(0,u'失败，请重新选择')
                                else:
                                    p2.append(pv)
                                    win32api.MessageBox(0,u'成功，请选择下面点')
                            elif len(p2)==1:
                                pv = self.kinect.getXYZ(ev.pos)
                                if pv[2]==0:
                                   win32api.MessageBox(0,u'失败，请重新选择')
                                else:
                                    p2.append(pv)
                                    win32api.MessageBox(0,u'成功')

                                    pc = self.kinect.PointCloud[int(p[0][1]):int(p[1][1]),int(p[0][0]):int(p[1][0]),:]
                                    print 'calibration matrix:'
                                    res = self.sc.Calib(pc,p2)
                                    print res
                                    self.sc.saveparam_txt(path,res)
                                    win32api.MessageBox(0,u'过度矩阵计算完成')
                                    END = True
                else:
                    if ev.type == MOUSEBUTTONUP:
                        if len(tem)<2:
                            tem.append(self.kinect.getXYZ(ev.pos))
                            print 'point',tem[-1]
                        if len(tem)==2:
                            print 'distance:',np.linalg.norm(tem[0]-tem[1])
                            tem = []
                            print '---------------------------------------'
            time.sleep(0.2)
            pygame.display.update()
            if END:break
        if self.sw2:pygame.quit()

if __name__ == '__main__':
    import sys,os
    path = os.path.split(sys.argv[0])[0]+'\\groundfilter_param.txt'
    ca = FitGround()
    ca.do(path)

    # path = os.path.split(sys.argv[0])[0]+'\\installtion_param.txt'
    # ca = CameraSettingCalibration()
    # ca.do(path)
