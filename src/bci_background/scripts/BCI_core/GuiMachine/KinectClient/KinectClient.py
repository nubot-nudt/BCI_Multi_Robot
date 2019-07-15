#!/usr/bin/env python
#coding:utf-8
import mmap
import numpy as np
import pygame
from pygame.locals import *
from process_op import check_exsit,kill_process
import time
from struct import pack,unpack
import os,sys

rootdir = os.path.dirname(__file__)

class KinectClientV2(object):
    """
    author: mrtang
    date: 2017.5
    version: 1.0
    email: mrtang@nudt.edu.cn
    
    A Kinect server runs in another process to capture RGB video and depth image,
    and futrher recoginze accessible are, self pose and detect interesting buttons,
    this information were ongoing written in shared memeory in special format.
    This class is used to read and unpack these data.
    """
    def __init__(self):
        self.servername = 'KinectBase.exe'
        if not check_exsit(self.servername):    os.startfile(rootdir + '/' + self.servername)
        print 'kinect driver is running...'
        time.sleep(3)

        self.SHrgb = mmap.mmap(0,921604,access=mmap.ACCESS_READ,tagname='_sharemem_for_colorpixels_cspy')
        self.SHdepth = mmap.mmap(0,921604,access=mmap.ACCESS_READ,tagname='_sharemem_for_depthpixels_cspy')
        self.SHpointcloud = mmap.mmap(0,3686404,access=mmap.ACCESS_READ,tagname='_sharemem_for_point_cloud_cspy') #double

        self.__color_image = pygame.surface.Surface((640,480))
        self.__depth_image = pygame.surface.Surface((640,480))
        self.__pointcloud = np.zeros((480,640,3))
        self.__color_index = self.__depth_index = self.__pc_index = 0

    def __del__(self):
        if check_exsit(self.servername):    kill_process(self.servername)

    @property
    def ColorImage(self):
        self.SHrgb.seek(0)
        newind = unpack('i',self.SHrgb.read(4))[0]
        if newind!=self.__color_index:
            self.__color_index = newind
            self.__color_image = pygame.image.frombuffer(self.SHrgb.read(921600),(640,480),'RGB').convert()
        return self.__color_image

    @property
    def DepthImage(self):
        self.SHdepth.seek(0)
        newind = unpack('i',self.SHdepth.read(4))[0]
        if newind!=self.__depth_index:
            self.__depth_index = newind
            try: self.__depth_image = pygame.image.frombuffer(self.SHdepth.read(921600),(640,480),'RGB').convert()
            except:pass
        return self.__depth_image

    @property
    def PointCloud(self):
        self.SHpointcloud.seek(0)
        newind = unpack('i',self.SHpointcloud.read(4))[0]
        if newind!=self.__pc_index:
            self.__pc_index = newind
            self.SHpointcloud.seek(4)
            self.__pointcloud = np.fromstring(self.SHpointcloud.read(3686400),dtype=np.float32).reshape((480,640,3))
        return self.__pointcloud

    def getXYZ(self,p): #pΪͼ������ ����
        return self.PointCloud[p[1],p[0],:]

    def saveim(self,filename,im = 'color'):
        if im == 'color':pygame.image.save(self.__color_image,filename)
        elif im=='depth':pygame.image.save(self.__depth_image,filename)

#-------------------------------------------------------
def example():
    import numpy.linalg as nlg
    pygame.init()
    clk = pygame.time.Clock()
    kk = KinectClientV2()
    screen = pygame.display.set_mode((1280,480), 0,32)
    END=0
    pp = []
    pppp = [0,0]
    while True:
        screen.blit(kk.DepthImage,(640,0))
        screen.blit(kk.ColorImage,(0,0))
        ev = pygame.event.get()
        for e in ev:
            if e.type==MOUSEBUTTONUP:
                pppp[0] = e.pos[0]
                pppp[1] = e.pos[1]
                if e.pos[0]>640:    pppp[0]-=640
                p = kk.getXYZ(pppp).astype(np.int32)
                pp.append(p)
                print p
                if len(pp)==2:
                    print nlg.norm(pp[0]-pp[1])
                    pp = []
            elif e.type==QUIT:
                END=1

        # pygame.draw.line(screen,(255,0,0),(0,240),(640,240))
        # pygame.draw.line(screen,(255,0,0),(320,0),(320,480))
        pygame.display.update()

        clk.tick(100)
        if END:break
    del kk
    pygame.quit()

if __name__ == "__main__":
    example()
