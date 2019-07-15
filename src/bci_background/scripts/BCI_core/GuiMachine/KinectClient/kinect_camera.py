#!/usr/bin/env python
# -*-coding:utf-8-*-

#FileName: camera.py
#Version: 1.0
#Author: Jingsheng Tang
#Date: 2017/12/27
#Email: mrtang@nudt.edu.cn
#Github: trzp

from __future__ import division
import pygame
from pygame.locals import *
import os

from KinectClient import KinectClientV2
from usb_camera import UsbCamera


class KinectCamera(UsbCamera):
    def __init__(self,root,**argw):
        super(KinectCamera,self).__init__(root,**argw)
    
    def initcamera(self):
        self.kinect = KinectClientV2()
    
    def capture(self):
        return self.kinect.ColorImage