#!/usr/bin/env python
# -*-coding:utf-8-*-

#FileName: camera.py
#Version: 1.0
#Author: Jingsheng Tang
#Date: 2017/12/27
#Email: mrtang@nudt.edu.cn
#Github: trzp

from __future__ import division
from VideoCapture import Device
import pygame
from pygame.locals import *
import os
import time
from pygame_anchors import *

class UsbCamera(object):
    size = (640,480)
    position = (0,0)
    anchor = 'center'
    device_num = 0
    layer = 0
    visible = False
    update = True
    Fps = 20
            
    parmkeys = ['size','position','anchor','device_num','layer','visible','Fps','update']
    sur = None

    def __init__(self,root,**argw):
        self.root = root
        self.update_parm(**argw)
        self.initcamera()
        self.pt = 1/self.Fps
        self.clk = time.clock()
        self.reset(**argw)
        print '============================'
    
    def initcamera(self):
        self.cam = Device(self.device_num)
        
    def capture(self):
        return pygame.image.fromstring(self.cam.getImage().tostring(),(640,480),'RGB').convert()

    def update_parm(self,**argw):
        for item in argw:   exec('self.%s = argw[item]'%(item))  

    def reset(self,**argw):
        self.update_parm(**argw)
        self.blitp = blit_pos1(self.size,self.position,self.anchor)

    def show(self):
        if self.visible:
            if self.update and time.clock()-self.clk > self.pt:
                self.sur = self.capture()
                if self.size[0]!=640 or self.size[1]!=480:  self.sur = pygame.transform.scale(self.sur,self.size)
                self.clk = time.clock()
            if self.sur != None:
                self.root.blit(self.sur,self.blitp)