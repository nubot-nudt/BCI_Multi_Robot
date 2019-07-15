#!/usr/bin/env python
#-*- coding:utf-8 -*-

#Copyright (C) 2018, Nudt, JingshengTang, All Rights Reserved
#Author: Jingsheng Tang
#Email: mrtang@nudt.edu.cn


import os,sys
import time
import platform

if platform.system()=='Linux':
    from linux_clock import clock as sysclock
else:
    from time import clock as sysclock

rootdir = os.path.split(os.path.realpath(__file__))[0]
sys.path.append(rootdir+'//KinectClient')
# from kinect_camera import KinectCamera

import pygame
from pygame.locals import *
from block import Block
# from usb_camera import UsbCamera
from imagebox import Imagebox
from imagearray import Imagearray
from multiprocessing import Queue

import threading

class GUImachine(threading.Thread):
    stimuli = {}
    __release_ID_list = []

    def __init__(self,stims,c2g,g2p):
        super(GUImachine,self).__init__()
        self.g2p = g2p
        self.quitgui = False
        pygame.init()
        self.c2g = c2g
        self.lock = threading.Lock()
        self.screen = pygame.display.set_mode(stims['screen']['size'],DOUBLEBUF,24)
        self.screen_color = stims['screen']['color']
        try:    self.fps = stims['screen']['fps']
        except: self.fps = 60
        self.screen.fill(self.screen_color)
        pygame.display.set_caption('Bciros Gui')
        del stims['screen']
        
        #register stimulis
        for ID in stims:
            element = stims[ID]
            if element['class'] == 'Block': self.stimuli[ID] = Block(self.screen,**element['parm'])
            elif element['class'] == 'Imagebox':self.stimuli[ID] = Imagebox(self.screen,**element['parm'])
            elif element['class'] == 'Imagearray': self.stimuli[ID] = Imagearray(self.screen,**element['parm'])
            elif element['class'] == 'UsbCamera':self.stimuli[ID] = UsbCamera(self.screen,**element['parm'])
            elif element['class'] == 'KinectCamera':
                self.stimuli[ID] = KinectCamera(self.screen,**element['parm'])
                # self.__release_ID_list.append(ID)   #进入回收列表，需要在结束程序时释放#原
            self.__release_ID_list.append(ID)   #进入回收列表，需要在结束程序时释放#刘

    def write_log(self,m):
        print '[Gui][%.4f]%s'%(sysclock(),m)

    def run(self):
        while True:
            arg = self.c2g.get()
            if arg == '_q_u_i_t_':
                self.quitgui = True
                break
            self.lock.acquire()
            [self.stimuli[id].reset(**arg[id]) for id in arg.keys()]
            self.lock.release()

    def StartRun(self):
        END = 0
        tick = pygame.time.Clock()
        while True:
            self.screen.fill(self.screen_color)
            stis = sorted(self.stimuli.items(),key=lambda k:k[1].layer)
            for s in stis:  s[1].show()
            pygame.display.flip()

            if self.quitgui:    break

            evs = pygame.event.get()
            for e in evs:
                if e.type == QUIT:
                    END=1
                elif e.type == KEYDOWN:
                    if e.key == K_ESCAPE: END=1
            if END:
                self.g2p.set()
                break
            tick.tick(75)
        pygame.quit()


        for ID in self.__release_ID_list:   del self.stimuli[ID]
        self.write_log('[info] process killed!')


def gui_process(sti,q,g2p):
    g = GUImachine(sti,q,g2p)
    g.start()
    g.StartRun()

def test():
    sti = {'screen':{'size':(600,500),'color':(0,0,0)},
           'ary':{'class':'Imagearray','parm':{'imagesource':rootdir+'\\testary','visible':True,
                                                'loops':4,'play':-1,'fps':40,'size':(400,500)}}
           }
    g = GUImachine(sti,Queue())
    g.start()
    g.StartRun()

if __name__ == '__main__':
    test()

