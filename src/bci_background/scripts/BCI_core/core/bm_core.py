#!/usr/bin/env python
#-*- coding:utf-8 -*-

#Copyright (C) 2018, Nudt, JingshengTang, All Rights Reserved
#Author: Jingsheng Tang
#Email: mrtang@nudt.edu.cn


import os,sys
file_dir = os.path.split(os.path.realpath(__file__))[0]
sys.path.append(os.path.split(file_dir)[0]+'//GuiMachine')

import numpy as np

import multiprocessing
from multiprocessing import Queue
from multiprocessing import Event
from bm_phase import *
from gui import gui_process
import random
import threading

try:    __INF__ = float('inf')
except: __INF__ = 0xFFFF

import platform

if platform.system()=='Linux':
    from linux_clock import clock as sysclock
else:
    from time import clock as sysclock


class core(threading.Thread):
    p2c = Queue()   #phase -> core
    c2p = Queue()   #core -> phase
    c2g = Queue()   #core -> gui
    s2c = Queue()   #sigpro -> core
    c2s = Queue()   #core -> sigpro
    e4s = Event()   #end -> sigpro
    g2p = Event()

    PHASES = []
    stimuli = {}
    
    def __init__(self):
        threading.Thread.__init__(self)
        self.setDaemon(True)

    def setTrigger(self,triggers):
        self.c2s.put(triggers)

    def change_phase(self,phase):
        self.c2p.put(['change',phase])
    
    def write_log(self,m):
        print '[Core][%.4f]%s'%(sysclock(),m)

    def mainloop(self):
        while True:
            ph = self.p2c.get()
            self.Transition(ph)
            if ph == 'stop':
                self.c2g.put('_q_u_i_t_')
                self.e4s.set()
                break

    def Transition(self,phase): #implement in the app level
        pass
    
    def GuiUpdate(self,sti):
        self.c2g.put(sti)

    def Initialize(self):
        pass

    def Process(self,res):
        pass

    def run(self):
        while True:
            self.Process(self.s2c.get())

    def StartRun(self): #mainloop
        print('bm-corepid',os.getpid())
        self.Initialize()
        p1 = multiprocessing.Process(target=gui_process,args=(self.stimuli,self.c2g,self.g2p))
        p2 = multiprocessing.Process(target=phase_process,args=(self.PHASES,self.p2c,self.c2p,self.g2p))
        p1.start()
        p2.start()
        self.start()
        self.mainloop()
        self.write_log('[info] process killed!')
