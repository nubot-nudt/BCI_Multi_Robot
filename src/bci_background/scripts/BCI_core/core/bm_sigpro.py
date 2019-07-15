#!/usr/bin/env python
#-*- coding:utf-8 -*-

#Copyright (C) 2018, Nudt, JingshengTang, All Rights Reserved
#Author: Jingsheng Tang
#Email: mrtang@nudt.edu.cn


import os,sys
rootdir = os.path.split(os.path.realpath(__file__))[0]
import numpy as np
from storage import Store
from Queue import Queue as QQ
import multiprocessing
from multiprocessing import Queue

import threading
import platform
from threading import Lock
from copy import copy
import time
from multiprocessing import Event
from amp_simulator import AmpSimulator
from amp_actichamp_q import AmpActichamp
#from amp_xintuo import AmpXintuo
#from amp_mindwave import AmpMindwave

if platform.system()=='Linux':
    from linux_clock import clock as sysclock
else:
    from time import clock as sysclock

class sigpro(threading.Thread):
    configs = {   'Experiment': 'EEG experiment',
                  'SubjectName': 'subject',
                  'Session':    1,
                  'Directory':  rootdir + '//data',
                  'Amplifier':  'simulator',  #simulator,actichamp,mindwave,xintuo
                  'Ampconfigs': {},
                  'Channellist':[1,2,3,4,5],
                  'SamplingRate':200,
                  'SaveData':   False}
                  
    Trigger = {}

    def __init__(self,c2s,s2c,kil):
        self.c2s = c2s
        self.s2c = s2c
        self.kil = kil
        self.q = Queue()
        self.bb = QQ()
        # self.bb = Event()
        self.amp = None
        self.lstcount = 0
        self.check = 0  #标记是否进行过参数检查
        self._lock = Lock()
        threading.Thread.__init__(self)
        self.setDaemon(True)
        self.indx = 0

    
    def write_log(self,m):
        print '[Sigpro][%.4f]%s'%(sysclock(),m)
    
    def run(self):
        while True:
            tri = self.c2s.get()
            p = int(self.p_once*(sysclock() - self.sysclk)/0.05)   #计算当前时刻对应的采样点
            if p>self.p_once:
                p = -1
            if p<0:
                p = 0

            self._lock.acquire()
            for n in tri:
                if self.Trigger.has_key(n):
                    self.Trigger[n]=tri[n]
                    self.__trigger_ary[n][p:]=tri[n]

                else:
                    self.write_log('[warning] trigger error')
            self._lock.release()

    def init_trigger_ary(self):
        self._lock.acquire()
        for tri in self.Trigger:
            self.__trigger_ary[tri]=self.Trigger[tri]*np.ones(self.p_once)
        self._lock.release()

    def Initialize(self):
        pass

    def Process(self,signal,tri):
        return 0

    def StartRun(self):
        self.Initialize()    #初始化变量，更新配置等

        self.p_once = self.configs['SamplingRate']/20

        ampname = self.configs['Amplifier']
        if ampname == 'simulator':
            self.amp = AmpSimulator(self.configs)
        elif ampname == 'actichamp':
            self.amp = AmpActichamp(self.configs)
        elif ampname == 'mindwave':
            self.amp = AmpMindwave(self.configs)
        elif ampname == 'xintuo':
            self.amp = AmpXintuo(self.configs)
        else:
            raise IOError,'unsupported amplifier'

        if self.configs['SaveData']:
            st = Store(self.configs,self.q)
            p = multiprocessing.Process(target=st.run)
            p.daemon = True
            p.start()

        self.__trigger_ary = {}
        self.init_trigger_ary()

        self.sysclk = sysclock()
        self.start()
        print('bm_sigpropid',os.getpid())

        while True:
            if self.kil.is_set():
                break
            clk = sysclock()
            if clk-self.sysclk>0.05:
                # print(clk-self.sysclk)
                signal = self.amp.read()        #read data from amplifier
                # print('miao')
                # while head-self.indx<1:
                    # print('miao')
                    # print(head,self.indx)
                    # [head, signal] = self.amp.read()
                # self.indx = head
                # print(head)
                self.sysclk += 0.05
                tri = copy(self.__trigger_ary)  #get the trigger array
                if self.configs['SaveData']:
                    self.q.put([signal,tri])
                self.init_trigger_ary()         #reset trigger array
                self.s2c.put(self.Process(signal,tri))
            time.sleep(0.005)

        self.write_log('[info] process killed!')
        # sys.exit()####liu

