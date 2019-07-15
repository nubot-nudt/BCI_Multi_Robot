#!/usr/bin/env python
#-*- coding:utf-8 -*-


import numpy as np
import socket
import multiprocessing
import platform
import mmap
import sys
import struct
# import Queue
from multiprocessing import Queue
import time
if platform.system()=='Linux':
    from linux_clock import clock as sysclock
else:
    from time import clock as sysclock



def sock_process(q):
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind(('192.168.43.162', 34217 ))
    except socket.error as msg:
        print msg
        sys.exit(1)
    # clk = sysclock()
    while True:
        # cl = sysclock()
        # print(cl - clk)
        # clk = sysclock()
        data = s.recv(25612)
        # datanew = struct.unpack('{0}f'.format(len(data)/4),data)
        # try:
        # if not q.empty():
        #     a = q.get()
        q.put(data)



class AmpActichamp(object):
    def __init__(self,configs):
        self.q = Queue(maxsize=1)
        p = multiprocessing.Process(target=sock_process,args=(self.q,))
        p.daemon = True
        p.start()
        self.configs = configs
        self.p_once = self.configs['SamplingRate']/20
        self.chlst = np.array(self.configs['Channellist'])
        self.ch = len(self.configs['Channellist'])
        self.bytenum = self.p_once*self.ch*4
        self.check = True
        self.indx = 0

    def read(self):
        # while True:
            # try:
            # if not self.q.empty():

        data1 = self.q.get()
        data1 = np.fromstring(data1,dtype=np.float32)
        if self.check:

            buf = data1[0:3]
            self.indx = buf[2]
            if buf[0]!=self.configs['SamplingRate']:
                raise IOError,'samplingrate does not match!'

            # print(len(data1[3:]))
            eeg = data1[3:].reshape(self.ch,self.p_once)#####
            self.check = False
        else:
            buf = data1[0:3]
            eeg = data1[3:].reshape(self.ch,self.p_once)#####
            # print(buf[2])
            if buf[2]-self.indx<1:
                print('[actichamp] old data returned!')
            elif buf[2]-self.indx>1:
                print('[actichamp] late package!')
            self.indx = buf[2]
            # return eeg
            #     break
            # except self.q.empty():
            # else:
            #     print('read eeg sock late!')
            #     time.sleep(0.002)
        return eeg
