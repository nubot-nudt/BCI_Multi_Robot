#!/usr/bin/env python
#-*- coding:utf-8 -*-

#Copyright (C) 2018, Nudt, JingshengTang, All Rights Reserved
#Author: Jingsheng Tang
#Email: mrtang@nudt.edu.cn


import numpy as np
import socket
import multiprocessing
# import platform
import mmap
import sys
import struct

import platform

if platform.system()=='Linux':
    from linux_clock import clock as sysclock
else:
    from time import clock as sysclock

def sock_process():
    f = open('_tem4mmap_.dat', 'w')
    f.write('\x00' * 25612)
    f.close()
    f = open('_tem4mmap_.dat', 'r+')
    mm = mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_WRITE)
    mm.seek(0)
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind(('192.168.43.162',34217))
        # s.connect(('192.168.43.225', 6666 ))
    except socket.error as msg:
        print msg
        sys.exit(1)
    # s.sendall('hello')
    clk = sysclock()
    # ind = 0
    while True:
        # ind+=1

        cl = sysclock()
        print(cl - clk)
        clk = sysclock()
        data = s.recv(25612)

        datanew = struct.unpack('{0}f'.format(len(data)/4),data)
        print(datanew[2])
        # if ind == 2:
        #     ind = 0
        #     mm.seek(0)
        mm.seek(0)
        # # print('writepo',mm.tell())
        mm.write(data)

class AmpActichamp(object):
    def __init__(self,configs):
        p = multiprocessing.Process(target=sock_process)
        p.daemon = True
        p.start()

        self.configs = configs
        self.p_once = self.configs['SamplingRate']/20
        self.chlst = np.array(self.configs['Channellist'])
        self.ch = len(self.configs['Channellist'])
        self.bytenum = self.p_once*self.ch*4
        self.check = True
        f = open('_tem4mmap_.dat','r')
        self.mm = mmap.mmap(f.fileno(),0,access=mmap.ACCESS_READ)
        self.indx = 0
        self.che = 1


    def read(self):



        if self.check:
            self.mm.seek(0)
            buf = np.fromstring(self.mm.read(12),dtype=np.float32)
            # buf = np.fromstring(self.mm.read(24),dtype=np.float64)
            # print(buf)
            # if buf[0]!=self.configs['SamplingRate']:
            #     print(buf[0])
                # raise IOError,'samplingrate does not match!'
            if self.ch>buf[1]:
                print(self.ch,buf)
                raise IOError,'request too much channels of signal!'
            # if np.max(self.chlst)>=buf[2]:
            #     raise IOError,'channel indices exceeded the source signal!'
            self.indx = buf[2]
            eeg = np.fromstring(self.mm.read(self.bytenum),dtype=np.float32).reshape(self.ch,self.p_once)
            # buf2 = self.mm.read(24)
            # eeg2 = np.fromstring(self.mm.read(self.bytenum), dtype=np.float64).reshape(self.ch, self.p_once)
            # eeg = np.hstack((eeg1, eeg2))

            self.check = False
        else:
            self.mm.seek(0)
            buf = np.fromstring(self.mm.read(12),dtype=np.float32)
            eeg = np.fromstring(self.mm.read(self.bytenum),dtype=np.float32).reshape(self.ch,self.p_once)
            # print('readpo', self.mm.tell())
            # buf2 = np.fromstring(self.mm.read(24),dtype=np.float64)
            # eeg2 = np.fromstring(self.mm.read(self.bytenum),dtype=np.float64).reshape(self.ch,self.p_once)
            # eeg = np.hstack((eeg1,eeg2))
            # print(buf[2])


            if buf[2] - self.indx<1:
                print '[actichamp] old data returned!'
            elif buf[2] - self.indx>1:
                print '[actichamp] late package!'
            self.indx = buf[2]
            # print(self.indx)
        # print('indx',self.indx)
        return [self.indx,eeg]

if __name__ == '__main__':

    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind(('192.168.43.162',34217))
        # s.connect(('192.168.43.225', 6666 ))
    except socket.error as msg:
        print msg
        sys.exit(1)
    # s.sendall('hello')
    clk = sysclock()
    while True:
        cl = sysclock()
        if cl - clk>0.06:
            print(cl - clk)
        clk = sysclock()

        data = s.recv(20000)
        # print(len(data))
        # datanew = struct.unpack('{0}f'.format(len(data)/4),data)
        # print(datanew[2])
