#!/usr/bin/env python
#-*- coding:utf-8 -*-

#Copyright (C) 2018, Nudt, JingshengTang, All Rights Reserved
#Author: Jingsheng Tang
#Email: mrtang@nudt.edu.cn


import sys
import os
import platform
import time

self_name = 'Phase'

if platform.system()=='Linux':
    from linux_clock import clock as sysclock
else:
    from time import clock as sysclock

try:    __INF__ = float('inf')
except: __INF__ = 0xFFFF

def register_phase(arg):    #接受一个列表
    PHASES = {}
    PHASES['start'] = {'next': '', 'duration': __INF__}
    PHASES['stop'] = {'next': '', 'duration': __INF__}
    for item in arg:
        if item.has_key('duration'):
            PHASES[item['name']]={'next':item['next'],'duration':item['duration']}
        else:
            PHASES[item['name']]={'next':'','duration':__INF__}
    return PHASES

def write_log(m):
    print '[%s][%3.4f]%s'%(self_name,sysclock(),m)

def phase_process(ph,p2c,c2p,g2p):
    PHASES = register_phase(ph)
    time.sleep(1)
    current_phase = 'start' #phase必须从start开始
    p2c.put(current_phase)
    _clk = sysclock()

    while True:
        clk = sysclock()

        if clk - _clk > PHASES[current_phase]['duration']:
            current_phase = PHASES[current_phase]['next']
            p2c.put(current_phase)
            _clk = clk
        
        if not c2p.empty():
            typ,p = c2p.get()
            if typ == 'change' and PHASES.has_key(p):
                current_phase = p
                p2c.put(current_phase)
            else:
                write_log(self_name,'[warning] someting error with <%s %s>'%(typ,p))

        if g2p.is_set():
            p2c.put('stop')
            break

        if current_phase == 'stop': break
        time.sleep(0.005)
    
    write_log('[info] process killed!')
