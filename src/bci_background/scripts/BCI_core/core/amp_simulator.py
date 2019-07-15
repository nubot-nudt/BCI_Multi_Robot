#!/usr/bin/env python
#-*- coding:utf-8 -*-

#Copyright (C) 2018, Nudt, JingshengTang, All Rights Reserved
#Author: Jingsheng Tang
#Email: mrtang@nudt.edu.cn


import numpy as np

class AmpSimulator(object):
    def __init__(self,configs):
        self.configs = configs
        self.p_once = self.configs['SamplingRate']/20
        self.ch = len(self.configs['Channellist'])

    def read(self):
        return np.random.rand(self.ch,self.p_once).astype(np.float32)
