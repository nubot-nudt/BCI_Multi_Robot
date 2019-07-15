#!/usr/bin/env python
#-*- coding:utf-8 -*-


import os,sys
file_dir = os.path.split(os.path.realpath(__file__))[0]
rootdir = os.path.split(file_dir)[0]
rootdir = os.path.split(rootdir)[0]
# print rootdir
sys.path.append(rootdir+'/BCI_core/core')
from bm_core import core
from bm_sigpro import sigpro
import numpy as np
import random
import multiprocessing
import scipy.io as sio
import scipy.signal
# import threading
# from multiprocessing import Queue
from random import choice
from sklearn.cross_decomposition import CCA
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int8
from nubot_common.msg import StrategyInfo
import logging
# import pygame
# pygame.mixer.set_num_channels(8)

import platform
if platform.system()=='Linux':
    from linux_clock import clock as sysclock
else:
    from time import clock as sysclock

class BCImain(core):
    def __init__(self,Config):
        super(BCImain,self).__init__()
        self.mode = Config[1]
        self.phaseflg = -1
        self.cueflg = 0

        # self.cameraflagpub = rospy.Publisher('cameraflag', String, queue_size=1)
        # rospy.init_node('BCI_moudle', anonymous=False)
        # rospy.Subscriber('/nubot1/nubotcontrol/recommend_strategy', String, self.callback1)####String and char
        #/***待修改

        self.asyn_stoptime =0
        self.asyn_detecttime =0
        self.syn_starttime =0
        self.syn_stoptime =0
        self.syn_detecttime =0
        self.onlinetasks = ['']#***/

        self.result1 = [0,0,0]
        self.score_threshold = 0.205
        self.activelayer = 1  #### 为了确定当前刺激数量 取值为 1,2,3,4
        self.stinum = [4,4,6,4]
        self.result_pub = rospy.Publisher('BCI_background/bci_signal', Int8, queue_size=1)###########

        configs = Config[0]
        logpath = configs['Directory'] + "/" + configs['SubjectName'] +"log"+ "/"
        if not os.path.exists(logpath):
            os.makedirs(logpath)
        lognum = len(os.listdir(logpath))
        logname = logpath+configs['SubjectName'] + '-' + str(lognum) + ".log"
        logger = logging.getLogger("asynexp")
        logger.setLevel(logging.INFO)
        nm = logging.FileHandler(logname)
        logger.addHandler(nm)
        self.logger = logger


    def Initialize(self):
        self.PHASES = [  {'name': 'start',      'next':'stop'},

                         {'name': 'cue',       'next': 'braindetect',       'duration': 2},
                         {'name': 'braindetect',       'next': 'resultrecord',       'duration': 3},
                         {'name': 'resultrecord',     'next': 'start',      'duration':1},##record the result,待修改

                         {'name': 'activeselect',     'next': 'temp',      'duration':4},#######主动发起
                         {'name': 'temp',     'next': 'activeselect',      'duration':0.5},

                         {'name':'stop'}
                      ]
        # scrw = 800
        # scrh = 600
        # self.stimuli['screen'] = {'size':(100,100),'color':(0,0,0)}
        # self.stimuli['cue'] = {'class':'Block',
        #                        'parm':{'size':(1000,60),'position':(scrw/2,scrh/2),
        #                                'anchor':'center',
        #                                'visible':True,
        #                                'forecolor':(0,0,0),
        #                                'text':'',
        #                                'textsize':50,
        #                                'textcolor':(0,0,255),
        #                                'textanchor':'center'}}



    def Transition(self,phase):
        # print(phase)
        if phase == 'start':
            print('phase: start')
            self.phaseflg = 0
            self.setTrigger({'state': 0, 'freqnum': 0})

        elif phase == 'cue':
            print('phase: cue')
            self.phaseflg = 1
            self.setTrigger({'state': 1, 'freqnum': 0})
            self.cueflg = 0

        elif phase == 'braindetect':  ## include syn and asyn detection
            print('phase: braindetect')
            self.phaseflg = 2
            self.setTrigger({'state': 2, 'freqnum': 0})

        elif phase == 'resultrecord':
            print('phase: resultrecord')
            self.phaseflg = 3
            self.setTrigger({'state': 3, 'freqnum': 0})
            self.result_pub.publish(self.result1.index(max(self.result1)))#############
            self.result1 = [0,0,0]

        elif phase == 'activeselect':
            self.phaseflg = 4
            self.setTrigger({'state': 4, 'freqnum': self.stinum[self.activelayer - 1]})
            print('phase: activeselect')

        elif phase == 'temp':
            print('phase: temp')
            self.phaseflg = 5
            self.setTrigger({'state': 5, 'freqnum': 0})
            result_score = max(self.result1)
            self.result_pub.publish(self.result1.index(result_score))
            if self.activelayer == 1:
                if self.result1.index(result_score) == len(self.result1)-1:  #######  return
                    self.change_phase('start')
                elif self.result1.index(result_score) == len(self.result1)-2:
                    self.activelayer = 3
                else:
                    self.activelayer = 2
            else:
                if self.result1.index(result_score) == len(self.result1)-1:  ###### return
                    self.activelayer = self.activelayer - self.activelayer % 2 - 1
                else:
                    if self.activelayer % 2:
                        self.activelayer = self.activelayer + 1
                    else:
                        self.activelayer = 1
                        self.change_phase('start')

            # self.syn_stoptime = sysclock()
            # self.logger.info("      synchronous task stop:"+str(self.syn_stoptime))
            # self.syn_detecttime = self.syn_stoptime - self.syn_starttime
            # self.logger.info("      synchronous task detection:"+str(self.syn_detecttime))
            # self.setTrigger({'state': 3, 'code': 0})###
            # self.logger.info("  synresult:"+str(self.useraction))
        # elif phase == 'stop':
            # pygame.quit()

    def Process(self,res):  #res来自信号处理模块的结果，长度为1或4的list，包括异步和同步结果：[asyn,syn(三个概率)]
        if self.phaseflg == 0:###start
            if res == [2]:
                print('startmiao')
                self.result_pub.publish(-1)
                self.change_phase('activeselect')

        elif self.phaseflg == 2:###braindetect,待修改~~~
            if res[0] == 1:###检测到异步信号，表示认同当前recommend strategy，不进行修改
                self.change_phase('resultrecord')

            elif res[0] == 0 and len(res)>1:
                result_score = max(res[1:])
                self.result1 = res[1:]
                if result_score > self.score_threshold:
                # if len(res)>1 and not res[1:] == [0,0,0]:
                    self.change_phase('resultrecord')
        elif self.phaseflg == 4:
            if len(res)>1:
                result_score = max(res)
                self.result1 = res
                if result_score > self.score_threshold:
                    self.change_phase('temp')

    def recommend_subscribe(self):
        rospy.init_node('BCI_moudle', anonymous=False)
        rospy.Subscriber('/nubot1/nubotcontrol/recommend_strategy', StrategyInfo, self.callback1)
        rospy.spin()

    def callback1(self,data):
        # self.task = data ####无误差应选对象，robot待修改，ROS发布
        self.cueflg = 1
        self.change_phase('cue')

class BCIsigpro(sigpro):
    def __init__(self,c2s,s2c,kil,Config):
        super(BCIsigpro,self).__init__(c2s,s2c,kil)
        # self.bmprm = bmprm
        self.Config = Config

    def Initialize(self):
        self.configs = self.Config[0]
        self.Channellist = self.configs['Channellist']
        self.Channelssvep = self.Channellist[:-1]

        self.Trigger = {'state':0,'freqnum':0} #注册trigger
        self.eeg = np.empty(0)
        self.trigger = np.empty(0)

        # self.mudname = rootdir+'/src/mud/MUD.mat'
        # self.sigparm = sio.loadmat(self.mudname)
        # self.FilterLR = self.sigparm['FilterLR']
        offlinedata = sio.loadmat(
            '/home/mars/lyrworkspace/BCI_Multi_Robot/BCI_Multi_Robot/src/bci_background/scripts/src/python/nontarget_score_offline_1_lxb.mat')
        self.nontarget_score_offline_1 = offlinedata['nontarget_score_offline_1']

        n = 3
        MdB = 20
        bprange = np.array([6.0, 35.0])
        Ws = bprange / (self.configs['SamplingRate']/2)
        self.b, self.a = scipy.signal.iirfilter(n, Ws, rs=MdB, ftype='cheby2')  # Hd_Bandpass
        self.prodata = np.empty(0)

        an = 3
        aMdB = 20
        abprange = np.array([55.0, 77.0])
        aWs = abprange / (self.configs['SamplingRate']/2)
        self.ab, self.aa = scipy.signal.iirfilter(an, aWs, rs=aMdB, ftype='cheby2')  # Hd_Bandpass
        self.aprodata = np.empty(0)

        # self.frequency = [9,11.7,14.5]
        self.frequency = [8.18,12.85,9.98,14.99,8.97,11.23]
        self.frelen = len(self.frequency)
        self.frelen1 = 0
        t = np.arange(0.005,5.5,0.005)
        self.Y = {}
        for i in range(self.frelen):
            y = np.array([np.sin(2 * np.pi * self.frequency[i] * t), np.cos(2 * np.pi * self.frequency[i] * t),
                          np.sin(4 * np.pi * self.frequency[i] * t), np.cos(4 * np.pi * self.frequency[i] * t),
                          np.sin(6 * np.pi * self.frequency[i] * t), np.cos(6 * np.pi * self.frequency[i] * t)])
            self.Y[str(i)] = y
        self.rank_min = min(len(self.Channelssvep),self.Y[str(0)].shape[0])
        self.cca = CCA(n_components=self.rank_min)
        # self.score_threshold = 0.25  #####load .mat

    def Process(self, signal, trigger):
        #大约100ms调用一次，每次从放大器读取一次数据
        #signal就是从放大其读取的数据，channels x points
        #trigger为字典，对应了注册的Trigger,每一项是一个array  1xpoints
        #必须有返回值，返回值将会发送至BCImain
        if self.eeg.size == 0:
            self.eeg = signal
            self.trigger = trigger
        else:
            self.eeg = np.hstack((self.eeg, signal))
            for t in self.trigger:
                self.trigger[t] = np.hstack((self.trigger[t], trigger[t]))
        res = [0]

        if trigger['state'][0] == 0:
            #异步检测,检测到后返回值为2
            if self.prodata.size == 0:
                self.aprodata = signal[-1,:]
            else:
                self.aprodata = np.hstack((self.aprodata, signal[-1,:]))
            aprodata_f = scipy.signal.lfilter(self.ab, self.aa, self.aprodata.T, axis=0)  # shuzhe
            if len(aprodata_f) >= 40:
                aprodata_f = aprodata_f[-40:,:]
                if np.var(aprodata_f) > 500:    ####待验证
                    res = [2]

        elif trigger['state'][0] == 2:
            if self.prodata.size == 0:
                self.aprodata = signal[-1,:]
                self.prodata = signal[self.Channelssvep,:]
                self.frelen1 = 3
            else:
                self.aprodata = np.hstack((self.aprodata, signal[-1,:]))
                self.prodata = np.hstack((self.prodata, signal[self.Channelssvep,:]))

            aprodata_f = scipy.signal.lfilter(self.ab, self.aa, self.aprodata.T, axis=0)  # shuzhe
            if len(aprodata_f) >= 40:
                aprodata_f = aprodata_f[-40:,:]
                if np.var(aprodata_f) > 500:    ####待验证
                    res = [1]

            prodata_f = scipy.signal.lfilter(self.b, self.a, self.prodata.T, axis=0)  # shuzhe
            # score = np.zeros((1,len(self.frequency)))
            # score1 = np.zeros((1,len(self.frequency)))
            score = [0] * self.frelen1
            score1 = [0] * self.frelen1
            calculate_len = self.prodata.shape[1]
            for k in range(self.frelen1):
                y = self.Y[str(k)][:,0:calculate_len].T
                x_scores, y_scores = self.cca.fit_transform(prodata_f,y)
                r = np.corrcoef(x_scores.T, y_scores.T)
                R = np.diag(r[-self.rank_min:,:-self.rank_min])
                score1[k] = max(R)   #cca-rv
                score1[k] = (score1[k] - self.nontarget_score_offline_1[k,round(calculate_len/10)])/(
                        score1[k] + self.nontarget_score_offline_1[k,round(calculate_len/10)])
                score[k] = score1[k]   #cca-rv
            res[1:] = score1


        elif trigger['state'][0] == 3 or trigger['state'][0] == 5:
            res = [0]
            self.prodata = np.empty(0)
            self.aprodata = np.empty(0)

        elif trigger['state'][0] == 4:
            if self.prodata.size == 0:
                print('############################################')
                self.prodata = signal[self.Channelssvep,:]
                self.frelen1 = int(trigger['freqnum'][0])
            else:
                self.prodata = np.hstack((self.prodata, signal[self.Channelssvep,:]))
            prodata_f = scipy.signal.lfilter(self.b, self.a, self.prodata.T, axis=0)  # shuzhe
            score = [0] * self.frelen1
            score1 = [0] * self.frelen1
            calculate_len = self.prodata.shape[1]
            for k in range(self.frelen1):
                y = self.Y[str(k)][:,0:calculate_len].T
                x_scores, y_scores = self.cca.fit_transform(prodata_f,y)
                r = np.corrcoef(x_scores.T, y_scores.T)
                R = np.diag(r[-self.rank_min:,:-self.rank_min])
                score1[k] = max(R)   #cca-rv
                score1[k] = (score1[k] - self.nontarget_score_offline_1[k, round(calculate_len / 10)]) / (
                        score1[k] + self.nontarget_score_offline_1[k, round(calculate_len / 10)])
                score[k] = score1[k]   #cca-rv
            res = score1

        return res

def sig_process(c2s,s2c,kil,Config):
    sg = BCIsigpro(c2s,s2c,kil,Config)
    sg.StartRun()

    
if __name__ == '__main__':
    configs = {'Experiment': 'Multi robot control experiment',
               'SubjectName': 'test',
               'Session': 1,
               'Directory': rootdir+'/src/data',
               # 'Amplifier': 'actichamp',
               'Amplifier': 'simulator',
               'Channellist': range(7),
               'SamplingRate': 200,
               'SaveData': False,
               # 'SaveData': True,
               }

    mode = 1 ####

    Config = [configs,mode]
    bm = BCImain(Config)
    p = multiprocessing.Process(target=sig_process,args=(bm.c2s,bm.s2c,bm.e4s,Config))
    p.start()
    p_recommend_subscribe = multiprocessing.Process(target=bm.recommend_subscribe)#recommend from robot control
    p_recommend_subscribe.daemon = True
    p_recommend_subscribe.start()
    bm.StartRun()
