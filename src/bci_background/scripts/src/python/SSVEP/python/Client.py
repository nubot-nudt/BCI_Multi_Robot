# -*- coding:utf-8 -*-
from numpy import *
import VisionEgg
import logging, time, struct
import pygame
import threading
from Queue import Queue
from socket import *
import os
os.sys.path.append('.\\common')
from BCIFunc import create_logger
from NewPrecisionTiming import *
from HzCreate import HZ_CREATE
from AppTools.Displays import fullscreen
from AppTools.Shapes import PolygonTexture, Disc, Block
from AppTools.Boxes import box
from AppTools.StateMonitors import addstatemonitor, addphasemonitor
from VisionEgg.Textures import Texture
from VisionEgg.MoreStimuli import Arrow, FilledCircle
from PIL import Image, ImageEnhance
# from VideoCapture import Device
from pygame.locals import *
from threading import Event

class BciApplication(BciGenericApplication):
    def Description(self):
        return "Basic SSVEP"

    def Construct(self):
        self.define_param(
            "PythonApp:Screen			float			WindowSize= 		0.5  1.0  0.0  1.0 // size of the stimulus window, proportional to the screen",
            "PythonApp:Task				intlist			Mode=					3 0 1 2 // ",
            "PythonApp:Task				intlist			TargetNum=				1 20 // ",
            "PythonApp:Task				floatlist		HzList= 				7 6 7 8 9.5 10.5 11 11.5 // Hz List",
            "PythonApp:StimuliMode				intlist			Shape=				4 0 4 0 0 // flckers' Shape",
            "PythonApp:StimuliMode				floatlist		Color=				4 0 255 0 255  // flckers' color",
            "PythonApp:StimuliMode				floatlist		Size=					2 100 100  // flckers' size",
            "PythonApp:Design		int			StartDuration= 		2000  %  %  % // Duration of start cue(ms)",
            "PythonApp:Design		int			DareDuration= 		5000  %  %  % // Duration of Dare cue(ms)",
        )
        self.define_state(
            "PhaseInSequence 8 0 0 0",  # 1: pre-sequence, 2: during sequence, 3: post-sequence
            "label 8 0 0 0", 
        )

    def Preflight(self, sigprops):
        if self.params['Mode'][0] < 2:
            fullscreen(scale=float(self.params['WindowSize']))
        else:
            fullscreen(scale=0.1)

    '''
	def Frame(self,phase):
		# self.camshot = ImageEnhance.Brightness(self.cam.getImage()).enhance(self.brightness)
		self.camshot = self.cam.getImage()
		self.camshot = self.camshot.tostring()
		self.cam_screen1 = pygame.image.frombuffer(self.camshot,(self.init_cam_size[0],self.init_cam_size[1]),'RGB')
		self.stimuli['flicker'].parameters.texture = Texture(self.cam_screen1)
	'''

    def Initialize(self, indim, outdim):
        self.p_N = ['Mode', 'TargetNum', 'HzList', 'Shape', 'Color', 'Size']
        self.tasklist = range(0,3)*4
        random.shuffle(self.tasklist)
        self.run_num = int(self.params['BlocksPerRun'])
        self.Mode = array(map(float, self.params['Mode']))
        self.TargetNum = array(map(float, self.params['TargetNum']))
        self.HzList = array(map(float, self.params['HzList']))
        self.SubjectName = array(self.params['SubjectName'])
        self.SubjectSession = array(self.params['SubjectSession'])
        Task_param_0 = [self.Mode, self.TargetNum, self.HzList, self.SubjectName, self.SubjectSession]
        self.Color = array(map(float, self.params['Color']))
        self.Shape = array(map(float, self.params['Shape']))
        self.Size = array(map(float, self.params['Size']))
        Sti_param_0 = [self.Shape, self.Color, self.Size]
        # Task_param_0 = self.Task_Para_init()
        # Sti_param_0 = self.Sti_Para_init()
        Tcp_paramlist = Task_param_0 + Sti_param_0  # warning:must be transmit the param because def_TCP will pack them
        self.TcpipSend(Tcp_paramlist)
        


    def StartRun(self):
        self.states["PhaseInSequence"] = 0
        self.states["label"] = 0

    def Phases(self):
        self.phase(name='begin', next='cue', duration=3000)
        self.phase(name='cue', next='on', duration=int(self.params['StartDuration']))
        self.phase(name='on', next='pause', duration=int(self.params['DareDuration']))
        self.phase(name='pause', next='cue', duration=int(self.params['StartDuration']))
        # self.phase(name='display', 		next='Begin',		duration=int(self.params['StartDuration']))
        self.phase(name='end', duration=2000)

        if self.in_phase('on') and self.run_num <= 0:
            self.phase(name='pause', next='end', duration=int(self.params['StartDuration']))

        self.design(start='begin', new_trial='cue', interblock='idle')

    def Transition(self, phase):
        # To_Pgame = ['begin', 'Ready', 'on', 'off', 'display', 'END']
        To_Pgame = ['begin', 'cue', 'on', 'off', 'display', 'END']
        if phase == 'begin':
            self.states['PhaseInSequence'] = 0
            self.s.send('begin')
        if phase == 'cue':
            self.states['PhaseInSequence'] = 1
            self.run_num -= 1
            self.currenttask = self.tasklist.pop(0)
            #self.s.send('cue')
            self.s.send('cue'+str(self.currenttask))
            self.states['label'] = self.currenttask + 1
        if phase == 'on':
            self.states['PhaseInSequence'] = 2
            self.s.send('on')
        if phase == 'pause':
            self.states['PhaseInSequence'] = 3
            self.s.send('off')
        if phase == 'end':
            self.states['PhaseInSequence'] = 5
            self.s.send('END')
        #self.s.send(To_Pgame[self.states["PhaseInSequence"]])

    def Process(self, sig):
        if int(sig[0, 0]) != 0 and (int(sig[0, 0]) <= len(self.HzList)):
            self.s.send('target' + str(sig[0, 0]))

    def StopRun(self):
        self.states["PhaseInSequence"] = 0
        self.s.close()

    def TcpipSend(self, paramlist):
        self.s = socket(AF_INET, SOCK_STREAM)
        host = 'localhost'
        port = 40016
        self.s.connect((host, port))
        # self.s.connect(('localhost', 40016))
        # self.s.connect(('127.0.0.1', 50000))
        a = '>'
        for k in xrange(len(paramlist)):
            paramlist[k] = paramlist[k].tostring()
            a += str(len(paramlist[k])) + 's'
        h = struct.pack(a, *paramlist)
        self.s.send(a + '+' + h)
