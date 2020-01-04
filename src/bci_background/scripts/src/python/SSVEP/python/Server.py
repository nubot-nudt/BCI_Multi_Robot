#-*- coding:utf-8 -*-
import pygame, sys, threading, struct
from pygame.locals import *
from pygame.time import *
from Queue import Queue
from socket import *
import numpy as np
import scipy.io as sio
import os
os.sys.path.append('.\\common')
from BCIFunc import create_logger
from NewPrecisionTiming import *
from HzCreate import HZ_CREATE
import logging


class Controller(object):
    def __init__(self, switchI, FreqQue, params, *args, **argd):
        self.__dict__.update(*args, **argd)
        super(Controller, self).__init__(*args, **argd)
        p_N = ['Mode', 'TargetNum', 'HzList', 'SubjectName', 'SubjectSession', 'Shape', 'Color', 'Size']
        for k in xrange(len(p_N)):
            exec ('self.' + p_N[k] + '=params[k]')
        self.FreqQue = FreqQue
        self.switchI = switchI
        self.phase = ''
        self.screen_size = self.current_scr_size = [1400,800]
        self.screen = pygame.display.set_mode(self.current_scr_size, RESIZABLE, 32)
        self.color1 = [[255, 255, 0], [255, 255, 0], [255, 255, 0]]
        # self.Size = np.int_(np.array(self.Size))
        self.Size = np.array([120,120])
        self.sti = []
        # self.sti0 = pygame.image.load('..\\image\\select_focus.png').convert_alpha()
        self.sti0 = pygame.image.load('..\\image\\select_balance.png').convert_alpha()
        self.sti0 = pygame.transform.scale(self.sti0, self.Size)
        self.sti.append(self.sti0)
        # self.sti1 = pygame.image.load('..\\image\\select_man2man.png').convert_alpha()
        self.sti1 = pygame.image.load('..\\image\\select_conservative.png').convert_alpha()
        self.sti1 = pygame.transform.scale(self.sti1, self.Size)
        self.sti.append(self.sti1)
        # self.sti2 = pygame.image.load('..\\image\\select_regional.png').convert_alpha()
        self.sti2 = pygame.image.load('..\\image\\select_radical.png').convert_alpha()
        self.sti2 = pygame.transform.scale(self.sti2, self.Size)
        self.sti.append(self.sti2)

        # self.fliker = self.sti1
        # self.fliker = pygame.Surface(self.Size).convert_alpha()
        # self.fliker.fill((0, 0, 0, 255))
        self.fliker1 = pygame.Surface(self.Size).convert_alpha()
        self.fliker1.fill((255, 255, 255, 255))
        # self.font = pygame.font.SysFont("simsunnsimsun", 40)
        self.pednum = 3
        self.pedpos_new = np.int_([[200,350],[600,350],[1000,350]])#################
        # self.tasklist = []
        # self.tasklist.extend(np.random.permutation(self.pednum))
        # self.tasklist.extend(np.random.permutation(self.pednum))
        # self.tasklist.extend(np.random.permutation(self.pednum))
        # self.tasklist.extend(np.random.permutation(self.pednum))
        self.val = [2] * self.pednum
        self.ratio = [1, 1]
        datapath = '..\\data\\' + self.SubjectName + self.SubjectSession + '\\'
        Run = int(len(os.listdir(datapath)) / 2) + 1
        self.filename = datapath + self.SubjectName + 'S' + self.SubjectSession + 'R' + str(Run) + '.log'
        logging.basicConfig(level=logging.INFO,
                            format='%(asctime)s       %(message)s',
                            filename=self.filename,
                            filemode='w')
        self.correct = 0
        self.incorrect = 0
        self.taskno = 0

    def overlap(self):
        if not self.FreqQue.empty():
            self.val = self.FreqQue.get()
        else:
            self.val = self.val
        if self.phase == 'on':
            for k in xrange(self.pednum):
            # if self.phase == 'cue':
            #     self.screen.blit(self.sti[k], self.pedpos_new[k])
            #     pygame.draw.rect(self.screen, self.color1[k], (list(self.pedpos_new[k])+[120,120]), 0)
            # if self.phase != 'on':
            #     pygame.draw.rect(self.screen, self.color1[k], (list(self.pedpos_new[k])+[120,120]), 0)
            # if self.phase == 'on':
                if self.val[k] == 1:
                    # self.screen.blit(self.fliker, self.pedpos_new[k])
                    self.screen.blit(self.sti[k], self.pedpos_new[k])
                else:
                    self.screen.blit(self.fliker1, self.pedpos_new[k])
            # self.screen.blit(self.font.render(str(k + 1), True, [0, 0, 0]), (self.pedpos_new + [40, 40])[k])
        pygame.display.update()

    def event_list(self):
        event = pygame.event.poll()
        if event.type == QUIT:
            quit()
        if event.type == VIDEORESIZE:
            pygame.display.set_mode(self.current_scr_size, RESIZABLE, 32)
            self.ratio = event.size / np.array(map(float, self.screen_size))
            self.current_scr_size = event.size
            self.screen = pygame.display.set_mode(self.current_scr_size, RESIZABLE, 32)
        pygame.display.set_caption('Window resized to' + str(self.current_scr_size))

    def switch(self):
        if not self.switchI.empty():
            self.choiceP = self.switchI.get()
            print('self.choiceP',self.choiceP)
            if 'target' in self.choiceP:

                n = int(float(self.choiceP[6:]) - 1)
                # self.color1[n] = [0, 255, 255, 255]
                pygame.draw.rect(self.screen, [0,255,0], (list(self.pedpos_new[n])+[120,120]), 0)
                #pygame.display.flip()
                #pygame.display.update()
                logging.info('result task is %s' % (n + 1))
                if self.taskcode == n:
                    self.correct += 1
                    logging.info('Truenum %s' % (self.correct))
                    logging.info('Falsenum %s' % (self.incorrect))
                else:
                    self.incorrect += 1
                    logging.info('Truenum %s' % (self.correct))
                    logging.info('Falsenum %s' % (self.incorrect))
                logging.info('')

            if self.choiceP == 'begin':
                self.screen.fill((255,255,255,0))
                for k in xrange(self.pednum):
                    self.screen.blit(self.sti[k], self.pedpos_new[k])
                # self.screen.blit(self.screen1,[0,0])
                #pygame.display.flip()
                #pygame.display.update()
            # elif self.choiceP == 'cue':
                # self.taskcode = self.tasklist.pop(0)
            elif 'cue' in self.choiceP:
                self.taskcode = int(self.choiceP[3:])
                for k in xrange(self.pednum):
                    pygame.draw.rect(self.screen, [255,255,255], (list(self.pedpos_new[k])+[120,120]), 0)
                    self.screen.blit(self.sti[k], self.pedpos_new[k])
                pygame.draw.rect(self.screen, [255,0,0], (list(self.pedpos_new[self.taskcode])+[120,120]), 0)
                #pygame.display.flip()

                print('cue:',self.taskcode)
                # self.color1 = [[255, 255, 0], [255, 255, 0], [255, 255, 0]]
                 
                self.taskno += 1
                logging.info('task No. %s' % (self.taskno))
                logging.info('current task is %s' % (self.taskcode + 1))
                # self.color1[self.taskcode] = [255, 0, 0]
            elif self.choiceP == 'on':
                self.color1 = [[255, 255, 0], [255, 255, 0], [255, 255, 0]]
            elif self.choiceP == 'off':
                for k in xrange(self.pednum):
                    self.screen.blit(self.sti[k], self.pedpos_new[k])
                #pygame.display.flip()
                #pygame.display.update()

            elif self.choiceP == 'Resume':
                pygame.quit()
                os.system(sys.argv[0][sys.argv[0].rfind(os.sep) + 1:])
                sys.exit()
            elif self.choiceP == 'END':
                pygame.quit()
                sys.exit()
            self.phase = self.choiceP

    def main(self):
        while 1:
            self.switch()
            self.event_list()
            self.overlap()

class SwitchTran(threading.Thread):
    def __init__(self, Switch_Main, Switch_Hz):
        threading.Thread.__init__(self)  # ,*args,**argd)
        self.Switch_Main = Switch_Main
        self.Switch_Hz = Switch_Hz
        t = socket(AF_INET, SOCK_STREAM)

        t.bind(('', 40016))
        t.listen(5)
        self.client, addr = t.accept()
        print ('got a connection %s' % str(addr))

    def run(self):
        while 1:
            try:
                tm = self.client.recv(4096)
                self.client.send('ok')
                if tm.find('>') >= 0 and tm.find('+') >= 0:
                    s0 = tm[tm.find('>'):tm.find('+')]
                    s1 = tm[tm.find('+') + 1:]
                    x = list(struct.unpack(s0, s1))
                else:
                    x = tm
                    if x == 'END':
                        self.client.recv(-1)
                        self.client.close()
                    self.Switch_Hz.put(x)
            except:
                self.Switch_Main.put('Resume')
                self.client.close()
            self.Switch_Main.put(x)

def main():
    pygame.init()
    Switch_Main = Queue()
    FreqQue = Queue()
    Switch_Hz = Queue()
    s = SwitchTran(Switch_Main, Switch_Hz)
    s.start()
    switchIn = Switch_Main.get()
    param_init = []
    param_init[0:2] = [np.fromstring(switchIn[k], dtype=float) for k in xrange(3)]
    param_init[3:4] = [switchIn[k + 3] for k in xrange(2)]
    param_init[5:7] = [np.fromstring(switchIn[k + 5], dtype=float) for k in xrange(3)]
    t = HZ_CREATE(param_init[2], FreqQue, Switch_Hz)
    t.start()
    c = Controller(Switch_Main, FreqQue, param_init)
    c.main()

if __name__ == '__main__':
    main()
