#!user/bin/python
# -*-coding:utf-8-*-

#FileName: block.py
#Version: 1.0
#Author: Jingsheng Tang
#Date: 2017/8/12
#Email: mrtang@nudt.edu.cn
#Github: trzp

from __future__ import division
import pygame
from pygame_anchors import *
import os,sys
import time

class Imagearray(object):
    #允许用户播放图像序列
    def __init__(self,root,**argw):
        self.imagesource = ''
        self.size = (0,0)
        self.position = (0,0)
        self.anchor = 'lefttop'
        self.borderon = False
        self.borderwidth = 1
        self.bordercolor = (255,255,255)
        self.layer = 0
        self.play = 1    #0 暂停 1 顺序播放 -1逆序播放 该值同时代表了每次跳过的贞数
        self.loops = 0   #0 无线限循环 >0 循环次数
        self.fps = 25    #帧率
        self.visible = False
        self.surarray = []
        self.parmkeys = ['image','size','position','anchor','text','textcolor','textfont','textsize',
                    'textanchor','borderon','borderwidth','bordercolor','layer','visible',
                    'forecolor','textcolor','textfont','textanchor','textsize','textbold',
                    'text','play','loops','fps']
        self.sur = None
        self.num = 0
        self.ind = 0
        self.lp = np.sign(self.loops)
    
    
        pygame.font.init()
        self.root = root
        self.update_parm(**argw)
        if self.loops <0 :  raise IOError,'error setting of loops'
        
        for r,d,files in os.walk(self.imagesource):
            for file in files:
                if self.size == (0,0):
                    sur = pygame.image.load(r+'\\'+file).convert()
                else:
                    sur = pygame.transform.scale(pygame.image.load(r+'\\'+file).convert(),self.size)

                self.surarray.append(sur)
            break

        self.size = sur.get_size()
        self.sur = self.surarray[0]
        self.blitp = blit_pos1(self.size,self.position,self.anchor)
        self.num = len(self.surarray)
        self.clk = time.clock()
    
    def update_parm(self,**argw):
        for item in argw:   exec('self.%s = argw[item]'%(item))
        if argw.has_key('loops'):
            if self.loops <0 :
                print 'warning: error setting of loops'
                self.loops = 1
            else:
                self.lp = np.sign(self.loops)

    def reset(self,**argw): #为了保障性能，不允许显示文字，不允许中途更改图像尺寸，
        self.update_parm(**argw)
        self.blitp = blit_pos1(self.size,self.position,self.anchor)
        
    def _show(self,sur):
        self.root.blit(sur,self.blitp)
        if self.borderon:   pygame.draw.rect(self.root,self.bordercolor,pygame.Rect(self.blitp,self.size),self.borderwidth)


    def show(self):
        clk = time.clock()
        if self.visible:
            if self.sur == None:
                return
            if self.loops < 0:  #循环终止
                self._show(self.sur)
                return
            
            if clk - self.clk < 1/self.fps:
                self._show(self.sur)
                return
            else:
                self.clk = clk
                self.ind += self.play
                if self.ind >= self.num:    #正向播放到头
                    self.loops -= self.lp
                    self.ind -= self.num
                if self.ind <0:
                    self.loops -= self.lp
                    self.ind += self.num
                self.sur = self.surarray[self.ind]
            self._show(self.sur)