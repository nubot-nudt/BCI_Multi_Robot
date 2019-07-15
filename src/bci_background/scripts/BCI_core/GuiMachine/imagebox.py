#!user/bin/python
# -*-coding:utf-8-*-

#FileName: block.py
#Version: 1.0
#Author: Jingsheng Tang
#Date: 2017/8/12
#Email: mrtang@nudt.edu.cn
#Github: trzp

import pygame
from pygame_anchors import *
import os,sys

class Imagebox(object):
    image = ''
    size = (0,0)
    position = (0,0)
    anchor = 'lefttop'
    text = ''
    textcolor = (0,255,255)
    textfont = 'arial'
    textsize = 10
    textanchor = 'lefttop'
    borderon = False
    borderwidth = 1
    bordercolor = (255,255,255)
    layer = 0
    visible = False
    
    parmkeys = ['image','size','position','anchor','text','textcolor','textfont','textsize',
                'textanchor','borderon','borderwidth','bordercolor','layer','visible',
                'forecolor','textcolor','textfont','textanchor','textsize','textbold',
                'text',]
    sur = None
    
    def __init__(self,root,**argw):
        pygame.font.init()
        self.root = root
        self.reset(**argw)
    
    def update_parm(self,**argw):
        for item in argw:   exec('self.%s = argw[item]'%(item))  

    def reset(self,**argw):
        self.update_parm(**argw)
        if self.size == (0,0):
            self.sur = pygame.image.load(self.image).convert()
            self.size = self.sur.get_size()
        else:
            self.sur = pygame.transform.scale(pygame.image.load(self.image).convert(),self.size)

        if self.text != '':
            self.font_object = pygame.font.Font(self.textfont,self.textsize)
            self.textsur = self.font_object.render(self.text,1,self.textcolor)
            corner = getcorner(self.siz,self.textanchor)
            p = blit_pos(self.textsur,corner,self.textanchor)
            self.sur.blit(self.textsur,p)
        self.blitp = blit_pos1(self.size,self.position,self.anchor)
        return self.sur,self.blitp

    def show(self):
        if self.visible:
            if self.sur != None:    
                self.root.blit(self.sur,self.blitp)
            if self.borderon:   pygame.draw.rect(self.root,self.bordercolor,pygame.Rect(self.blitp,self.size),self.borderwidth)
