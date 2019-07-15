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
import os
file_path = os.path.split(__file__)[0]


class Block(object):
    size = (5,5)
    position = (0,0)
    anchor = 'center'
    borderon = False
    borderwidth = 1
    bordercolor = (0,0,0)
    forecolor = (255,255,255,255)
    textcolor = (0,255,255)
    textfont = 'arial'
    textanchor = 'center'
    textsize = 10
    textbold = False
    text = ''
    layer = 0
    visible = False
            
    parmkeys = ['size','position','anchor','borderon','borderwidth','bordercolor',
                'forecolor','textcolor','textfont','textanchor','textsize','textbold',
                'text','layer','visible']
    sur = None

    def __init__(self,root,**argw):
        pygame.font.init()
        self.root = root
        self.update_parm(**argw)
        

        if not os.path.isfile(self.textfont): self.textfont = pygame.font.match_font(self.textfont)
        self.font_object = pygame.font.Font(self.textfont,self.textsize)
        self.font_object.set_bold(self.textbold)
        
        self.reset()

    def update_parm(self,**argw):
        for item in argw:   exec('self.%s = argw[item]'%(item))  

    def reset(self,**argw):
        self.update_parm(**argw)
        self.blitp = self.blitpborder = blit_pos1(self.size,self.position,self.anchor)
        self.sur = pygame.Surface(self.size)
        self.sur.fill(self.forecolor)

        if self.text != '':
            txt = self.font_object.render(self.text,1,self.textcolor)
            p0 = getcorner(self.sur.get_size(),self.textanchor)
            p = blit_pos(txt,p0,self.textanchor)
            self.sur.blit(txt,p)

        return self.sur,self.blitp

    def show(self):
        if self.visible:
            if self.sur!=None:self.root.blit(self.sur,self.blitp)
            if self.borderon:   pygame.draw.rect(self.root,self.bordercolor,pygame.Rect(self.blitpborder,self.size),self.borderwidth)

