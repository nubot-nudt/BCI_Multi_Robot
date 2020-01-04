
import pygame,sys,threading,struct
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

class imageload(threading.Thread):
	def __init__(self,frameque):
		threading.Thread.__init__(self)
		self.frameque = frameque
		
	def run(self):
		i = 0
		while True:
			i %=1010
			pygame.time.delay(50)
			im = pygame.image.load('..\\image\\'+str(i+1941)+'.jpg').convert()
			self.frameque.put([im,i])
			i += 1

class Controller(object):
	def __init__(self, switchI, FreqQue, params,Hit, *args, **argd):
		self.__dict__.update(*args, **argd)
		super(Controller,self).__init__(*args, **argd)
		p_N=['Mode','TargetNum','HzList','S_Shape','S_Color','S_Size']
		for k in xrange(len(p_N)):
			exec('self.'+p_N[k]+'=params[k]')
		self.TargetNum = int(self.TargetNum[0])
		self.FreqQue = FreqQue
		self.switchI = switchI
		self.Hit = Hit
		self.phase = 'on'
		self.frameque = Queue()
		self.screen_size = self.current_scr_size =[760,616]#[1200,700]
		self.screen = pygame.display.set_mode(self.current_scr_size,RESIZABLE,32)
		# self.screen = pygame.Surface(self.screen_size).convert_alpha()

		self.image_pos =(20,20)
		# self.color = [[255,0,0],[0,0,255],[255,125,0],[0,255,0]]
		self.color1 = [[255,255,0],[255,255,0],[255,255,0],[255,255,0],[255,255,0],[255,255,0]]
		
		self.S_Color=np.array(self.S_Color).reshape(-1,4)
		self.S_Color_C = np.copy(self.S_Color)
		self.S_Color_C[:,:-1] = 255-self.S_Color_C[:,:-1]
		self.S_Size=np.int_(np.array(self.S_Size).reshape(-1,2))
		
		self.font = pygame.font.SysFont("simsunnsimsun",40 )
		self.pednum = 6
		self.pedpos_new = []
		
		self.tracklet = sio.loadmat('..\\image\\locationmiao_1.mat')
		self.tasklist = []
		self.tasklist.extend(np.random.permutation(self.pednum))
		self.tasklist.extend(np.random.permutation(self.pednum))
		
		self.frameNo=0
		self.frame = pygame.image.load('..\\image\\'+str(1941)+'.jpg').convert_alpha()
		self.val=[2]*6
		self.ratio = [1,1]

	def overlap(self):
		
		
		
		if not self.frameque.empty():
			self.frame,self.frameNo = self.frameque.get()
		else: 
			self.frameNo = self.frameNo
			self.frame=self.frame
			
		self.frame = pygame.transform.scale(self.frame, self.current_scr_size)
		self.pedpos = self.tracklet['locationmiao_6'][self.frameNo]
		self.pedpos_new = np.int_(2*np.array(self.pedpos).reshape(-1,2)*self.ratio)
		self.pednum = 6#len(self.pedpos)/2

		self.screen.blit(self.frame,[0,0])
		
		# self.val=[2]*self.pednum
		if not self.FreqQue.empty():
			self.val = self.FreqQue.get()
		else:self.val=self.val
		
		for k in xrange(self.pednum):
			if self.phase != 'on':
				self.draw_sti(self.screen,5,self.color1[k],self.pedpos_new[k],self.S_Size[k]/4,k)
			if self.phase == 'on':
				if self.val[k]==1 :
					self.draw_sti(self.frame,self.S_Shape[k],(0,255,0,1),self.pedpos_new[k],self.S_Size[k],k)
				if self.val[k]==0 :# else:#
					self.draw_sti(self.frame,self.S_Shape[k],[0,0,0,0],self.pedpos_new[k],self.S_Size[k],k)
				# elif self.val[k]!=1 and k>=3:
					# self.draw_sti(self.screen,self.S_Shape[k],[0,0,0,5],self.pedpos_new[k],self.S_Size[k],k)
				# else:
					# pass
			self.screen.blit(self.font.render(str(k+1),True,[0,0,0]),(self.pedpos_new-[20,15])[k])
		
		pygame.display.flip()


	def event_list(self):
		event = pygame.event.poll()
		if event.type == QUIT:
			quit()
		if event.type == VIDEORESIZE:
			pygame.display.set_mode(self.current_scr_size,RESIZABLE,32)
			# self.ratio = event.size/np.array(map(float,self.current_scr_size))
			self.ratio = event.size/np.array(map(float,self.screen_size))
			self.current_scr_size = event.size
			self.screen = pygame.display.set_mode(self.current_scr_size,RESIZABLE,32)

		pygame.display.set_caption('Window resized to'+str(self.current_scr_size))

	def switch(self):
		if not self.switchI.empty():
			self.choiceP = self.switchI.get()
			
			if 'target' in self.choiceP:
				self.color1[int(float(self.choiceP[6:])-1.)] = [0,255,255,255]
				
			if self.choiceP =='on':
				self.color1 = [[255,255,0],[255,255,0],[255,255,0],[255,255,0],[255,255,0],[255,255,0]]

			if self.choiceP == 'Begin':
				self.image = imageload(self.frameque)
				self.image.start()
			elif self.choiceP == 'Ready':
				self.color1 = [[255,255,0],[255,255,0],[255,255,0],[255,255,0],[255,255,0],[255,255,0]]
				self.taskcode = self.tasklist.pop(0)
				print self.taskcode
				self.color1[self.taskcode] = [255,0,0]
			elif self.choiceP =='Resume':
				pygame.quit()
				os.system(sys.argv[0][sys.argv[0].rfind(os.sep)+1:])
				sys.exit()
				
			elif self.choiceP == 'END':
				# sys.exit()
				pygame.quit()
				sys.exit()

			self.phase = self.choiceP

	###========================================================================================###
	def draw_sti(self,screen,Shape,color,pos,size,flag):
		# 0,Block;1,Triangle;2,Pentagan;3,Arrow;4,Cross;5,Circle
		self.sti_num = int(self.pednum)
		if Shape==0:
			pygame.draw.rect(screen,color,list(pos-size[0]*.4)+list(size),0)
		if Shape==1:
			point_list = np.array([.5+.45*np.sin(0-2*np.pi/self.sti_num*flag),
							.5+.45*np.cos(0-2*np.pi/self.sti_num*flag),
							.5+.56*np.sin(-.75*np.pi-2*np.pi/self.sti_num*flag),
							.5+.56*np.cos(-.75*np.pi-2*np.pi/self.sti_num*flag),
							.5+.56*np.sin(.75*np.pi-2*np.pi/self.sti_num*flag),
							.5+.56*np.cos(.75*np.pi-2*np.pi/self.sti_num*flag)]).reshape(3,2)
			pos =pos-size/2.+point_list*size
			np.int_(pos.reshape(-1,1))
			pygame.draw.polygon(screen,color,pos,0)
		if Shape==2:
			point_list = np.array([.5+.5*np.sin(0-2*np.pi/self.sti_num*flag),
							.5+.5*np.cos(0-2*np.pi/self.sti_num*flag),
							.5+.5275*np.sin(-.4*np.pi-2*np.pi/self.sti_num*flag),
							.5+.5275*np.cos(-.4*np.pi-2*np.pi/self.sti_num*flag),
							.5+.55*np.sin(-.8*np.pi-2*np.pi/self.sti_num*flag),
							.5+.55*np.cos(-.8*np.pi-2*np.pi/self.sti_num*flag),
							.5+.55*np.sin(.8*np.pi-2*np.pi/self.sti_num*flag),
							.5+.55*np.cos(.8*np.pi-2*np.pi/self.sti_num*flag),
							.5+.5275*np.sin(.4*np.pi-2*np.pi/self.sti_num*flag),
							.5+.5275*np.cos(.4*np.pi-2*np.pi/self.sti_num*flag)]).reshape(5,2)
			pos =pos-size/2.+point_list*size
			np.int_(pos.reshape(-1,1))
			pygame.draw.polygon(screen,color,pos,0)
		if Shape==3:
			point_list = np.array([.5+.5*np.sin(0-2*np.pi/self.sti_num*flag),
							.5+.5*np.cos(0-2*np.pi/self.sti_num*flag),
							.5+np.sqrt(.1825)*np.sin(1.2120-2*np.pi/self.sti_num*flag),
							.5+np.sqrt(.1825)*np.cos(1.2120-2*np.pi/self.sti_num*flag),
							.5+np.sqrt(.085)*np.sin(1.0304-2*np.pi/self.sti_num*flag),
							.5+np.sqrt(.085)*np.cos(1.0304-2*np.pi/self.sti_num*flag),
							.5+np.sqrt(.3126)*np.sin(np.pi-.4636-2*np.pi/self.sti_num*flag),
							.5+np.sqrt(.3126)*np.cos(np.pi-.4636-2*np.pi/self.sti_num*flag),
							.5+np.sqrt(.3126)*np.sin(-np.pi+.4636-2*np.pi/self.sti_num*flag),
							.5+np.sqrt(.3126)*np.cos(-np.pi+.4636-2*np.pi/self.sti_num*flag),
							.5+np.sqrt(.085)*np.sin(-1.0304-2*np.pi/self.sti_num*flag),
							.5+np.sqrt(.085)*np.cos(-1.0304-2*np.pi/self.sti_num*flag),
							.5+np.sqrt(.1825)*np.sin(-1.2120-2*np.pi/self.sti_num*flag),
							.5+np.sqrt(.1825)*np.cos(-1.2120-2*np.pi/self.sti_num*flag)]).reshape(7,2)
			pos = pos-size/2.+point_list*size
			np.int_(pos.reshape(-1,1))
			pygame.draw.polygon(screen,color,pos,0)
		if Shape==4:
			point_list = [(4/24.,9/24.),(9/24.,9/24.),(9/24.,3/24.),(15/24.,3/24.),
							(15/24.,9/24.),(20/24.,9/24.),(20/24.,15/24.),(15/24.,15/24.),
							(15/24.,21/24.),(9/24.,21/24.),(9/24.,15/24.),(4/24.,15/24.)]
			pos =pos-size/2.+np.int_(point_list*size)
			# np.int_(pos)#.reshape(-1,1))
			pygame.draw.polygon(screen,color,pos,0)
		if Shape==5:
			pygame.draw.circle(screen,color,pos,size[0])
			
	def main(self):
		while 1:
			self.switch()
			self.event_list()
			self.overlap()

class SwitchTran(threading.Thread):

	def __init__(self,Switch_Main,Switch_Hz,Hit):
		threading.Thread.__init__(self)#,*args,**argd)
		self.Switch_Main = Switch_Main
		self.Switch_Hz = Switch_Hz
		self.Hit = Hit
		t=socket(AF_INET,SOCK_STREAM)
		t.bind(('',40016))
		t.listen(5)
		self.client,addr = t.accept()
		print ('got a connection %s'% str(addr))
	def run(self):
		while 1:
			try:
				tm=self.client.recv(4096)
				self.client.send('ok')

				if tm.find('>') >=0 and tm.find('+') >=0:
					s0=tm[tm.find('>'):tm.find('+')]
					s1=tm[tm.find('+')+1:]
					x = list(struct.unpack(s0,s1))
				else :
					x = tm
					if x=='END':
						self.client.recv(-1)
						self.client.close()
					self.Switch_Hz.put(x)
					
				if not self.Hit.empty():
					data1 = self.Hit.get()
					self.client.send(data1)
				else:
					self.client.send('0')
			except:	
				self.Switch_Main.put('Resume')
				self.client.close()

			self.Switch_Main.put(x)

def main():
	pygame.init()
	Switch_Main = Queue()
	FreqQue = Queue()
	Switch_Hz = Queue()
	Hit = Queue()


	
	s = SwitchTran(Switch_Main,Switch_Hz,Hit)
	s.start()
	###==========================================================================================###
	switchIn = Switch_Main.get()
	param_init=[np.fromstring(switchIn[k],dtype=float) for k in xrange(len(switchIn))]
	##==========================================================================================###
	t=HZ_CREATE(param_init[2],FreqQue,Switch_Hz)
	t.start()

	c=Controller(Switch_Main,FreqQue,param_init,Hit)
	c.main()

if __name__ =='__main__':
	main()
