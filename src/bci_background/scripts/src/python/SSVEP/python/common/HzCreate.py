# -*- coding:utf-8 -*-
"""
频率发生器
2013.12
Updated in 2015.7.19
"""
import sys,threading

from NewPrecisionTiming import *
from Queue import Queue
import time

###=============================================================================================###
###                               频率发生器线程类，计时芯片分频而得。                          ###
###=============================================================================================###
class HZ_CREATE(threading.Thread):

	def __init__(self,hzlist,que,switch_flag):
		#书频率而生序列：hzlist以入，que以出，而以switch_flag为令

		threading.Thread.__init__(self)
		self.hzlist = hzlist
		self.que = que
		self.switch_flag = switch_flag
		
		self.computer_fre = counter_hz()						#计时芯片之频率，源自PrecisionTiming1
		self.cnumlist = []
		self.output_on = False
		for item in self.hzlist:
			temp = int(round(self.computer_fre/float(2*item)))	#得所求频率而需之计数
			self.cnumlist.append(temp)							#尽入定值
		self.flaglist = [-1] * len(self.hzlist)
		self.pre_numlist = [counter_num()] * len(self.hzlist)
		
		SetThreadAffinity(1)											#固计时线程至CPU核心，以求其稳
		SetThreadPriority('THREAD_PRIORITY_TIME_CRITICAL')				#设置线程优先级：最高
	
	def output(self,cmd):
		if cmd != 'on':
			output_on = False
		else: output_on = True
		return output_on
		
	def run(self):
		list_change = {}

		#若读一时值，即减初值，而后取模次数，致零则变。以理可通，而实难行，因屡求零，不免疏漏。
		#故更其法，另起列表，而录其先，但数过已定，即可。
		cmd ='off'#on'#miaomiao
		while True:
			if not self.switch_flag.empty():
				cmd = self.switch_flag.get()
			else:cmd = cmd
			self.output_on=self.output(cmd)

			if self.output_on:
				cur_num = counter_num()
				for i in range(len(self.hzlist)):
					if ((cur_num - self.pre_numlist[i]< 0) and (abs(
							cur_num - self.pre_numlist[i])/float(self.cnumlist[i])< 1/1000.)
							) or (cur_num - self.pre_numlist[i]>0):
						list_change = True
					
					if list_change == True:
						list_change = False
						self.flaglist[i] = -self.flaglist[i]
						self.pre_numlist[i] = self.pre_numlist[i] + self.cnumlist[i]

				self.que.put(self.flaglist)
				
if __name__ == '__main__':
	hzlist=[6,7,8,9,10,11,12]
	que=Queue()
	switch_flag=Queue()
	t=HZ_CREATE(hzlist,que,switch_flag)
	t.start()
	switch_flag.put('on')
	print 'freq', t.computer_fre
	while 1:	print que.get()
