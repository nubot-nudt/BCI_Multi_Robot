#-*- coding:utf-8 -*-
__all__ = ['prectime', 'SetProcessPriority', 'SetThreadPriority', 'SetProcessAffinity', 'SetThreadAffinity']

kernel32dll = None

prectime = None
SetProcessPriority = None
SetThreadPriority = None
SetProcessAffinity = None
SetThreadAffinity = None

import threading
class ProcessAPIError(Exception): pass
class ThreadAPIError(Exception): pass

import ctypes
try: kernel32dll = ctypes.windll.kernel32 
except: pass

process_priorities = {
	'REALTIME_PRIORITY_CLASS':         0x00000100,
	'HIGH_PRIORITY_CLASS':             0x00000080,
	'ABOVE_NORMAL_PRIORITY_CLASS':     0x00008000,
	'NORMAL_PRIORITY_CLASS':           0x00000020,
	'BELOW_NORMAL_PRIORITY_CLASS':     0x00004000,
	'IDLE_PRIORITY_CLASS':             0x00000040,
	
								+3:    0x00000100,
								+2:    0x00000080,
								+1:    0x00008000,
								 0:    0x00000020,
								-1:    0x00004000,
								-2:    0x00000040,
}
thread_priorities = {
	'THREAD_PRIORITY_TIME_CRITICAL':   15,
	'THREAD_PRIORITY_HIGHEST':          2,
	'THREAD_PRIORITY_ABOVE_NORMAL':     1,
	'THREAD_PRIORITY_NORMAL':           0,
	'THREAD_PRIORITY_BELOW_NORMAL':    -1,
	'THREAD_PRIORITY_LOWEST':          -2,
	'THREAD_PRIORITY_IDLE':           -15,

								+3:    15,
								+2:     2,
								+1:     1,
								 0:     0,
								-1:    -1,
								-2:    -2,
								-3:   -15,
}
DWORD = ctypes.c_uint32
HANDLE = ctypes.c_voidp
LARGE_INTEGER = ctypes.c_int64

if kernel32dll != None:
	(kernel32dll.QueryPerformanceCounter, kernel32dll.QueryPerformanceFrequency)

	def counter_hz():	# 获取计时芯片的计时频率
		timebase = LARGE_INTEGER()
		kernel32dll.QueryPerformanceFrequency(ctypes.addressof(timebase))
		return timebase.value

	def counter_num():	# 获取当前计时芯片的计数值
		counter  = LARGE_INTEGER()
		kernel32dll.QueryPerformanceCounter(ctypes.addressof(counter))
		return counter.value

	(kernel32dll.GetCurrentProcess, kernel32dll.SetProcessAffinityMask, kernel32dll.SetPriorityClass)
	def SetProcessPriority(p):
		p = process_priorities.get(p, p)
		if not p in process_priorities.values(): raise ProcessAPIError, 'unrecognized priority value'
		res = kernel32dll.SetPriorityClass(HANDLE(kernel32dll.GetCurrentProcess()), DWORD(p))
		if res == 0: raise ProcessAPIError, 'SetPriorityClass call failed'
	def SetProcessAffinity(aff):
		if isinstance(aff, (tuple,list)): aff = sum(map(lambda x:2**x, aff))
		res = kernel32dll.SetProcessAffinityMask(HANDLE(kernel32dll.GetCurrentProcess()), DWORD(aff))
		if res == 0: raise ProcessAPIError, 'SetProcessAffinityMask call failed'

	(kernel32dll.GetCurrentThread, kernel32dll.SetThreadAffinityMask, kernel32dll.SetThreadPriority)
	def SetThreadPriority(p):
		p = thread_priorities.get(p, p)
		if not p in thread_priorities.values(): raise ThreadAPIError, 'unrecognized priority value'
		res = kernel32dll.SetThreadPriority(HANDLE(kernel32dll.GetCurrentThread()), ctypes.c_int(p))
		if res == 0: raise ThreadAPIError, 'SetThreadPriority call failed'
	def SetThreadAffinity(aff):
		if isinstance(aff, (tuple,list)): aff = sum(map(lambda x:2**x, aff))
		res = kernel32dll.SetThreadAffinityMask(HANDLE(kernel32dll.GetCurrentThread()), DWORD(aff))
		if res == 0: raise ThreadAPIError, 'SetThreadAffinityMask call failed'

if SetThreadAffinity == None:
	def SetThreadAffinity(aff): print "failed to set thread affinity"
if SetThreadPriority == None:
	def SetThreadPriority(p): print "failed to set thread priority"
if SetProcessAffinity == None:
	def SetProcessAffinity(aff): print "failed to set process affinity"
if SetProcessPriority == None:
	def SetProcessPriority(p): print "failed to set thread priority"




class HZ_CREATE(threading.Thread):			#定义频率发生器类，为该类开辟一个独立的线程。该发生器的原理是通过对计时芯片进行分频来进行。
	def __init__(self,hzlist,que,switch):	#该类接受一个频率列表hzlist、一个用于输出频率的队列和一个用于启动\关闭频率输出的队列.que和switch应该都是长度为1的队列
		self.hzlist = hzlist				#频率列表，以列表的形式放进所需发生的频率，可以以浮点数给出，如[10.5,30.1]
		self.que = que						#用于在线程间通信，在队列中传递一个与频率列表同样长度的列表，指示计时是否到达
		self.computer_fre = counter_hz()	#获取计时芯片的频率
		self.cnumlist = []
		self.switch = switch
		self.output_on = False
		for item in self.hzlist:
			temp = int(round(self.computer_fre/float(item)))	#计算对应每个频率列表中的频率所需要进行的计数次数
			self.cnumlist.append(temp)							#该列表在cnumlist中进行保存
		threading.Thread.__init__(self)

	def run(self):
		list_change = False
		SetThreadAffinity(1)									#将计时线程绑定到一个CPU核心上，保证计时稳定
		SetThreadPriority('THREAD_PRIORITY_HIGHEST')			#设置线程优先级：最高

		self.flaglist = [-1] * len(self.hzlist)				#计时标志
		pre_numlist = [counter_num()] * len(self.hzlist)		#记录初始计时数值

		#一种简便的实现方法是每次读取一次当前计数值，与初始计数值作差，将该差值与cnumlist中的值取模，为零，说明计数达到了一个周期，进行触发操作
		#考虑到 “==0”这样的操作有风险，即可能错过这个点。我们采用了下面的方法：用一个列表分别保存初始上一次计数值，当计数超过一个周期时触发
		while True:
			if not self.switch.empty:
				cmd = self.switch.get()
				if cmd == 'on':
					self.output_on = True
				if cmd == 'off':
					self.output_on = False
			cur_num = counter_num()
			for i in range(len(self.hzlist)):
				if cur_num - pre_numlist[i] > self.cnumlist[i]:	#当前计数值与上次计数值进行比较
					self.flaglist[i] = -self.flaglist[i]
					pre_numlist[i] = cur_num
					list_change = True
			if list_change and self.output_on:
				while not self.que.empty:
					self.que.get()
				self.que.put(self.flaglist)
				list_change = False