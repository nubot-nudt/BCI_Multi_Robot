#-*- coding:utf-8 -*-
'''
from BCPY2000.PrecisionTiming
Renew in 2013.12.30
'''
__all__ = ['prectime','counter_hz', 'counter_num','SetProcessPriority', 'SetThreadPriority', 'SetProcessAffinity', 'SetThreadAffinity']

kernel32dll = None

prectime = None
counter_hz = None 
counter_num = None
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
		
	def prectime():
		return 	float(counter_num()) / (float(counter_hz())/1000.0)
		
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

if prectime == None:
	from time import time
	def prectime(): return time() * 1000.0 
	a = b = prectime()
	while (b-a) == 0: b = prectime()
	a = b
	while (b-a) == 0: b = prectime()
	if (b-a) > 0.08:
		print __name__,'module could not find a precision timer: using less-precise time.time instead'

if SetThreadAffinity == None:
	def SetThreadAffinity(aff): print "failed to set thread affinity"
if SetThreadPriority == None:
	def SetThreadPriority(p): print "failed to set thread priority"
if SetProcessAffinity == None:
	def SetProcessAffinity(aff): print "failed to set process affinity"
if SetProcessPriority == None:
	def SetProcessPriority(p): print "failed to set thread priority"
