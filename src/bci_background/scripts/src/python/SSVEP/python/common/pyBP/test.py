"""
vep test using bp module
"""

import bp
from sigfunc import psd
import numpy as np
import pylab
from math import log, ceil
#from time import *

# parameters
# --amplifier
nchannels = 4
labels = ['P3','P4','Cz','Pz']
interval = 100
testwave = 'sine'
testfrq = 10


fs = 5000

rdata = []
dt = 3 # second
T = np.arange(0,dt,1./fs)

bp.config(chLabels = labels, nInterval = interval, waveform = testwave, nFrequency = testfrq)

hAmp = bp.open_device()

if hAmp and bp.setup(hAmp) and bp.start(hAmp,'test'):
	# get data
#	for i in range(int(dt*1000/40)):
	for i in range(1):
		print i
		rdata.extend(bp.readdata(hAmp))
	bp.stop(hAmp)
	bp.close_device(hAmp)
	
	# reshape
	data = np.array(rdata)
	data = data.reshape(-1, nchannels+1)
	data = data[:,:nchannels]*0.1
	print data.shape
	
	pylab.figure(1)
#	pylab.plot(T,data)
	pylab.plot(data)
	pylab.xlabel('time (ms)')
	pylab.ylabel('voltage ($\mu$V)')
	pylab.grid(True)
	pylab.savefig('signal_plot')

	pylab.show()
	