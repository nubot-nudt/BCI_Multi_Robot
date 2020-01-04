"""
vep test using bp module
"""

import bp
from sigfunc import psd
import numpy as np
from matplotlib import pyplot
from math import log, ceil
#from time import *

# parameters
# --amplifier
nchannels = 4
labels = ['CCz','O1','O2','Fz']
interval = 100
testwave = 'sine'
testfrq = 10

# -- fft
df_exp = 0.1
fs = 5000

# -- recognition
frq_target = [testfrq, 7, 8, 12]
frq_range = 0.1

rdata = []

bp.config(chLabels = labels, nInterval = interval, waveform = testwave, nFrequency = testfrq)

hAmp = bp.open_device()
if hAmp and bp.setup(hAmp) and bp.start(hAmp,'test'):
	# get data
	for i in range(30):
		rdata.extend(bp.readdata(hAmp))
	bp.stop(hAmp)
	bp.close_device(hAmp)
	
	# reshape
	data = np.array(rdata)
	data = data.reshape(-1, nchannels+1)
	data = data[:,:nchannels]
		
	# fft
	Pxx, f = psd(data, fs, df_exp)
	P = np.prod(Pxx,1)
	
	# feature extraction
	fea1 = np.array([(P*np.logical_and(frq-frq_range<f,f<frq+frq_range)).sum() for frq in frq_target])
	fea2 = np.array([(P*np.logical_and(2*frq-frq_range<f,f<2*frq+frq_range)).sum() for frq in frq_target])
	fea = fea1 + fea2
	print 'features', fea
	


	
	
	pyplot.figure(1)	
	pyplot.subplot(3,1,1)
	pyplot.plot(f,Pxx)
	pyplot.xlim(0,20)
	
	pyplot.subplot(3,1,2)
	pyplot.plot(f,P)
	pyplot.xlim(0,20)
	pyplot.ylim(0,10000000)
	
	pyplot.subplot(3,1,3)
	#pyplot.bar(fea)
	
	
	pyplot.figure(2)
	pyplot.subplot(2,1,1)
	pyplot.plot(data[:,0])
	pyplot.subplot(2,1,2)
	pyplot.plot(f,Pxx[:,0])
	pyplot.xlabel('Frequency(Hz)')
	pyplot.ylabel('Energy')
	pyplot.xlim(0,20)
	
	pyplot.show()
	