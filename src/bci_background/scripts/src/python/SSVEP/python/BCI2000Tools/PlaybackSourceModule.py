#   $Id: PlaybackSourceModule.py 2898 2010-07-08 19:09:30Z jhill $
#   
#   This file is part of the BCPy2000 framework, a Python framework for
#   implementing modules that run on top of the BCI2000 <http://bci2000.org/>
#   platform, for the purpose of realtime biosignal processing.
# 
#   Copyright (C) 2007-10  Jeremy Hill, Thomas Schreiner,
#                          Christian Puzicha, Jason Farquhar
#   
#   bcpy2000@bci2000.org
#   
#   The BCPy2000 framework is free software: you can redistribute it
#   and/or modify it under the terms of the GNU General Public License
#   as published by the Free Software Foundation, either version 3 of
#   the License, or (at your option) any later version.
#
#   This program is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#   GNU General Public License for more details.
#
#   You should have received a copy of the GNU General Public License
#   along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
import os,sys,re
import numpy
from BCI2000Tools.FileReader import bcistream

class PlaybackError(EndUserError): pass

#################################################################
#################################################################

class BciSource(BciGenericSource):

	#############################################################

	def Description(self):
		return 'plays back a pre-recorded .dat file in "slave" mode'
	
	#############################################################

	def Construct(self):
		parameters = [
			"Source:Playback string PlaybackFileName= % % % % // play back the named BCI2000 file (inputfile)",
			"Source:Playback string PlaybackStart= 00:00:00.000 % % % // offset at which to start",
			"Source:Playback matrix TestSignals= 0 { Channel Frequency Amplitude } % % % // sinusoidal signals may be added to the source signal here",
		]
		states = [
			"SignalStopRun 1 0 0 0",
		]
				
		return (parameters, states)
		
	#############################################################
	def Preflight(self, inprop):
		fn = self.params['PlaybackFileName']
		
		# start with environmental variables but add a couple of extra possible expansions:
		# (1) %DATA% or $DATA maps to the data subdir of the installation directory
		dataroot = os.path.realpath(os.path.join(self.data_dir, '..'))
		mappings = dict(os.environ.items() + [('DATA',dataroot)])
		sub = lambda x: mappings.get(x.group(1), x.group())
		if sys.platform.lower().startswith('win'):
			# (2) make %HOME% or $HOME equivalent to %USERPROFILE% or %HOMEDRIVE%%HOMEPATH%  
			if not 'HOME' in mappings: mappings['HOME'] = mappings.get('USERPROFILE', mappings.get('HOMEDRIVE') + mappings.get('HOMEPATH'))
			# expand Windoze-style environmental variables on Windoze
			fn = re.sub('%(.+?)%', sub, fn)
		# expand POSIX-style environmental variables on all platforms
		fn = re.sub(r'\$\{(.+?)\}', sub, fn)
		fn = re.sub(r'\$([A-Za-z0-9_]+)', sub, fn)

		try: self.stream = bcistream(fn)
		except Exception, e: raise PlaybackError(str(e))
		self.blocksize = int(self.params['SampleBlockSize'])
		self.master = int(self.params['EnslavePython']) != 0
		nch = int(self.params['SourceCh'])
		pbnch = self.stream.channels()
		if nch != pbnch:
			raise PlaybackError, 'mismatch between number of channels in SourceCh parameter (%d) and playback file (%d)' % (nch,pbnch)
		fs = self.samplingrate()
		pbfs = self.stream.samplingrate()
		if fs != pbfs:
			raise PlaybackError, 'mismatch between sampling rate in SamplingRate parameter (%gHz) and playback file (%gHz)' % (fs,pbfs)
		
		#self.out_signal_props['Type'] = self.stream.headline.get('DataFormat', 'float32')
		self.out_signal_props['Type'] = 'float32' # Hmm
		# default data format is actually int16, but float32 is safe to cast the other formats into 

		self.testsig = []
		ts = self.params['TestSignals']
		if isinstance(ts, list):
			if ts.matrixlabels()[1] != ['Channel', 'Frequency', 'Amplitude']: raise EndUserError, "TestSignals matrix must have 3 columns labelled Channel, Frequency and Amplitude"
			for x in ts:
				if x == ['','','']: continue
				chan,freq,amp = x['Channel'], x['Frequency'], x['Amplitude']
				try: chan = float(chan)
				except:
					if chan in self.params['ChannelNames']: chan = self.params['ChannelNames'].index(chan)
					else: raise EndUserError, "unrecognized Channel name '%s' in TestSignals matrix" % chan
				else:
					if chan < 1 or chan > nch or chan != round(chan): raise EndUserError, "invalid Channel index %d in TestSignals matrix" % chan
					chan = int(chan+0.5) - 1
				try: freq = float(freq)
				except: raise EndUserError, "invalid Frequency value '%s' in TestSignals matrix" % freq
				try: amp = float(amp)
				except: raise EndUserError, "invalid Amplitude value '%s' in TestSignals matrix" % amp
				self.testsig.append([chan,freq,amp])
					
		
	#############################################################

	def StartRun(self):
		self.stream.seek(self.params['PlaybackStart'])
		self.states['SignalStopRun'] = 0
		print "\nplaying back:"
		print self.stream
		
	#############################################################

	def Process(self, sig):
		if int(self.states['Running']) == 0:
			return sig * 0
		
		if self.states['SignalStopRun']:
			self.states['Running'] = 0
			self.states['SignalStopRun'] = 0
		
		newsig,states = self.stream.decode(self.blocksize, apply_gains=False)
		
		for chan,freq,amp in self.testsig:
			amp /= float(self.params['SourceChGain'][chan])
			newsig[chan,:] += amp * numpy.sin(2.0 * numpy.pi * freq * sig[0, :newsig.shape[1]])
		
		if newsig.shape[1] < self.blocksize:
			self.states['SignalStopRun'] = 1
			return sig * 0
		if self.stream.tell() >= self.stream.samples():
			self.states['SignalStopRun'] = 1
		
		if self.master:
			for k in self.states.keys():
				if not k in ('Running', 'Recording', 'AppStartTime', 'StimulusTime', 'SourceTime') and states.has_key(k):
					self.states[k] = int(numpy.asarray(states[k]).flat[-1])
		
		return newsig
		
#################################################################
#################################################################
