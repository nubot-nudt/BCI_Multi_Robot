#   $Id: AudioSourceModule.py 2898 2010-07-08 19:09:30Z jhill $
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
import time
import numpy
import SigTools
import WavTools
		
#################################################################
#################################################################

class BciSource(BciGenericSource):	

	#############################################################

	def Description(self):
		return "records audio from the computer's sound card and uses it as a signal source"
	
	#############################################################

	def Construct(self):
		parameters = [
			#"Source:Signal%20Properties:DataIOFilter int        SampleBlockSize=  20        20 1 % // the number of samples transmitted at a time",
			#"Source:Signal%20Properties:DataIOFilter int        SamplingRate=    441Hz     441 1 % // the sample rate",
			"Source:Signal%20Properties:DataIOFilter int        SourceCh=          2         2 1 % // number of digitized and stored channels",
			"Source:Signal%20Properties:DataIOFilter list       ChannelNames=      2  L R    % % % // list of channel names",
			"Source:Signal%20Properties:DataIOFilter floatlist  SourceChOffset=    2  0 0    0 % %  // Offset for channels in A/D units",
			"Source:Signal%20Properties:DataIOFilter floatlist  SourceChGain=      2  1e-3 1e-3    1 % %  // gain for each channel (A/D units -> muV)",
			"Source:Online%20Processing:TransmissionFilter list TransmitChList=    2  1 2    % % %  // list of transmitted channels",
			
			"Source:Audio%20Recording int   UseAudioEnvelope=          1        1       0 1 // check to clean up the signal by matching its envelope rather than just subsampling (boolean)",
			"Source:Audio%20Recording int   NumberOfAudioChannels=     2        2       1 % // number of audio channels to record",
			"Source:Audio%20Recording int   AudioSamplingRate=     44100    44100   11025 % // sampling rate at which to record sound",
			"Source:Audio%20Recording int   AudioBitDepth=            16       16       8 % // bit depth at which to record sound",
			"Source:Audio%20Recording float AudioChunkMsec=            0.3      0.2     0 % // milliseconds of audio to record at a time",
			"Source:Audio%20Recording float AudioBufferSizeMsec=   10000    10000       0 % // size of audio ring buffer",
		]
		states = [
		]
		self._add_thread('listen', self.Listen).start()
		return (parameters, states)

	#############################################################

	def Initialize(self, indim, outdim):
		self.warp = 1000.0 # let the audio samples flowing into the ring buffer set the pace
		self.eegfs = self.samplingrate()
		self.audiofs = int(self.params['AudioSamplingRate'])
		self.audiobits = int(self.params['AudioBitDepth'])
		self.audiochunk = WavTools.msec2samples(float(self.params['AudioChunkMsec']), self.audiofs)
		self.audiochannels = int(self.params['NumberOfAudioChannels'])
		self.use_env = int(self.params['UseAudioEnvelope'])
		ringsize = WavTools.msec2samples(float(self.params['AudioBufferSizeMsec']), self.audiofs)
		self.ring = SigTools.ring(ringsize, self.audiochannels)
		self.ring.allow_overflow = True
		self.nominal['AudioSamplesPerPacket'] = WavTools.msec2samples(self.nominal['SecondsPerPacket']*1000.0, self.audiofs)
		self.dsind = numpy.linspace(0.0, self.nominal['AudioSamplesPerPacket'], self.nominal['SamplesPerPacket']+1, endpoint=True)
		self.dsind = numpy.round(self.dsind).astype(numpy.int).tolist()
		self._threads['listen'].post('start')
		self._threads['listen'].read('ready', wait=True, remove=True)
		self._check_threads()
		
	#############################################################
	def Halt(self):
		self._threads['listen'].post('stop')
		self._check_threads()
	
	#############################################################

	def Process(self, sig):
		ns = int(self.nominal['AudioSamplesPerPacket'])
		while self.ring.to_read() < ns:
			time.sleep(0.001)
			if self._check_threads(): break
		x = self.ring.read(ns)
		
		nch = min([x.shape[0], sig.shape[0]])		
		x = numpy.asarray(x[:nch, :])
		out = numpy.asarray(sig[:nch, :])
		#sig[nch:, :] = 0
		packetsize = int(self.nominal['SamplesPerPacket'])
		sig[nch:, :] = self.packet_count * packetsize + numpy.array(range(packetsize), ndmin=2, dtype='float').repeat(sig.shape[0]-nch, axis=0)

		if self.use_env:
			for i in xrange(sig.shape[1]):
				out[:, i] = numpy.sign((i%2) - 0.5) * x[:, self.dsind[i]:self.dsind[i+1]].ptp(axis=1)
		else:
			out[:, :] = x[:, self.dsind[:-1]]
		
		return sig
		
	#############################################################
	
	def Listen(self, mythread):

		mythread.read('stop', remove=True)
		mythread.read('start', wait=True, remove=True)

		nchan = self.audiochannels
		nsamp = int(round(self.audiochunk))
		
		w = WavTools.wav(fs=self.audiofs, bits=self.audiobits, nchan=nchan)
		interface = WavTools.PyAudioInterface.grab_interface()
		format = interface.get_format_from_width(w.nbytes)	
		recorder = interface.open(format=format, channels=nchan, rate=self.audiofs, input=True)
		
		mythread.read('stop', remove=True)
		mythread.post('ready', wait=True)
		
		while not mythread.read('stop'):
			#while self.ring.to_write() < nsamp: pass
			strdat = recorder.read(nsamp)
			w.y = w.str2dat(strdat, nsamp, nchan)
			self.ring.write(w.y.T * w.fac) # multiply by w.fac to undo str2dat's normalization to +/-1
			                               # (let the AudioBitDepth and SourceChGain parameters sort it out)
		
		recorder.close()
		WavTools.PyAudioInterface.release_interface(interface)

#################################################################
#################################################################
