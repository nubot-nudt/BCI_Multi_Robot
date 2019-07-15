#   $Id$
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

import pylibusb as usb # NB: the original pynia-0.0.2.py uses a module named usb for usb access, but usb is for python 2.6 only.
import ctypes

class NIA_Interface(object):
    """ Attaches the NIA device, and provides low level data collection and information
    """###
    def __init__(self,):
        self.VENDOR_ID = 0x1234 #: Vendor Id
        self.PRODUCT_ID = 0x0000   #: Product Id for the bridged usb cable
        self.TIME_OUT = 1000
        self.handle = None
        self.device = None
        found =False

        usb.init()
        if not usb.get_busses():
          usb.find_busses()
          usb.find_devices()

        buses = usb.get_busses()
        for bus in buses :
            for device in bus.devices :
                if device.descriptor.idVendor == self.VENDOR_ID and device.descriptor.idProduct == self.PRODUCT_ID:
                  found = True
                  break
            if found:
              break
        if not found:
          raise RuntimeError("Cannot find device")

        interface_nr = 0
        self.device = device
        self.config = self.device.config[0]
        self.interface = interface_nr       #self.interface = self.config.interfaces[0][0]
        self.ENDPOINT1 = 0x81               #self.interface.endpoint[0].bEndpointAddress  #self.ENDPOINT1 = self.interface.endpoints[0].address
        self.ENDPOINT2 = 0x01               #self.interface.endpoints[1].address
        self.PACKET_LENGTH = 56            #self.interface.endpoints[0].maxPacketSize

    def open(self) :
        """ Attache NIA interface
        """###
        if not self.device:
            raise RuntimeError("Cable isn't plugged in")

        self.handle = usb.open(self.device)
        if hasattr(usb,'get_driver_np'):
          # non-portable libusb extension
          name = usb.get_driver_np(self.handle,self.interface)
          if name != '':
            debug("attached to kernel driver '%s', detaching."%name )
            usb.detach_kernel_driver_np(self.handle,self.interface)  			#self.handle.detachKernelDriver(0)
        																		#self.handle.detachKernelDriver(1)
        usb.set_configuration(self.handle, self.config.bConfigurationValue)   	#self.handle.setConfiguration(self.config)
        usb.claim_interface(self.handle, self.interface)                      	#self.handle.claimInterface(self.interface)
        																		#self.handle.setAltInterface(self.interface)
        self.INPUT_BUFFER = ctypes.create_string_buffer(self.PACKET_LENGTH)

    def close(self):
        """ Release NIA interface
        """###
        usb.close(self.handle)      											#self.handle.reset()
        																		# self.handle.releaseInterface()
        self.handle, self.device = None, None

    def read(self):
        """ Read data off the NIA from its internal buffer of up to 16 samples
        """###
        usb.interrupt_read(self.handle,self.ENDPOINT1,self.INPUT_BUFFER,self.TIME_OUT);
        return self.INPUT_BUFFER


#################################################################
#################################################################

class BciSource(BciGenericSource):	

	#############################################################

	def Description(self):
		return "records from the NIA"
	
	#############################################################

	def Construct(self):
		parameters = [
			"Source:Signal%20Properties:DataIOFilter int        SourceCh=          1         1 1 % // number of digitized and stored channels",
			"Source:Signal%20Properties:DataIOFilter list       ChannelNames=      1  NIA    % % % // list of channel names",
			"Source:Signal%20Properties:DataIOFilter floatlist  SourceChOffset=    1  0      0 % %  // Offset for channels in A/D units",
			"Source:Signal%20Properties:DataIOFilter floatlist  SourceChGain=      1  1e-3   1 % %  // gain for each channel (A/D units -> muV)",
			"Source:Online%20Processing:TransmissionFilter list TransmitChList=    1  1      % % %  // list of transmitted channels",
			
			"Source:NIA%20Recording int   HardwareSamplingRate=   3900     3900       1 % // sampling rate at which the NIA natively runs",
			"Source:NIA%20Recording float HardwareChunkMsec=         2.0      2.0     0 % // milliseconds of signal to record at a time",
			"Source:NIA%20Recording float NIABufferSizeMsec=     10000    10000       0 % // size of ring buffer",
			"Source:NIA%20Recording int   DSFilterOrder=            10       10       2 % // order of pre-decimation lowpass-filter used before decimation",
			"Source:NIA%20Recording float DSFilterFreqFactor=        0.4      0.4     0 1 // lowpass cutoff of pre-decimation filter expressed as a proportion of the desired Nyquist frequency",
		]
		states = [
		]
		self._add_thread('listen', self.Listen).start()
		return (parameters, states)

	#############################################################
	
	def Initialize(self, indim, outdim):
		self.warp = 1000.0 # let the samples flowing into the ring buffer set the pace
		self.eegfs = self.samplingrate()
		self.hwfs = int(self.params['HardwareSamplingRate'])
		self.chunk = SigTools.msec2samples(float(self.params['HardwareChunkMsec']), self.hwfs)
		ringsize = SigTools.msec2samples(float(self.params['NIABufferSizeMsec']), self.hwfs)
		self.ring = SigTools.ring(ringsize, indim[0])
		self.ring.allow_overflow = True
		self.nominal['HardwareSamplesPerPacket'] = SigTools.msec2samples(self.nominal['SecondsPerPacket']*1000.0, self.hwfs)
		
		cutoff = float(self.params['DSFilterFreqFactor']) * self.eegfs / 2.0
		order = int(self.params['DSFilterOrder'])
		if order > 0 and cutoff > 0.0:
			self.filter = SigTools.causalfilter(freq_hz=cutoff, samplingfreq_hz=self.hwfs, order=order, type='lowpass') #, method=SigTools.firdesign)
		else:
			self.filter = None
		self.dsind = numpy.linspace(0.0, self.nominal['HardwareSamplesPerPacket'], self.nominal['SamplesPerPacket']+1, endpoint=True)
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

		ns = int(self.nominal['HardwareSamplesPerPacket'])
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
	
		# low-pass
		if self.filter != None: x = self.filter.apply(x, axis=1)
		# downsample
		out[:, :] = x[:, self.dsind[:-1]]  # out is a view into a slice of sig	
		return sig
		
	#############################################################
	
	def Listen(self, mythread):

		mythread.read('stop', remove=True)
		mythread.read('start', wait=True, remove=True)

		nchan = self.ring.channels()
		nsamp = int(round(self.chunk))
		
		# initialization of NIA
		self.interface = NIA_Interface()
		self.interface.open()
		
		mythread.read('stop', remove=True)
		mythread.post('ready', wait=True)
		
		while not mythread.read('stop'):
			data = [] # prepares a new list to store the read NIA data
			while len(data) < nsamp:  # was set to perform self.Points=25 reads at a time---why 25, we don't know exactly
				time.sleep(0.001)
				raw = self.interface.read()
				nread = ord(raw[54]) # byte 54 gives the number of samples
				for t in range(nread):
					val = ord(raw[t*3+2])*65536 + ord(raw[t*3+1])*256 + ord(raw[t*3])
					data.append(val)
					
			data = numpy.array([data])
			self.ring.write(data)
		
		# de-initialization of NIA
		self.interface.close()
		self.interface = None

#################################################################
#################################################################
