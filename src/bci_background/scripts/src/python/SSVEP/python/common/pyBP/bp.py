"""
Manipulation of Brain Product's EEG amplifier

function list:

- config(chLabels = ChannelLabels[:32], nInterval = 40, lpFilter = 1, waveform = 'square', nFrequency = 5)
- find_amplifiers(hAmp)
- open_device()
- close_device(hAmp)
- setup(hAmp)
- start(hAmp, type = 'eeg')
- stop(hAmp)
- readdata(hAmp)

written by: Liu Yang, gloolar@gmail.com, July 2009
"""

from ctypes import windll, Structure, c_long, c_ulong, c_int, c_short, c_ushort, c_byte, c_ubyte, byref, sizeof, create_string_buffer
from win32con import GENERIC_READ, GENERIC_WRITE,OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, FILE_FLAG_WRITE_THROUGH
from winioctlcon import CTL_CODE, FILE_DEVICE_UNKNOWN, METHOD_BUFFERED, FILE_READ_DATA, FILE_WRITE_DATA
from array import array


IOCTL_BA_DRIVERVERSION 			= CTL_CODE(FILE_DEVICE_UNKNOWN, 0x80E, METHOD_BUFFERED, FILE_READ_DATA)
IOCTL_BA_AMPLIFIER_TYPE 		= CTL_CODE(FILE_DEVICE_UNKNOWN, 0x816, METHOD_BUFFERED, FILE_WRITE_DATA | FILE_READ_DATA)
IOCTL_BA_SETUP 					= CTL_CODE(FILE_DEVICE_UNKNOWN, 0x801, METHOD_BUFFERED, FILE_WRITE_DATA)
IOCTL_BA_START 					= CTL_CODE(FILE_DEVICE_UNKNOWN, 0x802, METHOD_BUFFERED, FILE_WRITE_DATA)
IOCTL_BA_STOP 					= CTL_CODE(FILE_DEVICE_UNKNOWN, 0x803, METHOD_BUFFERED, FILE_WRITE_DATA)
IOCTL_BA_ERROR_STATE 			= CTL_CODE(FILE_DEVICE_UNKNOWN, 0x809, METHOD_BUFFERED, FILE_READ_DATA)
IOCTL_BA_CALIBRATION_SETTINGS 	= CTL_CODE(FILE_DEVICE_UNKNOWN, 0x80B, METHOD_BUFFERED, FILE_WRITE_DATA)
IOCTL_BA_DIGITALINPUT_PULL_UP 	= CTL_CODE(FILE_DEVICE_UNKNOWN, 0x80C, METHOD_BUFFERED, FILE_WRITE_DATA)


dataType = {'impedance':0, 'eeg':1, 'test':2}
calibrationWaveform = {'ramp':0, 'triangle':1, 'square':2, 'sine':3}
channelLabels = "Fp1,Fp2,F3,F4,C3,C4,P3,P4,O1,O2,F7,F8,T7,T8,P7,P8,Fz,Cz,Pz,Oz,FC1,FC2,CP1,CP2,FC5,FC6,CP5,CP6,TP9,TP10,EOG,ECG,\
F1,F2,C1,C2,P1,P2,AF3,AF4,FC3,FC4,CP3,CP4,PO3,PO4,F5,F6,C5,C6,P5,P6,AF7,AF8,FT7,FT8,TP7,TP8,PO7,PO8,Fpz,AFz,CPz,POz".split(',')
label2num = dict(zip(channelLabels,range(len(channelLabels))))


class BA_SETUP(Structure):
	_fields_ = [("nChannels", c_long), 			# Number of channels
				("nChannelList", c_byte*256), 	# Channel lookup table, -1 to -8 means PolyBox channels
				("nPoints", c_long), 			# Number of points per block
				("nHoldValue", c_ushort),		# Hold value for digital input
				("n250Hertz", c_ubyte*256), 	# Low pass 250 Hz (0 = 1000Hz)
				("nResolution", c_ubyte*256), 	# ly: 0 = 0.1uV, 1 = 0.5 uV, 2 = 10uV, 3 = 152.6 uV
				("nDCCoupling", c_ubyte*256), 	# DC coupling (0 = AC)
				("nLowImpedance", c_ubyte)] 	# Low impedance i.e. 10 MOhm, (0 = > 100MOhm)
	_pack_ = 1


class BA_CALIBRATION_SETTINGS(Structure):
	_fields_ = [("nWaveForm", c_ushort), 		# 0 = ramp, 1 = triangle, 2 = square, 3 = sine wave
				("nFrequency", c_ulong)] 		# Frequency in millihertz.
	_pack_ = 2
	
	
Setup = BA_SETUP()
Setup.nChannels = 32
Setup.nChannelList[:Setup.nChannels] = range(Setup.nChannels)
Setup.nPoints = 40*5
Setup.nHoldValue = 0x0
Setup.n250Hertz[:Setup.nChannels] = [1]*Setup.nChannels
Setup.nResolution[:Setup.nChannels] = [0]*Setup.nChannels
Setup.nDCCoupling[:Setup.nChannels] = [0]*Setup.nChannels
Setup.nLowImpedance = 0

Calibration = BA_CALIBRATION_SETTINGS()
Calibration.nWaveForm = 2
Calibration.nFrequency = 5000

transfersize = (Setup.nChannels+1)*Setup.nPoints*sizeof(c_short)
buf = create_string_buffer(transfersize)


def config(chLabels = channelLabels[:32], nInterval = 40, lpFilter = 1, waveform = 'square', nFrequency = 5):
	"""
	configure amplifier's parameters
	
	Input:
	chLabels: list of labels of USED channel. ref: channelLabels
	nInterval: time interval(ms) of data block (acquired one time)
	lpFilter: filter or not. 1: 250Hz; 0: no filter
	waveform: waveform of calibration signal. ref: calibrationWaveform
	nFrequency: frequency(Hz) of calibration signal
	"""
	
	global Setup, Calibration, buf, transfersize

	if not 1 <= len(chLabels) <= 256:
		print 'BrainAmp configuration error: Invalid number of channels, set to default value: 32'
		Setup.nChannels = 32
	else:	
		Setup.nChannels = len(chLabels)
		
	try:
		Setup.nChannelList[:Setup.nChannels] = [label2num[a] for a in chLabels]
	except Exception:
		print 'BrainAmp configuration error: Invalid channel labels, set to default value: ', channelLabels[:Setup.nChannels]
		Setup.nChannelList[:Setup.nChannels] = range(Setup.nChannels)
		
	Setup.nPoints = nInterval*5 # sampling rate: 5000Hz
	Setup.nHoldValue = 0x0
	Setup.n250Hertz[:Setup.nChannels] = [lpFilter]*Setup.nChannels
	Setup.nResolution[:Setup.nChannels] = [0]*Setup.nChannels
	Setup.nDCCoupling[:Setup.nChannels] = [0]*Setup.nChannels
	Setup.nLowImpedance = 0
	
	Calibration.nWaveForm = calibrationWaveform[waveform.lower()]
	Calibration.nFrequency = nFrequency*1000
	
	transfersize = (Setup.nChannels+1)*Setup.nPoints*sizeof(c_short)
	buf = create_string_buffer(transfersize)

	
def find_amplifiers(hAmp):
	""" get number of amplifiers connected """
	
	if not hAmp:
		print 'Invalid handle.'
		return 0
		
	amps = create_string_buffer(4*sizeof(c_ushort))
	dwBytesReturned = c_long()
	windll.kernel32.DeviceIoControl(hAmp, IOCTL_BA_AMPLIFIER_TYPE, None, 0, byref(amps), sizeof(amps), byref(dwBytesReturned), None)
	
	v = array('h')
	v.fromstring(amps)
	try:
		nAmps = v.index(0)
	except ValueError:
		nAmps = 4
	
	return nAmps
	
	
def open_device():
	""" open amplifier """
	
	DEVICE_USB = "\\\\.\\BrainAmpUSB1"

	hAmp =  windll.kernel32.CreateFileA(DEVICE_USB, GENERIC_READ|GENERIC_WRITE, 0, None, 
					OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL|FILE_FLAG_WRITE_THROUGH, None)
							
	if hAmp:
		DriverVersion = c_int()
		dwBytesReturned = c_long()
		windll.kernel32.DeviceIoControl(hAmp, IOCTL_BA_DRIVERVERSION, None, 
				0, byref(DriverVersion), sizeof(DriverVersion), byref(dwBytesReturned), None);
		nModule = DriverVersion.value % 10000;
		nMinor = (DriverVersion.value % 1000000) / 10000;
		nMajor = DriverVersion.value / 1000000;
		print "%s Driver Found, Version %u.%02u.%04u" % ("USB", nMajor, nMinor, nModule)
		print 'BrainAmp opened.'
		nAmps = find_amplifiers(hAmp)
		if nAmps == 0:
			print 'error: No amplifier connected.'
			close_device(hAmp)
			return False
			
		print '%d amplifier connected.' % nAmps
			
	return hAmp

	
def close_device(hAmp):
	""" close amplifier """
	
	if hAmp:
		windll.kernel32.CloseHandle(hAmp)
		print 'BrainAmp closed.'

		
def setup(hAmp):
	""" setup amplifier	"""
	
	if not hAmp:
		print 'BrainAmp setup failed. Invalid handle.'
		return False
	
	nRequiredAmps = max(Setup.nChannelList)/32 + 1
	nAmps = find_amplifiers(hAmp)
	if nAmps < nRequiredAmps:
		print 'BrainAmp setup failed. Required Amplifiers: %d, Connected Amplifiers: %d' % (nRequiredAmps, nAmps)
		return False
	
	dwBytesReturned = c_long()
	if not windll.kernel32.DeviceIoControl(hAmp, IOCTL_BA_SETUP, byref(Setup), sizeof(Setup), None, 0, byref(dwBytesReturned), None):
		print 'BrainAmp setup failed, error code: %u' % windll.kernel32.GetLastError()
		return False
	
	print 'BrainAmp setuped.'
	return True

	
def start(hAmp, type = 'eeg'):
	""" start acquisition """
	
	if not hAmp:
		print 'Invalid handle.'
		return False
	
	dwBytesReturned = c_long()
	
	if type.lower() == 'test':
		if not windll.kernel32.DeviceIoControl(hAmp, IOCTL_BA_CALIBRATION_SETTINGS, byref(Calibration), sizeof(Calibration), None, 0, byref(dwBytesReturned), None):
			print "BrainAmp start failed. Can't set calibration, error code: %u" % windll.kernel32.GetLastError()
			return False
			
	elif type.lower() == 'eeg':
		pullup = c_ushort()
		if not windll.kernel32.DeviceIoControl(hAmp, IOCTL_BA_DIGITALINPUT_PULL_UP, byref(pullup), sizeof(pullup), None, 0, byref(dwBytesReturned), None):
			print "BrainAmp start failed. Can't set pull up/down resistors, error code: %u" % GetLastError()
			return False
			
	elif type.lower() == 'impedance':
		print 'BrainAmp start failed, %s is not supported yet.' % type
		return False

	else:
		print 'BrainAmp start failed, no supported type: %s.' % type
		return False
	
	acquisitionType = c_long(dataType[type.lower()])
	if not windll.kernel32.DeviceIoControl(hAmp, IOCTL_BA_START, byref(acquisitionType), sizeof(acquisitionType), None, 0, byref(dwBytesReturned), None):
		print 'BrainAmp start failed, error code: %u' % windll.kernel32.GetLastError()
		return False
		
	print 'BrainAmp started.'
	return True
	

def stop(hAmp):
	""" stop acquisition """
	
	if not hAmp:
		print 'Invalid handle.'
		return False
	
	dwBytesReturned = c_long()
	if not windll.kernel32.DeviceIoControl(hAmp, IOCTL_BA_STOP, None, 0, None, 0, byref(dwBytesReturned), None):
		print 'BrainAmp stop failed, error code: %u' % windll.kernel32.GetLastError()
		return False
		
	print 'BrainAmp stopped.'
	return True

	
def readdata(hAmp):
	""" read one data block from amplifier """
	
	# Check for error
	nTemp = c_int()
	dwBytesReturned = c_long()
	if not windll.kernel32.DeviceIoControl(hAmp, IOCTL_BA_ERROR_STATE, None, 0, byref(nTemp), sizeof(nTemp), byref(dwBytesReturned), None):
		print 'Acquisition Error, error code: %u' % windll.kernel32.GetLastError()
		return False
	if nTemp.value != 0:
		print 'Acquisition Error %d' % nTemp
		return False
		
	# read data
	dwBytesReturned = c_long(0)
	while dwBytesReturned.value == 0:
		if not windll.kernel32.ReadFile(hAmp, byref(buf), transfersize, byref(dwBytesReturned), None):
			print 'Acquisition Error, error code: %u' % windll.kernel32.GetLastError()
		if dwBytesReturned.value == 0:
			windll.kernel32.Sleep(1)
		#	print 'sleep'

	v = array('h')
	v.fromstring(buf)
	return v.tolist()

	
def main():

	config() # ues default configuration

	hAmp = open_device() # open amp device

	if hAmp and setup(hAmp) and start(hAmp,'test'): # setup amp and start acquisition
		data = []
		for i in range(25): # 1 sec
			data.extend(readdata(hAmp)) # read and gather data
		print 'data read.'
		stop(hAmp) # stop acquisition
		close_device(hAmp) # close amp device

		
if __name__ == "__main__":
	main()
	
		