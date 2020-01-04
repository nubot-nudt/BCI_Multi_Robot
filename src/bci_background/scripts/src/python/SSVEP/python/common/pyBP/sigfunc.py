def psd(data, fs, df_exp):
	""" psd of data """
	
	from math import ceil, log
	from numpy import fft, linspace
	
	a = ceil(log(data.shape[0])/log(2))
	b = ceil(log(fs/df_exp+2)/log(2))
	nfft = 2**int(max(a,b))
	
	Y0 = fft.fft(data, nfft, 0)
	Y = Y0[:nfft/2,:]/data.shape[0]
	Pxx = abs(Y)
    #Pangle = angle(Y(1:nfft/2,:));
	f = fs/2*linspace(0,1,nfft/2)
	
	return (Pxx, f)
	