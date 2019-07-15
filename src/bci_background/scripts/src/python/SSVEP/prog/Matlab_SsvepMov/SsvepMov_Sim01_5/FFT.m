function [fx,Fy] = FFT(signal, L, Fs)

NFFT = 2^nextpow2(L); % Next power of 2 from length of y
Y = fft(signal,NFFT)/L;
fx = Fs/2*linspace(0,1,NFFT/2);
Fy = 2*abs(Y(1:NFFT/2,:));
% figure
% Plot single-sided amplitude spectrum.
% plot(fx,Fy) 
% title('Single-Sided Amplitude Spectrum of y(t)')
% xlabel('Frequency (Hz)')
% ylabel('|Y(f)|')




