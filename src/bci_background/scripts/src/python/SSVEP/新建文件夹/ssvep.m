[datfiles, datdir] = ...
    uigetfile('*.dat','Select the BCI2000 P300 (.dat) training data file(s)','multiselect','on','..\data\');
if datdir == 0, return; end
if ~iscell(datfiles)
    datfiles = {datfiles};
end
datfiles = sort(datfiles);

[signal, state, parms] = getInfo(datfiles, datdir);
channel_num = length(parms.Channel);
samplingrate = parms.SamplingRate;
frequency = parms.Frequency;
phaseinsequence = state.PhaseInSequence;

BP_4_35_n96 = fir1(96,[7 35].*2./200);
% signal_filtered = filter(BP_4_35_n96,1,signal(:,parms.Channel));
% signal_filtered = signal(:,parms.Channel);
ind_begin = find(phaseinsequence(1:end-1)<2 & phaseinsequence(2:end)>=2)+1;
ind_end = find(phaseinsequence(1:end-1)==2 & phaseinsequence(2:end)==3);
% save ind_begin;
% save ind_end;
signal_slice = signal(ind_begin(2):ind_end(2),:);
signal_slice_1 = filter(BP_4_35_n96,1,signal_slice(:,parms.Channel));
% signal_slice_1 = signal_filtered(ind_begin(10):ind_end(10),:);
% signal_slice_len = length(signal_slice_1);


%%	% FFT1 for ssvep
Amp_FFT_p_Set = zeros();
w1 = ones(channel_num,1);% Weight equal
% w1(1)=0.8;
% w1(4) = 1.2;
% w1(5) = 0.6;
% w1(6) = 0.6;
signal_slice_1 = signal_slice_1*w1;
[Point_Hz,Amp] = FFT(signal_slice_1,512,samplingrate);%%%%%%%%blocksize?
amp_band_sum = sum(Amp(4/samplingrate*length(Point_Hz):35/samplingrate*length(Point_Hz)));
for qq = 1:length(frequency)
    FFT_p1 = find(Point_Hz<frequency(qq)+.3&Point_Hz>frequency(qq)-.3);
    FFT_p2 = find(Point_Hz<frequency(qq)*2+.3&Point_Hz>frequency(qq)*2-.3);
    Amp_FFT_p = sum(Amp(FFT_p1))+sum(Amp(FFT_p2));
    Amp_FFT_p_Set(qq) = Amp_FFT_p;
end
Test_maxFFT = max(Amp_FFT_p_Set);
index_FFT = find(Amp_FFT_p_Set == Test_maxFFT);

