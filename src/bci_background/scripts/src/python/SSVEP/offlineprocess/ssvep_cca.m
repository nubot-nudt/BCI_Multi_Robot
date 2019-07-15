clear all

% [datfiles, datdir] = ...
%     uigetfile('*.dat','Select the BCI2000 SSVEP (.dat) data file(s)','multiselect','on','..\data\');
% if datdir == 0, return; end
% if ~iscell(datfiles)
%     datfiles = {datfiles};
% end
% datfiles = sort(datfiles);
% 
% 
% [signal, state, parms] = getInfo(datfiles, datdir);
load('lxbdata.mat');
channel_num = length(parms.Channel);
samplingrate = parms.SamplingRate;   
frequency = parms.Frequency;        
phaseinsequence = state.PhaseInSequence;

% %
% BP_4_35_n96 = fir1(96,[6 35].*2./200);    %
% % signal_filtered = filter(BP_4_35_n96,1,signal(:,parms.Channel));
% % signal_filtered = signal(:,parms.Channel);
% signal = filter(BP_4_35_n96,1,signal(:,parms.Channel)); %


Fs = samplingrate;
n = 3;
MdB = 20;
bprange = [6 35];
Ws = bprange/(Fs/2);
[z,p,k] = cheby2(n, MdB, Ws);
[sos,g] = zp2sos(z,p,k);
BP_filter = dfilt.df2sos(sos,g);
signal = filter(BP_filter,signal(:,parms.Channel));


ind_begin = find(phaseinsequence(1:end-1)<2 & phaseinsequence(2:end)>=2)+1;    %
ind_end = find(phaseinsequence(1:end-1)==2 & phaseinsequence(2:end)==3);       
data_num = size(ind_begin,1);                                              %
length_all = ind_end-ind_begin;                                            %
length_max = max(length_all);                                              
t = (1:length_max)'/samplingrate;                                          %
Y=struct;
for f = 1:size(frequency,1)
    y = [sin(2*pi*frequency(f)*t),cos(2*pi*frequency(f)*t),sin(4*pi*frequency(f)*t),cos(4*pi*frequency(f)*t),sin(6*pi*frequency(f)*t),cos(6*pi*frequency(f)*t)];
    Y(f).frequency = y; 
end
 

label = double(state.label(ind_begin));
calculate_num = 5/0.2;
result_all = zeros(data_num,calculate_num);
score_all = [];

for num = 1:data_num  
    signal_slice = signal(ind_begin(num):ind_end(num),:);
    slice_length  = length_all(num);
    for time_num = 1:calculate_num  
        calculate_length=round((0.2/5*time_num)*slice_length);      
        calculate_slice = signal_slice(1:calculate_length,:);
        score = zeros(1,size(frequency,1));
        for ff = 1:size(frequency,1)         
            y = Y(ff).frequency(1:calculate_length,:);
            [A,~,R] = canoncorr(calculate_slice,y);   
            score(ff) = max(R);            
        end
        score_all = [score_all;score];
        result_score = max(score);
        result_target = find(score==result_score);
        
        result_all(num,time_num) = result_target;
    end
end
        
% load label_lyr
label = repmat(label,1,calculate_num);
temp = result_all-label;
accuracy = sum(temp==0)/data_num;   
xx = 0.2:0.2:5;                     
B = log2(length(frequency))+accuracy.*log2(accuracy)+(1-accuracy).*log2((1-accuracy)/5);           %
ITR = B*60./(2+xx);


figure, plot(xx,accuracy,'r','linewidth',2.5);
figure, plot(xx,ITR,'b','linewidth',2.5);
figure,plot(1:25,score_all(1:25,:));