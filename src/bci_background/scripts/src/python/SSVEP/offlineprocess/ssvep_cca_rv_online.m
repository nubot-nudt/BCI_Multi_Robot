clear all
%�ļ���ȡ
% [datfiles, datdir] = ...
%     uigetfile('*.dat','Select the BCI2000 SSVEP (.dat) data file(s)','multiselect','on','..\data\');
% if datdir == 0, return; end
% if ~iscell(datfiles)
%     datfiles = {datfiles};
% end
% datfiles = sort(datfiles);
% 
% %��Ϣ��ȡ
% [signal, state, parms] = getInfo(datfiles, datdir);
load('lxbdata.mat');
channel_num = length(parms.Channel);
samplingrate = parms.SamplingRate;
frequency = parms.Frequency;
phaseinsequence = state.PhaseInSequence;

% %�˲�
% BP_4_35_n96 = fir1(96,[6 35].*2./200);    %�˲���
% % signal_filtered = filter(BP_4_35_n96,1,signal(:,parms.Channel));
% % signal_filtered = signal(:,parms.Channel);
% signal = filter(BP_4_35_n96,1,signal(:,parms.Channel)); %�˲�

%�˲�
Fs = samplingrate;
n = 3;
MdB = 20;
bprange = [6 35];
Ws = bprange/(Fs/2);
[z,p,k] = cheby2(n, MdB, Ws);
[sos,g] = zp2sos(z,p,k);
BP_filter = dfilt.df2sos(sos,g);
signal = filter(BP_filter,signal(:,parms.Channel));

%��˸��ݶν�ȡ
ind_begin = find(phaseinsequence(1:end-1)<2 & phaseinsequence(2:end)>=2)+1;    %��ʼ��
ind_end = find(phaseinsequence(1:end-1)==2 & phaseinsequence(2:end)==3);       
data_num = size(ind_begin,1);                                              %��������
length_all = ind_end-ind_begin;                                            %���εĳ���
length_max = max(length_all);                                              
t = (1:length_max)'/samplingrate;                                          %��������ź�Y
Y=struct;
for f = 1:size(frequency,1)
    y = [sin(2*pi*frequency(f)*t),cos(2*pi*frequency(f)*t),sin(4*pi*frequency(f)*t),cos(4*pi*frequency(f)*t),sin(6*pi*frequency(f)*t),cos(6*pi*frequency(f)*t)];
    Y(f).frequency = y;
end

%�����ǩ
label = double(state.label(ind_begin));
detect_time_record = [];
result_target_temp = [];
accuracy_record = [];
detect_time_avg_record = [];
score1_record = [];
load nontarget_score_offline;
% for score_threshold = 0:0.05:1
for score_threshold = 0.25
    correct_num = 0;
    n = 0;
    detect_time = zeros(1,data_num);
%     for num = 1:data_num
      for num = 1:10
        label_current = label(num);
        result_target_temp = [];
%         signal_slice = signal(ind_begin(num):ind_end(num),:);
        signal_slice = signal(1000*num:1000*(num+1),:);
        slice_length  = length_all(num);
        result_target_new = -1;
        for calculate_len = 100:20:900 %500ms-slice_length,step:100ms
            calculate_slice = signal_slice(1:calculate_len,:);
            score1 = zeros(1,size(frequency,1));
            score = zeros(1,size(frequency,1));
            for ff = 1:size(frequency,1)
                y = Y(ff).frequency(1:calculate_len,:);
                [A,~,R] = canoncorr(calculate_slice,y); 
                score1(ff) = max(R);
                score(ff) = double((score1(ff)-nontarget_score_offline(ff,round(calculate_len/20)))/(score1(ff)+nontarget_score_offline(ff,round(calculate_len/20))));               
            end
            result_score = max(score);
            result_target = find(score==result_score);
            result_target_new = result_target;            
            if result_score > score_threshold
                n = n+1;
                break;
            end
        end
        score1_record = [score1_record;score1];
        detect_time(1,num) = calculate_len;
        if result_target_new == label_current
            correct_num = correct_num + 1;
        end
    end
%     result_record = [score1_record,state.label(ind_begin)];
    accuracy = correct_num/data_num;
    detect_time_avg = mean(detect_time);
    detect_time_record = [detect_time;detect_time];
    accuracy_record = [accuracy_record,accuracy];
    detect_time_avg_record = [detect_time_avg_record,detect_time_avg];
end

% ����ITR
B = log2(length(frequency))+accuracy_record.*log2(accuracy_record)+(1-accuracy_record).*log2((1-accuracy_record)/(length(frequency)-1));  
ITR = B*60./(detect_time_avg_record*5/1000);
% %��ͼ
% figure, plot(0:0.05:1,ITR,'b','linewidth',2.5);
% figure, plot(0:0.05:1,accuracy_record,'r','linewidth',2.5);






