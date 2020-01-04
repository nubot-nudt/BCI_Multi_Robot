clear all
%文件读取
[datfiles, datdir] = ...
    uigetfile('*.dat','Select the BCI2000 SSVEP (.dat) data file(s)','multiselect','on','..\data\');
if datdir == 0, return; end
if ~iscell(datfiles)
    datfiles = {datfiles};
end
datfiles = sort(datfiles);

%信息获取
[signal, state, parms] = getInfo(datfiles, datdir);
channel_num = length(parms.Channel); %数据通道数
samplingrate = parms.SamplingRate;   %采样频率
frequency = parms.Frequency;         %ssvep刺激频率序列
phaseinsequence = state.PhaseInSequence;

% %滤波
% BP_4_35_n96 = fir1(96,[6 35].*2./200);    %滤波器
% % signal_filtered = filter(BP_4_35_n96,1,signal(:,parms.Channel));
% % signal_filtered = signal(:,parms.Channel);
% signal = filter(BP_4_35_n96,1,signal(:,parms.Channel)); %滤波

%另一种滤波
Fs = 200;
n = 3;
MdB = 20;
bprange = [6 35];
Ws = bprange/(Fs/2);
[z,p,k] = cheby2(n, MdB, Ws);
[sos,g] = zp2sos(z,p,k);
BP_filter = dfilt.df2sos(sos,g);
signal = filter(BP_filter,signal(:,parms.Channel));


%闪烁数据段截取
ind_begin = find(phaseinsequence(1:end-1)<2 & phaseinsequence(2:end)>=2)+1;    %数据起始点
ind_end = find(phaseinsequence(1:end-1)==2 & phaseinsequence(2:end)==3);       %数据终止点
data_num = size(ind_begin,1);                                              %总任务数（闪了多少次）
length_all = ind_end-ind_begin;                                            %各段的长度
length_max = max(length_all);                                              %最长长度
t = (1:length_max)'/samplingrate;                                          %构造ssvep拟合信号Y
Y=struct;
for f = 1:size(frequency,1)
    y = [sin(2*pi*frequency(f)*t),cos(2*pi*frequency(f)*t),sin(4*pi*frequency(f)*t),cos(4*pi*frequency(f)*t),sin(6*pi*frequency(f)*t),cos(6*pi*frequency(f)*t)];
    %     y = [sin(2*pi*frequency(f)*t),cos(2*pi*frequency(f)*t),sin(4*pi*frequency(f)*t),cos(4*pi*frequency(f)*t)];
    Y(f).frequency = y;
end

%计算各时间正确率

%读入标签
label = double(state.label(ind_begin));
calculate_num = 5/0.1;
result_all = zeros(data_num,calculate_num);
score_all = [];
result_score_record = [];
nontarget_score_offline = zeros(length(frequency),calculate_num);
% nontarget_score_offline_1 = zeros(length(frequency),calculate_num);
% load nontarget_score_offline_1;
nn = 0;
for num = 1:data_num
    signal_slice = signal(ind_begin(num):ind_end(num),:);
    slice_length  = length_all(num);
    result_score_temp = [];
    %     nontarget_score_temp = [];
    for time_num = 1:calculate_num
        calculate_length=round((0.1/5*time_num)*slice_length);
        calculate_slice = signal_slice(1:calculate_length,:);
        score = zeros(1,size(frequency,1));
        for ff = 1:size(frequency,1)
            y = Y(ff).frequency(1:calculate_length,:);
            [A,~,R] = canoncorr(calculate_slice,y);        %cca
            score(ff) = max(R);           
%             score(ff) = double((score(ff)-nontarget_score_offline(ff,time_num))/(score(ff)+nontarget_score_offline(ff,time_num)));
%             score(ff) = double((score(ff)-nontarget_score_offline_1(ff,time_num))/(score(ff)+nontarget_score_offline_1(ff,time_num)));
%             if ff ~= label(num)
%                 nontarget_score_offline_1(ff,time_num) = nontarget_score_offline_1(ff,time_num) + score(ff);
%                 nn = nn + 1;
%             end                       
        end
        nontarget_score_offline(label(num),time_num) = nontarget_score_offline(label(num),time_num)+sum(score)-score(label(num));

        %         result_score = max(score);
        %         result_target = find(score==result_score);
        max_score = max(score);
        result_score = max_score;
        result_target = find(score==max_score);
        result_all(num,time_num) = result_target;
        result_score_temp = [result_score_temp,result_score];
    end
    result_score_record = [result_score_record;result_score_temp];
end
nontarget_score_offline = nontarget_score_offline/(data_num*(length(frequency)-1)/length(frequency));
% nontarget_score_offline_1 = nontarget_score_offline_1/(5*10);

% load label_lyr
label = repmat(label,1,calculate_num);
temp = result_all-label;
accuracy = sum(temp==0)/data_num;   %准确率计算
xx = 0.1:0.1:5;                     %时间轴
B = log2(length(frequency))+accuracy.*log2(accuracy)+(1-accuracy).*log2((1-accuracy)/2);           %计算ITR
ITR = B*60./(2+xx);

%保存数据
% save accuracy_sjp accuracy;
% save ITR_sjp ITR

%画图
figure, plot(xx,accuracy,'r','linewidth',2.5);
figure, plot(xx,ITR,'b','linewidth',2.5);
figure,plot(xx,result_score_record)
% figure,plot(1:25,score_all(1:25,:));
% signal_slice_1 = filter(BP_4_35_n96,1,signal_slice(:,parms.Channel));

% %%	% FFT1 for ssvep
% Amp_FFT_p_Set = zeros();
% w1 = ones(channel_num,1);% Weight equal
% signal_slice_1 = signal_slice_1*w1;
% [Point_Hz,Amp] = FFT(signal_slice_1,512,samplingrate);%%%%%%%%blocksize?
% amp_band_sum = sum(Amp(4/samplingrate*length(Point_Hz):35/samplingrate*length(Point_Hz)));
% for qq = 1:length(frequency)
%     FFT_p1 = find(Point_Hz<frequency(qq)+.3&Point_Hz>frequency(qq)-.3);
%     FFT_p2 = find(Point_Hz<frequency(qq)*2+.3&Point_Hz>frequency(qq)*2-.3);
%     Amp_FFT_p = sum(Amp(FFT_p1))+sum(Amp(FFT_p2));
%     Amp_FFT_p_Set(qq) = Amp_FFT_p;
% end
% Test_maxFFT = max(Amp_FFT_p_Set);
% index_FFT = find(Amp_FFT_p_Set == Test_maxFFT);

