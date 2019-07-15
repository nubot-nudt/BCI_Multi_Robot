function [signal,state,parms] = getInfo(datfiles,datdir)
% anotated by mrtang
% 该函数的作用在于从指定文件中读取信号等。

fprintf(1,'loading data...\n');
% 初始化state
% statestr = {'PhaseInSequence' 'StimulusType' 'StimulusCode' 'StimulusBegin'};
statestr = {'PhaseInSequence'};
numstat = length(statestr);
state = struct;
for dd = 1:numstat
    state.(char(statestr(dd))) = [];
end

% 文件读取控制
if isempty(datfiles)
    [datfiles, datdir] = uigetfile('*.dat','Select the P300 ASCII(.dat) data file(s)','multiselect','on');
end
if iscell(datfiles)==0
    datfiles = {datfiles};
end

numdat = length(datfiles);  %文件数量

signal = [];


for kk = 1:numdat
    [sig,sts,prm] = load_bcidat([datdir char(datfiles(kk))]);               %读取数据
    
    signal = cat(1,signal,sig);                                             %拼接信号
    parms.SoftwareCh_total(kk) = size(signal,2);                            %信号通道数量
    parms.Channel = prm.SSVEPChannel.NumericValue;
    parms.SamplingRate = prm.SamplingRate.NumericValue;                 %信号采样率
    parms.Frequency = prm.HzList.NumericValue;
    
    state.PhaseInSequence = cat(1,state.PhaseInSequence,sts.PhaseInSequence);%phase标记拼接
    
end

fprintf('...Done\n');
end

