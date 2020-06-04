function [signal,state,parms] = getInfo(datfiles,datdir)

% �ú������������ڴ�ָ���ļ��ж�ȡ���ݡ�

fprintf(1,'loading data...\n');
% ��ʼ��state
% statestr = {'PhaseInSequence' 'StimulusType' 'StimulusCode' 'StimulusBegin'};
statestr = {'PhaseInSequence' 'label'};
numstat = length(statestr);
state = struct;
for dd = 1:numstat
    state.(char(statestr(dd))) = [];
end

% �ļ���ȡ����
if isempty(datfiles)
    [datfiles, datdir] = uigetfile('*.dat','Select the P300 ASCII(.dat) data file(s)','multiselect','on');
end
if iscell(datfiles)==0
    datfiles = {datfiles};
end

numdat = length(datfiles);  %�ļ�����

signal = [];


for kk = 1:numdat
    [sig,sts,prm] = load_bcidat([datdir char(datfiles(kk))]);               %��ȡ����
    
    signal = cat(1,signal,sig);                                             %ƴ���ź�
    parms.SoftwareCh_total(kk) = size(signal,2);                            %�ź�ͨ������
    parms.Channel = prm.SSVEPChannel.NumericValue;
    parms.SamplingRate = prm.SamplingRate.NumericValue;                 %�źŲ�����
    parms.Frequency = prm.HzList.NumericValue;
    
    state.PhaseInSequence = cat(1,state.PhaseInSequence,sts.PhaseInSequence);%phase���ƴ��
    state.label = cat(1,state.label,sts.label);%label���ƴ��
    
end

fprintf('...Done\n');
end

